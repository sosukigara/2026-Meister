package com.meister.vision

import ai.onnxruntime.OnnxTensor
import ai.onnxruntime.OrtEnvironment
import ai.onnxruntime.OrtSession
import ai.onnxruntime.TensorInfo
import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.util.Log
import java.nio.FloatBuffer

// YOLOv8n ONNX を onnxruntime(-android) で動かす実装。
// 前処理→推論→後処理の順序は PC 側 yolo_detector.cpp と 1:1。
class YoloDetector(
    context: Context,
    // assets 内のモデル名。320 が既定 (Ultralytics 公式 export、640 より約4倍速い)。
    modelName: String = "yolov8n-320.onnx",
    private val threads: Int = 4,
    // NNAPI を試し、作れなければ CPU に落とす。実機実測で優劣を決める。
    useNnapi: Boolean = true,
) : Detector {
    companion object {
        private const val TAG = "YoloDetector"
        // モデル入力が正方形でない・読めない時の fallback。onnx 実測では 640。
        const val DEFAULT_INPUT_SIZE = 640
        const val IOU_THRESHOLD = 0.45f
        const val PAD_COLOR = 114

        // OrtEnvironment はプロセス単一。閉じると作り直しになるので使い回し、閉じない。
        private val sharedEnv: OrtEnvironment by lazy { OrtEnvironment.getEnvironment() }

        private fun YoloDetector.createSession(
            modelBytes: ByteArray,
            threads: Int,
            nnapi: Boolean,
        ): OrtSession {
            val opts = OrtSession.SessionOptions()
            try {
                opts.setIntraOpNumThreads(threads)
                if (nnapi) opts.addNnapi()
                return env.createSession(modelBytes, opts)
            } finally {
                opts.close()
            }
        }
    }

    private val env: OrtEnvironment = sharedEnv
    private val session: OrtSession
    private val inputName: String
    // 入力解像度はモデルから取る (640/320 どちらも同じ経路。PC 側と同じ)。
    val inputSize: Int
    // 実際に使えた EP。HUD/log の表示用。
    val usingNnapi: Boolean
    // 内訳計測用。呼び出し側が読む (interface は変えない)。
    @Volatile var lastPreMs: Long = 0
    @Volatile var lastInferMs: Long = 0

    init {
        check(modelName.isNotBlank()) { "modelName is blank" }
        val modelBytes = try {
            context.assets.open(modelName).use { it.readBytes() }
        } catch (e: Exception) {
            throw IllegalStateException("model asset missing: $modelName", e)
        }
        val created = try {
            createSession(modelBytes, threads, nnapi = useNnapi) to useNnapi
        } catch (e: Exception) {
            Log.w(TAG, "NNAPI session failed, falling back to CPU", e)
            createSession(modelBytes, threads, nnapi = false) to false
        }
        session = created.first
        usingNnapi = created.second
        inputName = session.inputNames.firstOrNull() ?: "images"
        val shape = (session.inputInfo[inputName]?.info as? TensorInfo)?.shape
        inputSize = if (shape != null && shape.size == 4 && shape[0] == 1L &&
            shape[1] == 3L && shape[2] == shape[3] && shape[2] > 0
        ) {
            shape[2].toInt()
        } else {
            DEFAULT_INPUT_SIZE
        }
    }

    override fun label(): String = "YOLOv8n-$inputSize+" + if (usingNnapi) "NNAPI" else "CPU"

    // 初回推論を通して session が実働することを保証する。重いのでバックグラウンドで呼ぶ。
    // 失敗したら例外→呼び出し側が Fake に落とす。画面に理由が出る。
    fun validate() {
        val n = inputSize * inputSize
        val zeros = FloatBuffer.wrap(FloatArray(3 * n))
        val shape = longArrayOf(1, 3, inputSize.toLong(), inputSize.toLong())
        OnnxTensor.createTensor(env, zeros, shape).use { tensor ->
            session.run(mapOf(inputName to tensor)).use { result ->
                @Suppress("UNCHECKED_CAST")
                val out = (result[0].value as Array<Array<FloatArray>>)[0]
                require(out.size == YoloPostprocess.CHANNELS) { "bad channels: ${out.size}" }
                require(out[0].isNotEmpty()) { "empty output" }
            }
        }
    }

    override fun detect(input: Bitmap, threshold: Float): List<Detection> {
        val tPre0 = System.nanoTime()
        val lb = YoloPostprocess.letterboxMath(input.width, input.height, inputSize)
        val padded = Bitmap.createBitmap(inputSize, inputSize, Bitmap.Config.ARGB_8888)
        padded.eraseColor(Color.rgb(PAD_COLOR, PAD_COLOR, PAD_COLOR))
        val scaled = if (lb.newW == input.width && lb.newH == input.height) {
            input
        } else {
            Bitmap.createScaledBitmap(input, lb.newW, lb.newH, true)
        }
        Canvas(padded).drawBitmap(scaled, lb.padW.toFloat(), lb.padH.toFloat(), null)
        if (scaled !== input) scaled.recycle()

        val n = inputSize * inputSize
        val data = FloatArray(3 * n)
        val pixels = IntArray(n)
        padded.getPixels(pixels, 0, inputSize, 0, 0, inputSize, inputSize)
        // フレーム毎の Bitmap はここで捨てる。捨てないと 10 分回した時に落ちる。
        padded.recycle()
        for (i in 0 until n) {
            val p = pixels[i]
            data[i] = ((p shr 16) and 0xFF) / 255f
            data[n + i] = ((p shr 8) and 0xFF) / 255f
            data[2 * n + i] = (p and 0xFF) / 255f
        }

        lastPreMs = (System.nanoTime() - tPre0) / 1_000_000
        val tInfer0 = System.nanoTime()
        val channels: Array<FloatArray> = try {
            OnnxTensor.createTensor(env, FloatBuffer.wrap(data), longArrayOf(1, 3, inputSize.toLong(), inputSize.toLong())).use { tensor ->
                session.run(mapOf(inputName to tensor)).use { result ->
                    @Suppress("UNCHECKED_CAST")
                    val out = (result[0].value as Array<Array<FloatArray>>)[0]
                    out
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "inference failed", e)
            return emptyList()
        }
        lastInferMs = (System.nanoTime() - tInfer0) / 1_000_000
        if (channels.size != YoloPostprocess.CHANNELS) {
            Log.e(TAG, "unexpected channels: ${channels.size}")
            return emptyList()
        }
        val gridCells = channels[0].size
        return YoloPostprocess.postprocess(
            channels, gridCells, lb, input.width, input.height, threshold, IOU_THRESHOLD,
        )
    }

    override fun close() {
        // sharedEnv は閉じない (プロセス単一で使い回す)。
        session.close()
    }
}
