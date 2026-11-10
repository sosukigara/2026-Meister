package com.meister.vision

import ai.onnxruntime.OnnxTensor
import ai.onnxruntime.OrtEnvironment
import ai.onnxruntime.OrtSession
import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.util.Log
import java.nio.FloatBuffer

// YOLOv8n ONNX を onnxruntime-mobile で動かす実装。
// 前処理→推論→後処理の順序は PC 側 yolo_detector.cpp と 1:1。
class YoloDetector(
    context: Context,
    // assets 内のモデル名。640 入力 ([1,3,640,640]) を実測済み。
    modelName: String = "yolov8n.onnx",
    private val threads: Int = 4,
) : Detector {
    companion object {
        private const val TAG = "YoloDetector"
        // onnx load で実測した入力形 [1,3,640,640]。変えたら Ultralytics 再 export が要る。
        const val INPUT_SIZE = 640
        const val IOU_THRESHOLD = 0.45f
        const val PAD_COLOR = 114
    }

    private val env: OrtEnvironment = OrtEnvironment.getEnvironment()
    private val session: OrtSession
    private val inputName: String

    init {
        check(modelName.isNotBlank()) { "modelName is blank" }
        val modelBytes = try {
            context.assets.open(modelName).use { it.readBytes() }
        } catch (e: Exception) {
            env.close()
            throw IllegalStateException("model asset missing: $modelName", e)
        }
        val opts = OrtSession.SessionOptions()
        opts.setIntraOpNumThreads(threads)
        session = env.createSession(modelBytes, opts)
        opts.close()
        inputName = session.inputNames.firstOrNull() ?: "images"
    }

    override fun detect(input: Bitmap, threshold: Float): List<Detection> {
        val lb = YoloPostprocess.letterboxMath(input.width, input.height, INPUT_SIZE)
        val padded = Bitmap.createBitmap(INPUT_SIZE, INPUT_SIZE, Bitmap.Config.ARGB_8888)
        padded.eraseColor(Color.rgb(PAD_COLOR, PAD_COLOR, PAD_COLOR))
        val scaled = if (lb.newW == input.width && lb.newH == input.height) {
            input
        } else {
            Bitmap.createScaledBitmap(input, lb.newW, lb.newH, true)
        }
        Canvas(padded).drawBitmap(scaled, lb.padW.toFloat(), lb.padH.toFloat(), null)
        if (scaled !== input) scaled.recycle()

        val n = INPUT_SIZE * INPUT_SIZE
        val data = FloatArray(3 * n)
        val pixels = IntArray(n)
        padded.getPixels(pixels, 0, INPUT_SIZE, 0, 0, INPUT_SIZE, INPUT_SIZE)
        for (i in 0 until n) {
            val p = pixels[i]
            data[i] = ((p shr 16) and 0xFF) / 255f
            data[n + i] = ((p shr 8) and 0xFF) / 255f
            data[2 * n + i] = (p and 0xFF) / 255f
        }

        val channels: Array<FloatArray> = try {
            OnnxTensor.createTensor(env, FloatBuffer.wrap(data), longArrayOf(1, 3, INPUT_SIZE.toLong(), INPUT_SIZE.toLong())).use { tensor ->
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
        session.close()
        env.close()
    }
}
