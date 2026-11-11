package com.meister.skeleton

import android.content.Context
import android.graphics.Bitmap
import android.graphics.RectF
import com.google.mediapipe.framework.image.BitmapImageBuilder
import com.google.mediapipe.tasks.core.BaseOptions
import com.google.mediapipe.tasks.core.Delegate
import com.google.mediapipe.tasks.vision.core.RunningMode
import com.google.mediapipe.tasks.vision.handlandmarker.HandLandmarker

// MediaPipe HandLandmarker を包む検出器。呼び出し側がバックグラウンドで呼ぶ。
// 入力Bitmapは ARGB_8888 を想定 (ImageUtil.toRgbBitmap の出力)。
class HandLandmarkerDetector(
    context: Context,
    modelAssetPath: String = MODEL_ASSET,
    numHands: Int = MAX_HANDS,
) : AutoCloseable {
    companion object {
        const val MODEL_ASSET = "hand_landmarker.task"
        const val MAX_HANDS = 2
        private const val TAG = "HandLandmarkerDetector"

        private fun createLandmarker(
            context: Context,
            modelAssetPath: String,
            numHands: Int,
            delegate: Delegate,
        ): HandLandmarker {
            val base = BaseOptions.builder()
                .setModelAssetPath(modelAssetPath)
                .setDelegate(delegate)
                .build()
            // VIDEO固定。初回は検出、以降は追跡になる。
            // LIVE_STREAM はコールバック管理が要るので持たない。
            val options = HandLandmarker.HandLandmarkerOptions.builder()
                .setBaseOptions(base)
                .setRunningMode(RunningMode.VIDEO)
                .setNumHands(numHands)
                .build()
            return HandLandmarker.createFromOptions(context, options)
        }
    }

    private val landmarker: HandLandmarker
    val delegateName: String

    init {
        // assets に無ければ即例外→呼び出し側が FakeSkeleton に落とす。画面に理由が出る。
        try {
            context.assets.open(modelAssetPath).close()
        } catch (e: Exception) {
            throw IllegalStateException("model asset missing: $modelAssetPath", e)
        }
        // 高速な順に試す (NPU→GPU→CPU)。作れなければ次に落とす。
        // 速度の優劣は実機実測で決める (速い順の仮置き。遅ければ戻す)。
        val chosen = try {
            createLandmarker(context, modelAssetPath, numHands, Delegate.NPU) to "NPU"
        } catch (e1: Exception) {
            android.util.Log.w(TAG, "NPU delegate unavailable, trying GPU", e1)
            try {
                createLandmarker(context, modelAssetPath, numHands, Delegate.GPU) to "GPU"
            } catch (e2: Exception) {
                android.util.Log.w(TAG, "GPU delegate unavailable, using CPU", e2)
                createLandmarker(context, modelAssetPath, numHands, Delegate.CPU) to "CPU"
            }
        }
        landmarker = chosen.first
        delegateName = chosen.second
    }

    fun label(): String = "HandLandmarker+$delegateName"

    // 同期推論。空=手なし (正常系)。
    // timestampMs は単調増加のフレーム時刻 (VIDEO追跡用)。CameraX の timestamp をそのまま渡す。
    fun detectHands(input: Bitmap, threshold: Float, timestampMs: Long): List<HandResult> {
        val mpImage = BitmapImageBuilder(input).build()
        try {
            val result = landmarker.detectForVideo(mpImage, timestampMs)
            return mapResult(result, threshold, input.width, input.height)
        } finally {
            mpImage.close()
        }
    }

    private fun mapResult(
        result: com.google.mediapipe.tasks.vision.handlandmarker.HandLandmarkerResult,
        threshold: Float,
        width: Int,
        height: Int,
    ): List<HandResult> {
        val out = ArrayList<HandResult>(result.landmarks().size)
        for ((i, landmarks) in result.landmarks().withIndex()) {
            if (landmarks.size < 21) continue
            val score = result.handedness().getOrNull(i)?.firstOrNull()?.score() ?: 0f
            if (score < threshold) continue
            val points = landmarks.map { HandPoint(it.x(), it.y()) }
            var left = Float.MAX_VALUE
            var top = Float.MAX_VALUE
            var right = -Float.MAX_VALUE
            var bottom = -Float.MAX_VALUE
            for (p in points) {
                if (p.x < left) left = p.x
                if (p.y < top) top = p.y
                if (p.x > right) right = p.x
                if (p.y > bottom) bottom = p.y
            }
            out.add(
                HandResult(
                    points = points,
                    janken = JankenClassifier.classify(points),
                    score = score,
                    bbox = RectF(
                        left * width,
                        top * height,
                        right * width,
                        bottom * height,
                    ),
                ),
            )
            if (out.size >= MAX_HANDS) break
        }
        return out
    }

    override fun close() {
        landmarker.close()
    }
}
