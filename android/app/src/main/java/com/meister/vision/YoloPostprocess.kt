package com.meister.vision

import kotlin.math.max
import kotlin.math.min

// PC 側 YoloPostprocess / Letterbox / NMS の移植。Android 非依存で単体テスト可能。
object YoloPostprocess {
    const val NUM_CLASSES = 80
    const val BBOX_COLS = 4
    const val CHANNELS = NUM_CLASSES + BBOX_COLS

    data class LetterboxMath(
        val ratio: Double,
        val padW: Int,
        val padH: Int,
        val newW: Int,
        val newH: Int,
    )

    // letterbox.cpp と同じ式。丸めは banker's rounding (Math.rint)。
    fun letterboxMath(width: Int, height: Int, side: Int): LetterboxMath {
        require(width > 0 && height > 0 && side > 0)
        val ratio = min(side.toDouble() / height, side.toDouble() / width)
        val newH = Math.rint(height * ratio).toInt()
        val newW = Math.rint(width * ratio).toInt()
        return LetterboxMath(ratio, (side - newW) / 2, (side - newH) / 2, newW, newH)
    }

    // output: [84][gridCells]。bbox は side 座標、返却は元画像座標。
    fun postprocess(
        output: Array<FloatArray>,
        gridCells: Int,
        lb: LetterboxMath,
        imageWidth: Int,
        imageHeight: Int,
        confThreshold: Float,
        iouThreshold: Float,
    ): List<Detection> {
        require(output.size == CHANNELS)
        require(output.all { it.size == gridCells })
        // クラス表と定数の対応がずれると class_id が範囲外になる。増減は両方同時に。
        require(COCO_CLASSES.size == NUM_CLASSES)
        require(lb.ratio > 0.0)

        data class Candidate(val det: Detection, val ix1: Float, val iy1: Float, val ix2: Float, val iy2: Float)
        val candidates = mutableListOf<Candidate>()
        for (cell in 0 until gridCells) {
            var bestClass = 0
            var bestScore = output[BBOX_COLS][cell]
            for (c in 1 until NUM_CLASSES) {
                val s = output[BBOX_COLS + c][cell]
                if (s > bestScore) {
                    bestScore = s
                    bestClass = c
                }
            }
            if (bestScore < confThreshold) continue

            val cx = output[0][cell]
            val cy = output[1][cell]
            val w = output[2][cell]
            val h = output[3][cell]
            val x1 = ((cx - w / 2 - lb.padW) / lb.ratio).toFloat().coerceIn(0f, imageWidth.toFloat())
            val y1 = ((cy - h / 2 - lb.padH) / lb.ratio).toFloat().coerceIn(0f, imageHeight.toFloat())
            val x2 = ((cx + w / 2 - lb.padW) / lb.ratio).toFloat().coerceIn(0f, imageWidth.toFloat())
            val y2 = ((cy + h / 2 - lb.padH) / lb.ratio).toFloat().coerceIn(0f, imageHeight.toFloat())
            candidates.add(
                Candidate(
                    Detection(bestClass, COCO_CLASSES[bestClass], bestScore, x1, y1, x2, y2),
                    x1, y1, x2, y2,
                ),
            )
        }
        // NMS はスコア降順で採用。その順がそのまま出力順 (C++ と同じ)。
        val order = candidates.indices.sortedByDescending { candidates[it].det.score }
        val kept = mutableListOf<Detection>()
        val suppressed = BooleanArray(candidates.size)
        for (i in order) {
            if (suppressed[i]) continue
            kept.add(candidates[i].det)
            val a = candidates[i]
            for (j in order) {
                if (j == i || suppressed[j]) continue
                val b = candidates[j]
                if (iou(a.ix1, a.iy1, a.ix2, a.iy2, b.ix1, b.iy1, b.ix2, b.iy2) > iouThreshold) {
                    suppressed[j] = true
                }
            }
        }
        return kept
    }

    private fun iou(
        ax1: Float, ay1: Float, ax2: Float, ay2: Float,
        bx1: Float, by1: Float, bx2: Float, by2: Float,
    ): Float {
        val interX1 = max(ax1, bx1)
        val interY1 = max(ay1, by1)
        val interX2 = min(ax2, bx2)
        val interY2 = min(ay2, by2)
        val inter = max(0f, interX2 - interX1) * max(0f, interY2 - interY1)
        if (inter <= 0f) return 0f
        val union = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
        if (union <= 0f) return 0f
        return inter / union
    }
}
