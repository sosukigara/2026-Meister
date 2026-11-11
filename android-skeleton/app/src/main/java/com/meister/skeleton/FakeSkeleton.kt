package com.meister.skeleton

import android.graphics.RectF

// モデル無しで配線を通すための仮実装。グー→チョキ→パーを巡回する仮21点。
class FakeSkeleton {
    private var tick = 0

    companion object {
        const val DUMMY_SCORE = 0.9f
    }

    fun label(): String = "FAKE"

    fun detectHands(width: Int, height: Int, threshold: Float): List<HandResult> {
        if (DUMMY_SCORE < threshold) return emptyList()
        val janken = when ((tick++ % 3)) {
            0 -> Janken.GOO
            1 -> Janken.CHOKI
            else -> Janken.PAR
        }
        // 画面中央の固定21点。分類器は通さずラベル直指定 (配線確認が目的)。
        val cx = 0.5f
        val cy = 0.5f
        val points = List(21) { i -> HandPoint(cx + (i % 5) * 0.02f, cy + (i / 5) * 0.02f) }
        val w = width.toFloat()
        val h = height.toFloat()
        return listOf(
            HandResult(points, janken, DUMMY_SCORE, RectF(w * 0.3f, h * 0.3f, w * 0.7f, h * 0.7f)),
        )
    }
}
