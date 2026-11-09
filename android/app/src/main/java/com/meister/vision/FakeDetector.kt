package com.meister.vision

import android.graphics.Bitmap

// モデル無しで配線を通すための仮実装。モデル到着までの土台確認用。
class FakeDetector : Detector {
    private var tick = 0

    companion object {
        const val DUMMY_SCORE = 0.6f
    }

    override fun detect(input: Bitmap, threshold: Float): List<Detection> {
        tick++
        // しきい値スライダの動作確認用に固定スコアでフィルタする。
        if (DUMMY_SCORE < threshold) return emptyList()
        val w = input.width.toFloat()
        val h = input.height.toFloat()
        // 画面中央をゆっくり往復する枠。描画・FPS 経路の疎通確認が目的。
        val phase = (tick % 100) / 100f
        val x1 = w * (0.2f + 0.3f * phase)
        return listOf(
            Detection(0, "person", DUMMY_SCORE, x1, h * 0.25f, x1 + w * 0.3f, h * 0.75f),
        )
    }

    override fun close() = Unit
}
