package com.meister.skeleton

import android.graphics.RectF

// 1手分の結果。points は正規化21点 (0.0-1.0、MediaPipe規約の添字順)。
// bbox は解析画像px基準 (オーバーレイのラベル位置用)。
data class HandResult(
    val points: List<HandPoint>,
    val janken: Janken,
    val score: Float,
    val bbox: RectF,
)
