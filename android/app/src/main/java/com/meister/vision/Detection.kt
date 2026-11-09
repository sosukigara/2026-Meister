package com.meister.vision

// 1 検出枠。座標は解析画像 px 基準 (YOLO 後処理の出力系と一致させる)。
data class Detection(
    val classId: Int,
    val className: String,
    val score: Float,
    val x1: Float,
    val y1: Float,
    val x2: Float,
    val y2: Float,
)
