package com.meister.skeleton

// じゃんけんの4値。表示名は日本語ラベルそのまま。
enum class Janken(val label: String) {
    GOO("グー"),
    CHOKI("チョキ"),
    PAR("パー"),
    UNKNOWN("不明"),
}

// 正規化21点の1点 (0.0-1.0)。android.graphics 直参照を避け、単体テスト可能にする。
data class HandPoint(val x: Float, val y: Float)
