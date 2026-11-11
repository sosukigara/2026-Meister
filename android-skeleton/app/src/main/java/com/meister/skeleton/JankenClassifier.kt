package com.meister.skeleton

import kotlin.math.hypot

// 21点→グー/チョキ/パー/不明の純粋関数。Android非依存で単体テストする。
// 添字は MediaPipe 規約 (0=手首 / 1-4=親指 / 5-8=人差し指 / 9-12=中指 / 13-16=薬指 / 17-20=小指)。
object JankenClassifier {
    // Spec §2 の値 verbatim。角度計算はしない (Snapdragon 665 でも毎フレーム回すため)。
    const val FINGER_RATIO = 1.1f
    const val THUMB_RATIO = 0.8f

    fun classify(points: List<HandPoint>): Janken {
        if (points.size < 21) return Janken.UNKNOWN
        val wrist = points[0]

        fun open(tip: Int, joint: Int): Boolean {
            val dTip = hypot(points[tip].x - wrist.x, points[tip].y - wrist.y)
            val dJoint = hypot(points[joint].x - wrist.x, points[joint].y - wrist.y)
            if (dJoint <= 0f) return false
            return dTip > dJoint * FINGER_RATIO
        }

        val indexOpen = open(8, 6)
        val middleOpen = open(12, 10)
        val ringOpen = open(16, 14)
        val pinkyOpen = open(20, 18)

        // 親指は別扱い。グーで埋もれる誤判定を避けるため付け根並みで見る。
        val indexLen = hypot(points[8].x - points[5].x, points[8].y - points[5].y)
        val thumbOpen = hypot(points[4].x - points[5].x, points[4].y - points[5].y) >
            indexLen * THUMB_RATIO

        val fingers = listOf(indexOpen, middleOpen, ringOpen, pinkyOpen).count { it }
        val total = fingers + if (thumbOpen) 1 else 0
        return when {
            total == 0 -> Janken.GOO
            // 親指だけ出た握り拳はグー。指1本だけは不明。
            total == 1 && thumbOpen && fingers == 0 -> Janken.GOO
            total == 2 && indexOpen && middleOpen -> Janken.CHOKI
            total >= 4 -> Janken.PAR
            else -> Janken.UNKNOWN
        }
    }
}
