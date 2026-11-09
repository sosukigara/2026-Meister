package com.meister.vision

// 将来の差し替え口 (Q19/Q32)。UI はこの interface しか触らない。
// 送信層 (ROS ブリッジ等) を足す時は実装側だけ変える。
interface Detector : AutoCloseable {
    // 同期推論。呼び出し側がバックグラウンドスレッドで呼ぶ。
    // input: RGB 画像。width/height は返却 bbox の座標基準になる。
    fun detect(input: android.graphics.Bitmap, threshold: Float): List<Detection>
}
