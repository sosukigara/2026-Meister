package com.meister.skeleton

// 推論 FPS と推論 ms の移動平均。20Hz 検証の証拠表示用。
class FpsCounter(private val windowSize: Int = 30) {
    private val intervalsNs = ArrayDeque<Long>()
    private var lastNs: Long = -1
    private var lastMs: Long = 0

    @Synchronized
    fun tick(inferenceMs: Long) {
        val now = System.nanoTime()
        if (lastNs > 0) {
            intervalsNs.addLast(now - lastNs)
            while (intervalsNs.size > windowSize) intervalsNs.removeFirst()
        }
        lastNs = now
        lastMs = inferenceMs
    }

    @Synchronized
    fun fps(): Double {
        if (intervalsNs.isEmpty()) return 0.0
        val avgNs = intervalsNs.average()
        if (avgNs <= 0) return 0.0
        return 1e9 / avgNs
    }

    @Synchronized
    fun lastInferenceMs(): Long = lastMs
}
