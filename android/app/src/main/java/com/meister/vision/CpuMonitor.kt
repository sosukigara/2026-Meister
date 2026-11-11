package com.meister.vision

import java.io.File

// 自プロセスのCPU使用率 (top 表示相当。マルチコアで 100% を超えることがある)。
// /proc/stat(全体) は端末の SELinux 次第で読めないため使わない。
// 自プロセス tick 差分 ÷ 実時間で求める。/proc/self/stat は自プロセスなので読める。
// HUD更新は1Hzで呼ぶ。
class CpuMonitor(
    private val selfStat: File = File("/proc/self/stat"),
    private val clockTick: Long = CLK_TICK,
    private val wallNs: () -> Long = System::nanoTime,
) {
    companion object {
        // getconf CLK_TCK。実機 (Snapdragon 665) で 100 を確認。
        const val CLK_TICK = 100L
    }

    private var lastProcTicks = -1L
    private var lastWallNs = -1L

    // 初回は差分が無いので NaN を返す。2回目以降は %。
    fun sample(): Float {
        val proc = readProcTicks() ?: return Float.NaN
        val wall = wallNs()
        if (lastProcTicks < 0 || lastWallNs < 0 || wall <= lastWallNs) {
            lastProcTicks = proc
            lastWallNs = wall
            return Float.NaN
        }
        val procSec = (proc - lastProcTicks).toDouble() / clockTick
        val wallSec = (wall - lastWallNs).toDouble() / 1_000_000_000.0
        lastProcTicks = proc
        lastWallNs = wall
        if (wallSec <= 0.0 || procSec < 0.0) return Float.NaN
        return (100.0 * procSec / wallSec).toFloat().coerceAtLeast(0f)
    }

    private fun readProcTicks(): Long? = try {
        // comm が括弧を含む場合に備え、最後の ')' 以降で割る。utime=14, stime=15。
        val after = selfStat.readText().substringAfterLast(')').trim().split("\\s+".toRegex())
        after[11].toLong() + after[12].toLong()
    } catch (e: Exception) {
        null
    }
}
