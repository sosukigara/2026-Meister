package com.meister.skeleton

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File
import java.nio.file.Files

class CpuMonitorTest {

    private fun dir(): File = Files.createTempDirectory("cpumon").toFile()

    @Test fun firstSample_returnsNaN() {
        val d = dir()
        val self = File(d, "self").apply { writeText("1 (app) R 0 0 0 0 0 0 0 0 0 0 100 50 0 0") }
        var wall = 0L
        assertTrue(CpuMonitor(self, 100L, { wall }).sample().isNaN())
    }

    @Test fun secondSample_returnsPercent() {
        val d = dir()
        // proc: 150tick→250tick (+1.0s分)。wall: +1s。期待 100%。
        val self = File(d, "self").apply { writeText("1 (app) R 0 0 0 0 0 0 0 0 0 0 100 50 0 0") }
        var wall = 0L
        val mon = CpuMonitor(self, 100L, { wall })
        mon.sample()
        self.writeText("1 (my app) R 0 0 0 0 0 0 0 0 0 0 180 70 0 0")
        wall = 1_000_000_000L
        assertEquals(100f, mon.sample(), 0.01f)
    }

    @Test fun quarterLoad_returns25() {
        val d = dir()
        val self = File(d, "self").apply { writeText("1 (app) R 0 0 0 0 0 0 0 0 0 0 100 50 0 0") }
        var wall = 0L
        val mon = CpuMonitor(self, 100L, { wall })
        mon.sample()
        self.writeText("1 (app) R 0 0 0 0 0 0 0 0 0 0 120 55 0 0")
        wall = 1_000_000_000L
        assertEquals(25f, mon.sample(), 0.01f)
    }
}
