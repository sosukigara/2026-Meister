package com.meister.vision

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class FpsCounterTest {

    @Test
    fun emptyReturnsZero() {
        assertEquals(0.0, FpsCounter().fps(), 0.0)
    }

    @Test
    fun tickRecordsInferenceMs() {
        val counter = FpsCounter()
        counter.tick(12)
        counter.tick(14)
        assertEquals(14, counter.lastInferenceMs())
        assertTrue(counter.fps() >= 0.0)
    }
}
