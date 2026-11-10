package com.meister.vision

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class YoloPostprocessTest {

    private fun blankOutput(cells: Int): Array<FloatArray> =
        Array(YoloPostprocess.CHANNELS) { FloatArray(cells) }

    @Test
    fun letterboxMath_matchesCpp() {
        val lb = YoloPostprocess.letterboxMath(1280, 720, 640)
        assertEquals(0.5, lb.ratio, 1e-9)
        assertEquals(640, lb.newW)
        assertEquals(360, lb.newH)
        assertEquals(0, lb.padW)
        assertEquals(140, lb.padH)
    }

    @Test
    fun singleDetection_coordinatesRestored() {
        val out = blankOutput(2)
        // cell0: 中心 (320,320) 100x100、class0=0.9
        out[0][0] = 320f
        out[1][0] = 320f
        out[2][0] = 100f
        out[3][0] = 100f
        out[4][0] = 0.9f
        val lb = YoloPostprocess.LetterboxMath(1.0, 0, 0, 640, 640)
        val dets = YoloPostprocess.postprocess(out, 2, lb, 640, 640, 0.25f, 0.45f)
        assertEquals(1, dets.size)
        val d = dets[0]
        assertEquals(0, d.classId)
        assertEquals("person", d.className)
        assertEquals(270f, d.x1, 1e-3f)
        assertEquals(270f, d.y1, 1e-3f)
        assertEquals(370f, d.x2, 1e-3f)
        assertEquals(370f, d.y2, 1e-3f)
    }

    @Test
    fun nms_suppressesOverlap() {
        val out = blankOutput(2)
        for (cell in 0..1) {
            out[0][cell] = 320f
            out[1][cell] = 320f
            out[2][cell] = 100f
            out[3][cell] = 100f
            out[4][cell] = if (cell == 0) 0.9f else 0.8f
        }
        val lb = YoloPostprocess.LetterboxMath(1.0, 0, 0, 640, 640)
        val dets = YoloPostprocess.postprocess(out, 2, lb, 640, 640, 0.25f, 0.45f)
        assertEquals(1, dets.size)
        assertEquals(0.9f, dets[0].score, 1e-6f)
    }

    @Test
    fun belowThreshold_dropped() {
        val out = blankOutput(1)
        out[0][0] = 320f
        out[1][0] = 320f
        out[2][0] = 100f
        out[3][0] = 100f
        out[4][0] = 0.1f
        val lb = YoloPostprocess.LetterboxMath(1.0, 0, 0, 640, 640)
        val dets = YoloPostprocess.postprocess(out, 1, lb, 640, 640, 0.25f, 0.45f)
        assertTrue(dets.isEmpty())
    }
}
