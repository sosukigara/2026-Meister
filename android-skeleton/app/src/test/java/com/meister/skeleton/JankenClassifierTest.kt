package com.meister.skeleton

import org.junit.Assert.assertEquals
import org.junit.Test

// Spec §2 の境界を跨ぐ合成21点で4値を検証する。
class JankenClassifierTest {

    // 手首(0.5,0.9)。指は上向き(y減)。開き=tipが遠い、折り=tipが第二関節より近い。
    private fun hand(
        index: Boolean,
        middle: Boolean,
        ring: Boolean,
        pinky: Boolean,
        thumb: Boolean,
    ): List<HandPoint> {
        val p = Array(21) { HandPoint(0.5f, 0.9f) }
        p[0] = HandPoint(0.5f, 0.9f)
        // 親指
        p[1] = HandPoint(0.45f, 0.85f)
        p[2] = HandPoint(0.40f, 0.80f)
        p[3] = HandPoint(0.35f, 0.75f)
        p[4] = if (thumb) HandPoint(0.15f, 0.60f) else HandPoint(0.42f, 0.82f)
        // 4指: mcp/pip/dip/tip
        val bases = mapOf(5 to 0.42f, 9 to 0.50f, 13 to 0.58f, 17 to 0.66f)
        val opens = mapOf(5 to index, 9 to middle, 13 to ring, 17 to pinky)
        for ((mcp, x) in bases) {
            val open = opens[mcp] == true
            p[mcp] = HandPoint(x, 0.70f)
            p[mcp + 1] = HandPoint(x, 0.60f)
            p[mcp + 2] = HandPoint(x, 0.55f)
            p[mcp + 3] = if (open) HandPoint(x, 0.40f) else HandPoint(x, 0.62f)
        }
        return p.toList()
    }

    @Test fun goo_allFolded_returnsGoo() {
        assertEquals(
            Janken.GOO,
            JankenClassifier.classify(hand(false, false, false, false, false)),
        )
    }

    @Test fun goo_thumbOut_returnsGoo() {
        assertEquals(
            Janken.GOO,
            JankenClassifier.classify(hand(false, false, false, false, true)),
        )
    }

    @Test fun choki_indexMiddle_returnsChoki() {
        assertEquals(
            Janken.CHOKI,
            JankenClassifier.classify(hand(true, true, false, false, false)),
        )
    }

    @Test fun par_allOpen_returnsPar() {
        assertEquals(
            Janken.PAR,
            JankenClassifier.classify(hand(true, true, true, true, true)),
        )
    }

    @Test fun par_thumbFolded_returnsPar() {
        assertEquals(
            Janken.PAR,
            JankenClassifier.classify(hand(true, true, true, true, false)),
        )
    }

    @Test fun unknown_singleFinger_returnsUnknown() {
        assertEquals(
            Janken.UNKNOWN,
            JankenClassifier.classify(hand(true, false, false, false, false)),
        )
    }

    @Test fun unknown_threeFingers_returnsUnknown() {
        assertEquals(
            Janken.UNKNOWN,
            JankenClassifier.classify(hand(true, true, true, false, false)),
        )
    }

    @Test fun unknown_tooFewPoints_returnsUnknown() {
        assertEquals(Janken.UNKNOWN, JankenClassifier.classify(emptyList()))
    }
}
