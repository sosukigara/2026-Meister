package com.meister.skeleton

import org.junit.Assert.assertEquals
import org.junit.Test

class FakeSkeletonTest {

    

    @Test fun fake_cyclesGooChokiPar() {
        val fake = FakeSkeleton()
        assertEquals(Janken.GOO, fake.detectHands(640, 480, 0.5f).first().janken)
        assertEquals(Janken.CHOKI, fake.detectHands(640, 480, 0.5f).first().janken)
        assertEquals(Janken.PAR, fake.detectHands(640, 480, 0.5f).first().janken)
    }

    @Test fun fake_belowThreshold_returnsEmpty() {
        val fake = FakeSkeleton()
        assertEquals(0, fake.detectHands(640, 480, 0.95f).size)
    }
}
