package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.teamcode.subsystems.BlockerFeedDelay
import org.junit.Assert.assertEquals
import org.junit.Test

class BlockerFeedDelayTest {
    @Test fun `forward feed waits 100 milliseconds after opening without restarting each loop`() {
        var now = 0L
        val delay = BlockerFeedDelay { now }
        assertEquals(0.0, delay.apply(0.8, true), 0.0)
        now = 99_999_999L
        assertEquals(0.0, delay.apply(0.8, true), 0.0)
        now = 100_000_000L
        assertEquals(0.8, delay.apply(0.8, true), 0.0)
        now = 200_000_000L
        assertEquals(0.2, delay.apply(0.2, true), 0.0)
    }

    @Test fun `closing and reopening restarts delay but collection while closed is unchanged`() {
        var now = 0L
        val delay = BlockerFeedDelay { now }
        assertEquals(0.2, delay.apply(0.2, false), 0.0)
        delay.apply(0.8, true)
        now = 100_000_000L
        assertEquals(0.8, delay.apply(0.8, true), 0.0)
        assertEquals(0.2, delay.apply(0.2, false), 0.0)
        assertEquals(0.0, delay.apply(0.8, true), 0.0)
        now = 200_000_000L
        assertEquals(0.8, delay.apply(0.8, true), 0.0)
    }

    @Test fun `stop and reverse remain immediate and opening is tracked while stopped`() {
        var now = 0L
        val delay = BlockerFeedDelay { now }
        assertEquals(-0.6, delay.apply(-0.6, true), 0.0)
        now = 50_000_000L
        assertEquals(0.0, delay.apply(0.0, true), 0.0)
        assertEquals(0.0, delay.apply(0.8, true), 0.0)
        now = 100_000_000L
        assertEquals(0.8, delay.apply(0.8, true), 0.0)
    }
}
