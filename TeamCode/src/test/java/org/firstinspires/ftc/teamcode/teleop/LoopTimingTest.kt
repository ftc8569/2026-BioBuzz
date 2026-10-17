package org.firstinspires.ftc.teamcode.teleop

import org.junit.Assert.*
import org.junit.Test

class LoopTimingTest {
    @Test fun `first update establishes baseline and subsequent updates measure full interval`() {
        var now = 8_000_000_000L
        val timing = LoopTiming { now }
        timing.update()
        assertEquals(0.0, timing.intervalMs, 0.0)
        assertEquals(0.0, timing.frequencyHz, 0.0)
        now += 20_000_000L
        timing.update()
        assertEquals(20.0, timing.intervalMs, 1e-9)
        assertEquals(50.0, timing.frequencyHz, 1e-9)
        now += 5_000_000L
        timing.update()
        assertEquals(5.0, timing.intervalMs, 1e-9)
        assertEquals(200.0, timing.frequencyHz, 1e-9)
    }

    @Test fun `reset excludes idle time and zero intervals never produce infinity`() {
        var now = 0L
        val timing = LoopTiming { now }
        timing.update()
        timing.update()
        assertEquals(0.0, timing.frequencyHz, 0.0)
        now = 20_000_000L
        timing.update()
        timing.reset()
        now = 100_000_000_000L
        timing.update()
        assertEquals(0.0, timing.intervalMs, 0.0)
        assertEquals(0.0, timing.frequencyHz, 0.0)
        now += 10_000_000L
        timing.update()
        assertEquals(10.0, timing.intervalMs, 1e-9)
        assertEquals(100.0, timing.frequencyHz, 1e-9)
    }
}
