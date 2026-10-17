package org.firstinspires.ftc.teamcode.teleop

import org.junit.Assert.*
import org.junit.Test

class TelemetryRefreshTest {
    @Test fun `100 control ticks publish 20 complete telemetry frames without blocking`() {
        var now = 0L
        val refresh = TelemetryRefresh { now }
        var frames = 0
        repeat(100) {
            if (refresh.isDue()) frames++
            now += 10_000_000L
        }
        assertEquals(20, frames)
    }

    @Test fun `slow loops do not queue stale frames and reset publishes immediately`() {
        var now = 0L
        val refresh = TelemetryRefresh { now }
        assertTrue(refresh.isDue())
        now = 49_999_999L
        assertFalse(refresh.isDue())
        now = 50_000_000L
        assertTrue(refresh.isDue())
        now = 1_000_000_000L
        assertTrue(refresh.isDue())
        assertFalse(refresh.isDue())
        refresh.reset()
        assertTrue(refresh.isDue())
    }
}
