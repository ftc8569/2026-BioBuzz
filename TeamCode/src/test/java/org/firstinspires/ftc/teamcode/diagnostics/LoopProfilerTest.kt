package org.firstinspires.ftc.teamcode.diagnostics

import org.junit.Assert.*
import org.junit.Test

class LoopProfilerTest {
    @Test fun `reports milliseconds with average maximum and call counts once per second`() {
        var now = 0L
        val profiler = LoopProfiler { now }
        profiler.measure(LoopStage.PINPOINT) { now += 2_000_000 }
        profiler.measure(LoopStage.PINPOINT) { now += 6_000_000 }
        assertNull(profiler.snapshotIfDue())
        now = 1_000_000_000
        val sample = profiler.snapshotIfDue()!!.single()
        assertEquals(LoopStage.PINPOINT, sample.stage)
        assertEquals(2L, sample.calls)
        assertEquals(4.0, sample.averageMs, 0.0)
        assertEquals(6.0, sample.maxMs, 0.0)
        now += 1_000_000_000
        assertTrue(profiler.snapshotIfDue()!!.isEmpty())
    }

    @Test fun `exceptions preserve return behavior and still record elapsed time`() {
        var now = 0L
        val profiler = LoopProfiler { now }
        val error = IllegalStateException("hardware read")
        try {
            profiler.measure(LoopStage.PINPOINT) { now += 3_000_000; throw error }
            fail("Expected the original exception")
        } catch (actual: IllegalStateException) {
            assertSame(error, actual)
        }
        assertEquals(7, profiler.measure(LoopStage.DRIVE_OUTPUT) { 7 })
        now = 1_000_000_000
        assertEquals(3.0, profiler.snapshotIfDue()!!.first { it.stage == LoopStage.PINPOINT }.maxMs, 0.0)
    }

    @Test fun `loop interval includes between-loop delays while work measures only execution`() {
        var now = 0L
        val profiler = LoopProfiler { now }
        profiler.beginLoop()
        now += 5_000_000
        profiler.endLoop()
        now += 15_000_000
        profiler.beginLoop()
        now += 7_000_000
        profiler.endLoop()
        now = 1_000_000_000
        val samples = profiler.snapshotIfDue()!!.associateBy { it.stage }
        assertEquals(20.0, samples.getValue(LoopStage.LOOP_INTERVAL).averageMs, 0.0)
        assertEquals(6.0, samples.getValue(LoopStage.LOOP_WORK).averageMs, 0.0)
        profiler.reset()
        assertNull(profiler.snapshotIfDue())
    }

    @Test fun `background and main measurements share a consistent snapshot`() {
        var now = 0L
        val profiler = LoopProfiler { now }
        val worker = Thread { repeat(1000) { profiler.record(LoopStage.VOLTAGE_POLL, 2_000_000) } }
        worker.start()
        repeat(1000) { profiler.record(LoopStage.DRIVE_OUTPUT, 1_000_000) }
        worker.join(2000)
        assertFalse(worker.isAlive)
        now = 1_000_000_000
        val samples = profiler.snapshotIfDue()!!.associateBy { it.stage }
        assertEquals(1000L, samples.getValue(LoopStage.VOLTAGE_POLL).calls)
        assertEquals(2.0, samples.getValue(LoopStage.VOLTAGE_POLL).averageMs, 0.0)
        assertEquals(1000L, samples.getValue(LoopStage.DRIVE_OUTPUT).calls)
    }
}
