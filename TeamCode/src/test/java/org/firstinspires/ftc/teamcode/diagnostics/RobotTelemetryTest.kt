package org.firstinspires.ftc.teamcode.diagnostics

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.junit.Assert.*
import org.junit.Test
import org.mockito.Mockito.*

class RobotTelemetryTest {
    @Test fun `real JoinedTelemetry formats a row once and shares the text with both outputs`() {
        val ds = mock(Telemetry::class.java)
        val panels = mock(Telemetry::class.java)
        val report = RobotTelemetry(ds, panels, LoopProfiler { 0L }) { 0L }
        var formatted = 0
        val value = object { override fun toString(): String { formatted++; return "cached 100% RPM" } }
        report.publish {
            addData("Shooter", "%s", value)
            addData("Ready", "100% ready")
        }
        assertEquals(1, formatted)
        for (output in listOf(ds, panels)) {
            verify(output).msTransmissionInterval = 50
            verify(output).addData("Shooter", "cached 100% RPM" as Any)
            verify(output).addData("Ready", "100% ready" as Any)
            verify(output).update()
            verifyNoMoreInteractions(output)
        }
    }

    @Test fun `build and both telemetry updates share the same fifty millisecond throttle`() {
        val ds = mock(Telemetry::class.java)
        val panels = mock(Telemetry::class.java)
        var now = 0L
        var builds = 0
        val report = RobotTelemetry(ds, panels, LoopProfiler { now }) { now }
        report.publish { builds++ }
        now = 49_999_999
        repeat(10) { report.publish { builds++ } }
        assertEquals(1, builds)
        now = 50_000_000
        report.publish { builds++ }
        assertEquals(2, builds)
        verify(ds, times(2)).update()
        verify(panels, times(2)).update()
        report.clear()
        report.publish { builds++ }
        assertEquals(3, builds)
        verify(ds).clearAll()
        verify(panels).clearAll()
    }

    @Test fun `each destination update keeps its own timing measurement`() {
        var now = 0L
        val profiler = LoopProfiler { now }
        val ds = mock(Telemetry::class.java)
        val panels = mock(Telemetry::class.java)
        doAnswer { now += 2_000_000; true }.`when`(ds).update()
        doAnswer { now += 3_000_000; true }.`when`(panels).update()
        val report = RobotTelemetry(ds, panels, profiler) { now }
        report.publish { }
        now += 1_000_000_000
        val samples = profiler.snapshotIfDue()!!.associateBy { it.stage }
        assertEquals(2.0, samples.getValue(LoopStage.DS_TELEMETRY_UPDATE).averageMs, 0.0)
        assertEquals(3.0, samples.getValue(LoopStage.PANELS_TELEMETRY_UPDATE).averageMs, 0.0)
        assertEquals(5.0, samples.getValue(LoopStage.TELEMETRY_UPDATE).averageMs, 0.0)
    }
}
