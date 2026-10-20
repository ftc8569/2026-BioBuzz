package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.diagnostics.LoopStage
import org.junit.Assert.*
import org.junit.Test
import org.mockito.Mockito.*

class TelemetryToggleTest {
    private fun render(r: RobotTestRig) {
        r.displayNow += 50_000_000
        r.driver.captureControls(r.g1, r.g2)
        r.driver.updateTelemetry()
    }

    @Test fun `right stick click hides details on both outputs and holding does not retoggle`() {
        val r = RobotTestRig()
        r.g1.right_stick_button = true
        repeat(3) { render(r) }
        r.g1.right_stick_button = false
        render(r)
        for (output in listOf(r.ds, r.panels)) {
            verify(output).clearAll()
            verify(output, times(4)).addData("Loop interval (ms)", "0.00")
            verify(output, times(4)).update()
            verifyNoMoreInteractions(output)
        }
        r.verifyNoHardware()
    }

    @Test fun `a second click restores details and starts encoder deltas from fresh baseline`() {
        val r = RobotTestRig()
        r.motors.take(4).forEach { `when`(it.currentPosition).thenReturn(100) }
        r.driver.update(r.g1, r.g2)
        r.g1.right_stick_button = true
        r.driver.update(r.g1, r.g2)
        r.g1.right_stick_button = false
        r.driver.update(r.g1, r.g2)
        r.motors.take(4).forEach { `when`(it.currentPosition).thenReturn(500) }
        clearInvocations(r.ds, r.panels)
        r.g1.right_stick_button = true
        r.driver.update(r.g1, r.g2)
        for (output in listOf(r.ds, r.panels)) {
            verify(output).clearAll()
            verify(output).addData("Detailed logging (G1 right-stick click)", "On")
            verify(output).addData("Drive encoder delta (ticks)", "FL 0, FR 0, BL 0, BR 0")
            verify(output).addData("Shooter flywheel 1 RPM", "0")
        }
    }

    @Test fun `quiet mode keeps intake and localization active without drive encoder reads`() {
        val r = RobotTestRig()
        r.g1.right_stick_button = true
        r.g1.right_trigger = 1f
        r.driver.update(r.g1, r.g2)
        r.motors.take(4).forEach { verify(it, never()).currentPosition }
        verify(r.physical).update()
        verify(r.motors[6]).power = 1.0
        verify(r.motors[7]).power = 1.0
    }

    @Test fun `timing reports continue with details hidden and toggling preserves the window`() {
        val r = RobotTestRig()
        r.profiler.record(LoopStage.PINPOINT, 9_000_000)
        r.g1.right_stick_button = true
        render(r)
        r.profiler.record(LoopStage.PINPOINT, 3_000_000)
        r.now = 1_000_000_000
        mockStatic(RobotLog::class.java).use { logs ->
            render(r)
            for (output in listOf(r.ds, r.panels))
                verify(output).addData("Timing: Pinpoint location read", "avg=6.000 ms max=9.000 ms n=2")
            logs.verify { RobotLog.ii(eq("BioBuzzLoop"), contains("Pinpoint location read: avg=6.000 ms")) }
        }
    }
}
