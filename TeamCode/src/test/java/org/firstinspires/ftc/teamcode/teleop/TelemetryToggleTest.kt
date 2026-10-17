package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.util.RobotLog
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.Subsytem.DriveSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.RobotLocationSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.FieldLocation
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem
import org.firstinspires.ftc.teamcode.diagnostics.LoopStage
import org.firstinspires.ftc.teamcode.diagnostics.LoopProfiler
import org.junit.Assert.*
import org.junit.Test
import org.mockito.ArgumentMatchers.*
import org.mockito.Mockito.*

class TelemetryToggleTest {
    private fun setField(opMode: BioBuzzTeleOp, name: String, value: Any?) {
        BioBuzzTeleOp::class.java.getDeclaredField(name).apply { isAccessible = true }.set(opMode, value)
    }

    private fun render(opMode: BioBuzzTeleOp) { opMode.captureControls(); opMode.onUpdate() }

    private fun opMode(nanoTime: () -> Long = { 0L }): BioBuzzTeleOp = BioBuzzTeleOp().apply {
        telemetry = mock(Telemetry::class.java)
        gamepad1 = Gamepad()
        gamepad2 = Gamepad()
        // Keep report logging off the Android runtime in local JVM tests.
        setField(this, "profiler", LoopProfiler(nanoTime))
        var displayClock = 0L
        setField(this, "telemetryRefresh", TelemetryRefresh { displayClock += 50_000_000; displayClock })
    }

    @Test fun `right stick click hides non-timing telemetry and holding does not toggle repeatedly`() {
        val opMode = opMode()
        opMode.gamepad1.right_stick_button = true
        repeat(3) { render(opMode) }
        opMode.gamepad1.right_stick_button = false
        render(opMode)
        // Uninitialized shooter/hold fields must never be evaluated in quiet mode.
        verify(opMode.telemetry).clearAll()
        verify(opMode.telemetry, times(4)).addData(eq("Loop interval (ms)"), eq("%.2f"), anyDouble())
        verify(opMode.telemetry, times(4)).update()
        verifyNoMoreInteractions(opMode.telemetry)
    }

    @Test fun `a second click restores details and clears stale encoder deltas`() {
        val opMode = opMode()
        setField(opMode, "shooter", mock(ShooterSubsystem::class.java))
        setField(opMode, "shootingDriveHold", mock(ShootingDriveHold::class.java))
        render(opMode)
        verify(opMode.telemetry).addData("Detailed logging (G1 right-stick click)", "On")
        opMode.gamepad1.right_stick_button = true
        render(opMode)
        opMode.gamepad1.right_stick_button = false
        render(opMode)
        setField(opMode, "previousDriveEncoderPositions", intArrayOf(1, 2, 3, 4))
        clearInvocations(opMode.telemetry)
        opMode.gamepad1.right_stick_button = true
        render(opMode)
        verify(opMode.telemetry).clearAll()
        verify(opMode.telemetry).addData("Detailed logging (G1 right-stick click)", "On")
        verify(opMode.telemetry).addData(eq("Shooter flywheel 1 RPM"), eq("%.0f"), anyDouble())
        val positions = BioBuzzTeleOp::class.java.getDeclaredField("previousDriveEncoderPositions")
            .apply { isAccessible = true }.get(opMode)
        assertNull(positions)
    }
    @Test fun `quiet driver and mechanism callbacks still control hardware without diagnostic reads`() {
        val opMode = opMode()
        opMode.gamepad1.right_stick_button = true
        render(opMode)
        val motors = List(4) { mock(DcMotor::class.java) }
        val drive = mock(DriveSubsystem::class.java)
        val intake = mock(IntakeSubsystem::class.java)
        val shooter = mock(ShooterSubsystem::class.java)
        val location = mock(RobotLocationSubsystem::class.java)
        `when`(location.getLocation()).thenReturn(FieldLocation(0.0, 0.0, 0.0))
        setField(opMode, "driveMotors", motors)
        setField(opMode, "drive", drive)
        setField(opMode, "intake", intake)
        setField(opMode, "shooter", shooter)
        setField(opMode, "shootingDriveHold", mock(ShootingDriveHold::class.java))
        clearInvocations(opMode.telemetry)
        for (callback in listOf("updateDrive", "updateMechanisms")) {
            BioBuzzTeleOp::class.java.getDeclaredMethod(callback).apply { isAccessible = true }.invoke(opMode)
        }
        verify(drive).drive(anyDouble(), anyDouble(), anyDouble())
        verify(shooter).setFlywheelPower(0.0)
        verify(intake).setPower(0.0)
        verifyNoInteractions(*motors.toTypedArray(), opMode.telemetry)
    }

    @Test fun `timing reports continue with details hidden and toggling preserves the window`() {
        var now = 0L
        val opMode = opMode { now }
        val profiler = BioBuzzTeleOp::class.java.getDeclaredField("profiler")
            .apply { isAccessible = true }.get(opMode) as LoopProfiler
        profiler.record(LoopStage.PINPOINT, 9_000_000)
        opMode.gamepad1.right_stick_button = true
        render(opMode)
        profiler.record(LoopStage.PINPOINT, 3_000_000)
        now = 1_000_000_000
        mockStatic(RobotLog::class.java).use { logs ->
            render(opMode)
            verify(opMode.telemetry).addData("Timing: Pinpoint location read", "avg=6.000 ms max=9.000 ms n=2")
            logs.verify { RobotLog.ii(eq("BioBuzzLoop"), contains("Pinpoint location read: avg=6.000 ms")) }
        }
    }

}
