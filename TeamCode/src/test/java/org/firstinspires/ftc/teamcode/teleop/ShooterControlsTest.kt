package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem.ControlMode
import org.junit.Assert.*
import org.junit.Test

class ShooterControlsTest {
    @Test fun `starts in PID and Y toggles exactly once per press`() {
        val controls = ShooterControls()
        assertEquals(ControlMode.VELOCITY_PID, controls.mode)
        assertEquals(0.66, controls.preset, 1e-9)
        controls.update(true, false, false)
        repeat(100) { controls.update(true, false, false) }
        assertEquals(ControlMode.MANUAL_POWER, controls.mode)
        controls.update(false, false, false)
        controls.update(true, false, false)
        assertEquals(ControlMode.VELOCITY_PID, controls.mode)
        assertEquals(ControlMode.VELOCITY_PID, ShooterControls().mode)
    }

    @Test fun `preset changes only on edges and stays within motor power range`() {
        val controls = ShooterControls()
        repeat(20) { controls.update(false, true, false) }
        assertEquals(0.68, controls.preset, 1e-9)
        repeat(100) {
            controls.update(false, false, false)
            controls.update(false, true, false)
        }
        assertEquals(1.0, controls.preset, 1e-9)
        repeat(100) {
            controls.update(false, false, false)
            controls.update(false, false, true)
        }
        assertEquals(0.0, controls.preset, 1e-9)
        controls.update(false, false, false)
        controls.update(false, true, true)
        assertEquals(0.0, controls.preset, 1e-9)
    }
}
