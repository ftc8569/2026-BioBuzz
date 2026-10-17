package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem.ControlMode
import org.junit.Assert.*
import org.junit.Test

class ShooterControlsTest {
    @Test fun `PID display target remains selected while idle and tracks preset changes`() {
        val controls = ShooterControls()
        assertEquals(0.0, controls.requestedPower(false, false), 0.0)
        assertEquals(3127.5, controls.pidTargetRpm!!, 1e-9)
        controls.update(false, true, false)
        assertEquals(3150.0, controls.pidTargetRpm!!, 1e-9)
        assertEquals(0.70, controls.requestedPower(false, true), 1e-9)
        assertEquals(3150.0, controls.pidTargetRpm!!, 1e-9)
        controls.update(true, false, false)
        assertNull(controls.pidTargetRpm)
        controls.update(false, false, false)
        assertEquals(3150.0, controls.pidTargetRpm!!, 1e-9)
    }

    @Test fun `starts in PID and manual shooter runs only while Y is held`() {
        val controls = ShooterControls()
        assertEquals(ControlMode.VELOCITY_PID, controls.mode)
        assertEquals(0.695, controls.preset, 1e-9)
        assertEquals(0.0, controls.requestedPower(false, false), 1e-9)
        assertEquals(controls.preset, controls.requestedPower(false, true), 1e-9)
        controls.update(true, false, false)
        repeat(200) {
            controls.update(true, false, false)
            assertEquals(controls.preset, controls.requestedPower(false, false), 1e-9)
        }
        assertEquals(ControlMode.MANUAL_POWER, controls.mode)
        assertEquals(-0.8, controls.requestedPower(true, false), 1e-9)
        controls.update(false, false, false)
        assertEquals(ControlMode.VELOCITY_PID, controls.mode)
        assertEquals(0.0, controls.requestedPower(false, false), 1e-9)
        assertEquals(controls.preset, controls.requestedPower(false, true), 1e-9)
        controls.update(true, false, false)
        assertEquals(ControlMode.MANUAL_POWER, controls.mode)
        assertEquals(controls.preset, controls.requestedPower(false, false), 1e-9)
        assertEquals(ControlMode.VELOCITY_PID, ShooterControls().mode)
    }

    @Test fun `preset changes only on edges and stays within motor power range`() {
        val controls = ShooterControls()
        repeat(20) { controls.update(false, true, false) }
        assertEquals(0.70, controls.preset, 1e-9)
        repeat(200) {
            controls.update(false, false, false)
            controls.update(false, true, false)
        }
        assertEquals(1.0, controls.preset, 1e-9)
        repeat(200) {
            controls.update(false, false, false)
            controls.update(false, false, true)
        }
        assertEquals(0.0, controls.preset, 1e-9)
        controls.update(false, false, false)
        controls.update(false, true, true)
        assertEquals(0.0, controls.preset, 1e-9)
    }
}
