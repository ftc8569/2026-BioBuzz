package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.teamcode.control.ShooterController.ControlMode
import org.firstinspires.ftc.teamcode.control.ShooterControls
import org.junit.Assert.*
import org.junit.Test

class ShooterControlsTest {
    @Test fun `LQR display target remains selected while idle and tracks preset changes`() {
        val controls = ShooterControls()
        assertEquals(0.0, controls.requestedPower(false, false), 0.0)
        assertEquals(3240.0, controls.velocityTargetRpm!!, 1e-9)
        controls.update(false, true, false)
        assertEquals(3262.5, controls.velocityTargetRpm!!, 1e-9)
        assertEquals(0.725, controls.requestedPower(false, true), 1e-9)
        assertEquals(3262.5, controls.velocityTargetRpm!!, 1e-9)
        controls.update(true, false, false)
        assertNull(controls.velocityTargetRpm)
        controls.update(false, false, false)
        assertEquals(3262.5, controls.velocityTargetRpm!!, 1e-9)
    }

    @Test fun `starts in LQR and manual shooter runs only while Y is held`() {
        val controls = ShooterControls()
        assertEquals(ControlMode.VELOCITY_LQR, controls.mode)
        assertEquals(0.72, controls.preset, 1e-9)
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
        assertEquals(ControlMode.VELOCITY_LQR, controls.mode)
        assertEquals(0.0, controls.requestedPower(false, false), 1e-9)
        assertEquals(controls.preset, controls.requestedPower(false, true), 1e-9)
        controls.update(true, false, false)
        assertEquals(ControlMode.MANUAL_POWER, controls.mode)
        assertEquals(controls.preset, controls.requestedPower(false, false), 1e-9)
        assertEquals(ControlMode.VELOCITY_LQR, ShooterControls().mode)
    }

    @Test fun `preset changes only on edges and stays within motor power range`() {
        val controls = ShooterControls()
        repeat(20) { controls.update(false, true, false) }
        assertEquals(0.725, controls.preset, 1e-9)
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
