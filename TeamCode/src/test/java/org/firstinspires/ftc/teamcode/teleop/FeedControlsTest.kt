package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem.ControlMode
import org.junit.Assert.*
import org.junit.Test

class FeedControlsTest {
    private fun feed(ready: Boolean, manual: Double = 0.0, b: Boolean = false,
                     mode: ControlMode = ControlMode.VELOCITY_PID,
                     shooterPower: Double = 0.66, stop: Boolean = false): Double =
        FeedControls.intakePower(stop, manual, shooterPower, 0.66, mode, ready, b)

    @Test fun `PID cannot forward feed early even with B and trigger held`() {
        assertEquals(0.0, feed(false), 0.0)
        assertEquals(0.0, feed(false, manual = 0.9, b = true), 0.0)
        assertEquals(0.66, feed(true), 0.0)
        assertEquals(0.9, feed(true, manual = 0.9, b = true), 0.0)
    }

    @Test fun `automatic intake starts stops and resumes with readiness without trigger presses`() {
        assertEquals(0.0, feed(false), 0.0)
        assertEquals(0.66, feed(true), 0.0)
        assertEquals(0.0, feed(false), 0.0)
        assertEquals(0.66, feed(true), 0.0)
        assertEquals(0.0, feed(true, stop = true), 0.0)
    }

    @Test fun `ordinary collection reverse jam clearing and manual override remain available`() {
        assertEquals(0.7, feed(false, manual = 0.7, shooterPower = 0.0), 0.0)
        assertEquals(-0.8, feed(false, manual = -0.8), 0.0)
        assertEquals(0.7, feed(false, manual = 0.7, b = true, mode = ControlMode.MANUAL_POWER), 0.0)
        assertEquals(0.0, feed(false, manual = 0.7, mode = ControlMode.MANUAL_POWER), 0.0)
        assertEquals(0.66, feed(true, manual = Double.NaN), 0.0)
    }
}
