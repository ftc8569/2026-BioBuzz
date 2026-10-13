package org.firstinspires.ftc.teamcode.Subsytem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem.ControlMode
import org.junit.Assert.*
import org.junit.Test
import org.mockito.ArgumentMatchers.anyDouble
import org.mockito.Mockito.*

class ShooterSubsystemTest {
    private class Rig(cpr1: Double = 28.0, cpr2: Double = 28.0) {
        val motor1 = mock(DcMotorEx::class.java)
        val motor2 = mock(DcMotorEx::class.java)
        val servo = mock(Servo::class.java)
        private val map = mock(HardwareMap::class.java)
        var now = 0L
        var speed1 = 0.0
        var speed2 = 0.0
        var power1 = 0.0
        var power2 = 0.0
        var servoPosition = 0.0
        val shooter: ShooterSubsystem

        init {
            `when`(map.get(DcMotorEx::class.java, "shooter1")).thenReturn(motor1)
            `when`(map.get(DcMotorEx::class.java, "shooter2")).thenReturn(motor2)
            `when`(map.get(Servo::class.java, "blocker")).thenReturn(servo)
            val profile1 = mock(MotorConfigurationType::class.java)
            val profile2 = mock(MotorConfigurationType::class.java)
            `when`(profile1.ticksPerRev).thenReturn(cpr1)
            `when`(profile2.ticksPerRev).thenReturn(cpr2)
            `when`(motor1.motorType).thenReturn(profile1)
            `when`(motor2.motorType).thenReturn(profile2)
            `when`(motor1.velocity).thenAnswer { speed1 }
            `when`(motor2.velocity).thenAnswer { speed2 }
            doAnswer { power1 = it.getArgument(0); null }.`when`(motor1).setPower(anyDouble())
            doAnswer { power2 = it.getArgument(0); null }.`when`(motor2).setPower(anyDouble())
            doAnswer { servoPosition = it.getArgument(0); null }.`when`(servo).setPosition(anyDouble())
            shooter = ShooterSubsystem(map) { now }
        }

        fun step(ms: Long, power: Double = 0.66, mode: ControlMode = ControlMode.VELOCITY_PID) {
            now += ms * 1_000_000L
            shooter.updateShot(power, mode)
        }

        fun ready() {
            speed1 = 2970.0 * shooter.configuredTicksPerRev1 / 60.0
            speed2 = -2970.0 * shooter.configuredTicksPerRev2 / 60.0
            step(0)
            step(150)
            assertTrue(shooter.feedReady)
        }
    }

    @Test fun `PID starts enabled and 66 percent sets 2970 RPM immediately`() {
        val rig = Rig()
        verify(rig.motor1).setDirection(DcMotorSimple.Direction.REVERSE)
        verify(rig.motor2).setDirection(DcMotorSimple.Direction.FORWARD)
        assertEquals(ControlMode.VELOCITY_PID, rig.shooter.controlMode)
        verify(rig.motor1).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        verify(rig.motor2).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        rig.step(0)
        assertEquals(4500.0, ShooterSubsystem.MAX_MOTOR_RPM, 0.0)
        assertEquals(2970.0, rig.shooter.targetRpm, 1e-9)
        assertEquals(1386.0, rig.shooter.targetTicksPerSecond1, 1e-9)
        assertTrue(rig.power1 > 0.66)
        assertFalse(rig.shooter.feedReady)
        assertEquals(0.24, rig.servoPosition, 1e-12)
    }

    @Test fun `each encoder converts to the same motor RPM using its own counts per rev`() {
        val rig = Rig(28.0, 56.0)
        rig.ready()
        assertEquals(2970.0, rig.shooter.measuredRpm1, 1e-9)
        assertEquals(2970.0, rig.shooter.measuredRpm2, 1e-9)
        assertEquals(1386.0, rig.shooter.targetTicksPerSecond1, 1e-9)
        assertEquals(2772.0, rig.shooter.targetTicksPerSecond2, 1e-9)
        assertEquals(0.66, rig.power1, 1e-12)
        assertEquals(0.66, rig.power2, 1e-12)
    }

    @Test fun `both wheels need dwell and shot speed loss closes blocker until recovery`() {
        val rig = Rig()
        rig.speed1 = 1386.0
        rig.speed2 = -1100.0
        rig.step(0)
        rig.step(1000)
        assertFalse(rig.shooter.feedReady)
        assertTrue(rig.power2 > rig.power1)
        rig.speed2 = -1386.0
        rig.step(20)
        rig.step(149)
        assertFalse(rig.shooter.feedReady)
        rig.step(1)
        assertTrue(rig.shooter.blockerOpen)
        rig.speed1 *= 0.90
        rig.step(20)
        assertFalse(rig.shooter.feedReady)
        assertEquals(0.24, rig.servoPosition, 1e-12)
        rig.speed1 = 1386.0
        rig.step(20)
        rig.step(150)
        assertTrue(rig.shooter.feedReady)
    }

    @Test fun `release battery sag and preset change never learn a lower RPM`() {
        val rig = Rig()
        rig.ready()
        rig.shooter.setFlywheelPower(0.0)
        assertFalse(rig.shooter.feedReady)
        rig.speed1 = 1100.0
        rig.speed2 = -1100.0
        rig.step(20)
        rig.step(2000)
        assertEquals(2970.0, rig.shooter.targetRpm, 1e-9)
        assertFalse(rig.shooter.feedReady)
        rig.step(20, power = 0.68)
        assertEquals(3060.0, rig.shooter.targetRpm, 1e-9)
        assertFalse(rig.shooter.feedReady)
        rig.step(20, power = 1.0)
        assertEquals(4500.0, rig.shooter.targetRpm, 1e-9)
    }

    @Test fun `missing encoder blocks feed and recovers without bumper release`() {
        val rig = Rig()
        rig.speed1 = 1386.0
        rig.step(0)
        rig.step(3000)
        assertTrue(rig.shooter.encoderFault)
        assertFalse(rig.shooter.feedReady)
        rig.shooter.setManualBlockerOpen(true)
        assertFalse(rig.shooter.blockerOpen)
        assertEquals(0.66, rig.power1, 1e-12)
        rig.speed2 = -1386.0
        rig.step(20)
        assertFalse(rig.shooter.encoderFault)
        rig.step(150)
        assertTrue(rig.shooter.feedReady)
        assertEquals(2970.0, rig.shooter.targetRpm, 1e-9)
    }

    @Test fun `B cannot bypass PID speed gate on startup during shots or when idle`() {
        val rig = Rig()
        rig.shooter.setManualBlockerOpen(true)
        assertFalse(rig.shooter.blockerOpen)
        rig.step(0)
        assertFalse(rig.shooter.blockerOpen)
        rig.ready()
        rig.speed1 = 0.0
        rig.step(20)
        assertTrue(rig.shooter.encoderFault)
        assertFalse(rig.shooter.blockerOpen)
        rig.shooter.setFlywheelPower(0.0)
        assertFalse(rig.shooter.blockerOpen)
    }

    @Test fun `manual mode retains timed feed and B override but switching to PID closes it`() {
        val rig = Rig()
        rig.step(0, mode = ControlMode.MANUAL_POWER)
        rig.step(2999, mode = ControlMode.MANUAL_POWER)
        assertFalse(rig.shooter.feedReady)
        assertEquals(0.66, rig.power1, 1e-12)
        rig.step(1, mode = ControlMode.MANUAL_POWER)
        assertTrue(rig.shooter.feedReady)
        assertFalse(rig.shooter.atSpeed)
        rig.shooter.setFlywheelPower(0.0)
        rig.shooter.setManualBlockerOpen(true)
        assertTrue(rig.shooter.blockerOpen)
        rig.shooter.setControlMode(ControlMode.VELOCITY_PID)
        assertFalse(rig.shooter.blockerOpen)
        rig.step(0)
        assertFalse(rig.shooter.feedReady)
    }

    @Test fun `preset changes reverse stop and invalid power clear readiness`() {
        val rig = Rig()
        rig.ready()
        rig.step(20, power = 0.68)
        assertFalse(rig.shooter.feedReady)
        rig.shooter.setFlywheelPower(-0.8)
        assertEquals(-0.8, rig.power1, 1e-12)
        assertFalse(rig.shooter.feedReady)
        assertEquals(0.0, rig.shooter.targetRpm, 0.0)
        assertFalse(rig.shooter.blockerOpen)
        rig.step(20, power = Double.NaN)
        assertEquals(0.0, rig.power1, 0.0)
        rig.shooter.setFlywheelPower(Double.NaN)
        assertEquals(0.0, rig.power2, 0.0)
    }

    @Test fun `nonfinite feedback blocks feed and recovers without propagating NaN power`() {
        val rig = Rig()
        rig.speed1 = Double.NaN
        rig.step(0)
        assertTrue(rig.shooter.encoderFault)
        assertEquals(0.66, rig.power1, 1e-12)
        assertFalse(rig.shooter.feedReady)
        rig.ready()
        assertFalse(rig.shooter.encoderFault)
    }

    @Test fun `invalid counts per rev stop PID motors and cannot open blocker`() {
        for (cpr in listOf(0.0, -1.0, Double.NaN, Double.POSITIVE_INFINITY)) {
            val rig = Rig(cpr)
            rig.step(0)
            rig.shooter.setManualBlockerOpen(true)
            assertFalse(rig.shooter.rpmConfigurationValid)
            assertTrue(rig.shooter.encoderFault)
            assertFalse(rig.shooter.feedReady)
            assertFalse(rig.shooter.blockerOpen)
            assertEquals(0.0, rig.power1, 0.0)
            assertEquals(0.0, rig.power2, 0.0)
        }
    }
}
