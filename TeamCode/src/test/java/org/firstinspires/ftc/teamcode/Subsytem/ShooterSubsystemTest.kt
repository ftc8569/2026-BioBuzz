package org.firstinspires.ftc.teamcode.Subsytem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.teamcode.hardware.BioBuzzShooterMotor
import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem.ControlMode
import org.firstinspires.ftc.teamcode.teleop.FeedControls
import org.junit.After
import org.junit.Before
import org.junit.Assert.*
import org.junit.Test
import org.mockito.ArgumentMatchers.anyDouble
import org.mockito.Mockito.*

class ShooterSubsystemTest {
    @Before fun useTestGains() {
        // Keep regression expectations independent of the operator's tuned defaults.
        ShooterPidTuning.kP = 0.0003
        ShooterPidTuning.kI = 0.0002
        ShooterPidTuning.kD = 0.0
        ShooterPidTuning.kF = 0.0
    }

    @After fun restoreGains() {
        ShooterPidTuning.kP = ShooterPidTuning.DEFAULT_KP
        ShooterPidTuning.kI = ShooterPidTuning.DEFAULT_KI
        ShooterPidTuning.kD = ShooterPidTuning.DEFAULT_KD
        ShooterPidTuning.kF = ShooterPidTuning.DEFAULT_KF
    }

    private class Rig(val cpr1: Double = 28.0, val cpr2: Double = 28.0) {
        val motor1 = mock(DcMotorEx::class.java)
        val motor2 = mock(DcMotorEx::class.java)
        val blocker = mock(Servo::class.java)
        var speed1 = 0.0
        var speed2 = 0.0
        var power1 = 0.0
        var power2 = 0.0
        var now = 0L
        var voltage = 12.0
        var otherVoltage = Double.NaN
        var voltageReads = 0
        val shooter: ShooterSubsystem

        init {
            val map = mock(HardwareMap::class.java)
            val sensor = mock(VoltageSensor::class.java)
            val otherSensor = mock(VoltageSensor::class.java)
            `when`(sensor.voltage).thenAnswer { voltageReads++; voltage }
            `when`(otherSensor.voltage).thenAnswer { voltageReads++; otherVoltage }
            `when`(map.getAll(VoltageSensor::class.java)).thenReturn(listOf(sensor, otherSensor))
            `when`(map.get(DcMotorEx::class.java, "shooter1")).thenReturn(motor1)
            `when`(map.get(DcMotorEx::class.java, "shooter2")).thenReturn(motor2)
            `when`(map.get(Servo::class.java, "blocker")).thenReturn(blocker)
            for ((motor, cpr) in listOf(motor1 to cpr1, motor2 to cpr2)) {
                val profile = mock(MotorConfigurationType::class.java)
                `when`(profile.ticksPerRev).thenReturn(cpr)
                `when`(motor.motorType).thenReturn(profile)
            }
            `when`(motor1.velocity).thenAnswer { speed1 }
            `when`(motor2.velocity).thenAnswer { speed2 }
            doAnswer { power1 = it.getArgument(0); null }.`when`(motor1).setPower(anyDouble())
            doAnswer { power2 = it.getArgument(0); null }.`when`(motor2).setPower(anyDouble())
            shooter = ShooterSubsystem(map) { now }
        }

        fun rpm(first: Double, second: Double = first) {
            speed1 = first * cpr1 / 60.0
            speed2 = -second * cpr2 / 60.0
        }

        // Fixed test target: 0.64 * 4500 = 2880 RPM, independent of the operator preset.
        fun step(ms: Long = 0, power: Double = 0.64, mode: ControlMode = ControlMode.VELOCITY_PID) {
            now += ms * 1_000_000
            // Publish a sample deterministically; production does this on the poller.
            shooter.batteryVoltage.pollOnce()
            shooter.updateShot(power, mode)
        }
    }

    @Test fun `control loops never read voltage sensors`() {
        val rig = Rig()
        repeat(10) {
            rig.shooter.updateShot(0.5, ControlMode.MANUAL_POWER)
            rig.shooter.setFlywheelPower(-0.5)
            rig.shooter.setFlywheelPower(0.0)
        }
        assertEquals(0, rig.voltageReads)
    }

    @Test fun `control uses latest published voltage without polling`() {
        val rig = Rig()
        rig.voltage = 10.0
        rig.shooter.batteryVoltage.pollOnce()
        val reads = rig.voltageReads
        rig.voltage = 14.0
        rig.shooter.updateShot(0.5, ControlMode.MANUAL_POWER)
        assertEquals(0.6, rig.power1, 1e-9)
        assertEquals(reads, rig.voltageReads)
        rig.shooter.batteryVoltage.pollOnce()
        rig.shooter.updateShot(0.5, ControlMode.MANUAL_POWER)
        assertEquals(0.5 * 12.0 / 14.0, rig.power1, 1e-9)
    }

    @Test fun `PID output tracks battery voltage without changing RPM target`() {
        ShooterPidTuning.kP = 0.0001
        ShooterPidTuning.kI = 0.0
        val rig = Rig()
        for (voltage in listOf(12.0, 10.0, 14.0)) {
            rig.voltage = voltage
            rig.step(20)
            assertEquals(0.288 * 12.0 / voltage, rig.power1, 1e-9)
            assertEquals(rig.power1, rig.power2, 0.0)
            assertEquals(2880.0, rig.shooter.targetRpm, 1e-9)
        }
    }

    @Test fun `manual and reverse power are normalized and saturated`() {
        val rig = Rig()
        rig.voltage = 10.0
        rig.step(power = 0.5, mode = ControlMode.MANUAL_POWER)
        assertEquals(0.6, rig.power1, 1e-9)
        assertEquals(0.6, rig.power2, 1e-9)
        rig.shooter.setFlywheelPower(-0.5)
        assertEquals(-0.6, rig.power1, 1e-9)
        assertEquals(-0.6, rig.power2, 1e-9)
        rig.step(power = 1.0, mode = ControlMode.MANUAL_POWER)
        assertEquals(1.0, rig.power1, 0.0)
        rig.shooter.setFlywheelPower(-1.0)
        assertEquals(-1.0, rig.power1, 0.0)
        rig.shooter.setFlywheelPower(0.0)
        assertEquals(0.0, rig.power1, 0.0)
    }

    @Test fun `lowest valid sensor is used and invalid readings fall back to nominal`() {
        val rig = Rig()
        rig.voltage = 14.0
        rig.otherVoltage = 10.0
        rig.step(power = 0.5, mode = ControlMode.MANUAL_POWER)
        assertEquals(0.6, rig.power1, 1e-9)
        for (invalid in listOf(0.0, -1.0, Double.NaN, Double.POSITIVE_INFINITY)) {
            rig.voltage = invalid
            rig.otherVoltage = invalid
            rig.step(power = 0.5, mode = ControlMode.MANUAL_POWER)
            assertEquals(0.5, rig.power1, 0.0)
            assertEquals(0.5, rig.power2, 0.0)
        }
    }

    @Test fun `one to one profile converts both readings to actual RPM`() {
        val profile = BioBuzzShooterMotor::class.java.getAnnotation(MotorType::class.java)!!
        assertEquals(28.0, profile.ticksPerRev, 0.0)
        assertEquals(1.0, profile.gearing, 0.0)
        val rig = Rig(profile.ticksPerRev, profile.ticksPerRev)
        rig.speed1 = 50.0 * 2786.0 / 60.0
        rig.speed2 = -rig.speed1
        rig.step()
        assertEquals(4975.0, rig.shooter.measuredRpm1, 1e-9)
        assertEquals(4975.0, rig.shooter.measuredRpm2, 1e-9)
        assertEquals(2880.0, rig.shooter.targetRpm, 1e-9)
        assertEquals(1344.0, rig.shooter.targetTicksPerSecond1, 1e-9)
    }

    @Test fun `Panels P changes affect both FTCLib controllers on next loop`() {
        ShooterPidTuning.kP = 0.0001
        ShooterPidTuning.kI = 0.0
        val rig = Rig()
        rig.step()
        assertEquals(0.288, rig.power1, 1e-9)
        assertEquals(0.288, rig.power2, 1e-9)
        ShooterPidTuning::class.java.getField("kP").setDouble(null, 0.0002)
        rig.step(20)
        assertEquals(0.576, rig.power1, 1e-9)
        assertEquals(0.576, rig.power2, 1e-9)
        verify(rig.motor1).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }

    @Test fun `FTCLib feedforward is optional and zero gains stop output and transfer`() {
        ShooterPidTuning.kP = 0.0
        ShooterPidTuning.kI = 0.0
        ShooterPidTuning.kF = 0.0001
        val rig = Rig()
        rig.step()
        assertEquals(0.288, rig.power1, 1e-9)
        rig.rpm(2880.0)
        rig.step(20)
        assertTrue(rig.shooter.feedReady)
        ShooterPidTuning.kF = 0.0
        rig.step(20)
        assertEquals(0.0, rig.power1, 0.0)
        assertEquals(0.0, rig.power2, 0.0)
        assertFalse(rig.shooter.feedReady)
        assertFalse(rig.shooter.blockerOpen)
    }

    @Test fun `transfer latches after RPM one reaches one percent and survives speed drops`() {
        val rig = Rig(60.0, 60.0)
        rig.rpm(2735.0, 2880.0)
        rig.step()
        assertFalse(rig.shooter.feedReady)
        rig.rpm(3025.0, 2880.0)
        rig.step(20)
        assertFalse(rig.shooter.feedReady)
        rig.rpm(2852.0, 0.0) // Flywheel 2 is ignored; RPM 1 is just inside -1%.
        rig.step(20)
        assertTrue(rig.shooter.feedReady)
        assertTrue(rig.shooter.blockerOpen)
        rig.rpm(2908.0, 6000.0) // RPM 1 just inside +1%.
        rig.step(20)
        assertTrue(rig.shooter.feedReady)
        rig.rpm(2735.0, 2880.0)
        rig.step(20)
        assertTrue(rig.shooter.feedReady)
        assertTrue(rig.shooter.blockerOpen)
        assertFalse(rig.shooter.atSpeed)
        assertEquals(0.64, FeedControls.intakePower(false, 0.0, 0.64, 0.64,
            ControlMode.VELOCITY_PID, rig.shooter.feedReady, false), 0.0)
        assertEquals(0.0, FeedControls.intakePower(true, 0.0, 0.64, 0.64,
            ControlMode.VELOCITY_PID, rig.shooter.feedReady, false), 0.0)
        rig.rpm(3025.0, 2880.0)
        rig.step(20)
        assertTrue(rig.shooter.feedReady)
        rig.rpm(2880.0, 2880.0)
        rig.step(20)
        assertTrue(rig.shooter.feedReady)
    }

    @Test fun `release reverse and mode changes reset the feed latch`() {
        val rig = Rig()
        rig.rpm(2880.0)
        rig.step()
        assertTrue(rig.shooter.feedReady)
        rig.shooter.setFlywheelPower(0.0)
        assertFalse(rig.shooter.feedReady)
        assertFalse(rig.shooter.blockerOpen)
        rig.rpm(2000.0)
        rig.step(20)
        assertFalse(rig.shooter.feedReady)
        rig.rpm(2880.0)
        rig.step(20)
        assertTrue(rig.shooter.feedReady)
        rig.shooter.setFlywheelPower(-0.8)
        assertFalse(rig.shooter.feedReady)
        rig.rpm(2880.0)
        rig.step(20)
        assertTrue(rig.shooter.feedReady)
        rig.shooter.setControlMode(ControlMode.MANUAL_POWER)
        assertFalse(rig.shooter.feedReady)
        rig.shooter.setControlMode(ControlMode.VELOCITY_PID)
        rig.rpm(2000.0)
        rig.step(20)
        assertFalse(rig.shooter.feedReady)
    }

    @Test fun `each motor uses its own encoder counts and a preset change resets readiness`() {
        val rig = Rig(28.0, 56.0)
        rig.rpm(2880.0)
        rig.step()
        assertEquals(2880.0, rig.shooter.measuredRpm1, 1e-9)
        assertEquals(2880.0, rig.shooter.measuredRpm2, 1e-9)
        assertTrue(rig.shooter.feedReady)
        rig.step(20, power = 0.8)
        assertEquals(3600.0, rig.shooter.targetRpm, 0.0)
        assertFalse(rig.shooter.feedReady)
    }

    @Test fun `zero RPM never triggers a timed motor shutdown`() {
        ShooterPidTuning.kP = 0.0001
        ShooterPidTuning.kI = 0.0
        val rig = Rig()
        rig.step()
        rig.step(10000)
        assertEquals(0.288, rig.power1, 1e-9)
        assertEquals(0.288, rig.power2, 1e-9)
        assertFalse(rig.shooter.feedReady)
        rig.rpm(2880.0, 0.0)
        rig.step(20)
        assertTrue(rig.shooter.feedReady)
        rig.rpm(0.0, 2880.0)
        rig.step(20)
        assertEquals(0.288, rig.power1, 1e-9)
        assertEquals(0.288, rig.power2, 1e-9)
        assertTrue(rig.shooter.feedReady)
    }

    @Test fun `both motors use RPM one even when RPM two disagrees`() {
        ShooterPidTuning.kP = 0.0001
        ShooterPidTuning.kI = 0.0
        val rig = Rig()
        rig.rpm(2000.0, 6000.0)
        rig.step()
        assertEquals(0.088, rig.power1, 1e-9)
        assertEquals(0.088, rig.power2, 1e-9)
        rig.speed2 = Double.NaN
        rig.step(20)
        assertEquals(0.088, rig.power1, 1e-9)
        assertEquals(0.088, rig.power2, 1e-9)
    }

    @Test fun `zero gains keep both motors off while the button remains held`() {
        val rig = Rig()
        rig.step()
        for (name in listOf("kP", "kI", "kD", "kF"))
            ShooterPidTuning::class.java.getField(name).setDouble(null, 0.0)
        rig.step(20)
        rig.step(3000)
        assertEquals(0.0, rig.power1, 0.0)
        assertEquals(0.0, rig.power2, 0.0)
        assertFalse(rig.shooter.feedReady)
    }

    @Test fun `manual mode runs raw power and switching or releasing closes feeding`() {
        val rig = Rig()
        rig.step(mode = ControlMode.MANUAL_POWER)
        assertEquals(0.64, rig.power1, 0.0)
        rig.shooter.setManualBlockerOpen(true)
        assertTrue(rig.shooter.blockerOpen)
        rig.step(20, mode = ControlMode.VELOCITY_PID)
        assertFalse(rig.shooter.blockerOpen)
        rig.shooter.setFlywheelPower(0.0)
        assertEquals(0.0, rig.power1, 0.0)
        assertEquals(0.0, rig.power2, 0.0)
        assertEquals(0.0, rig.shooter.targetRpm, 0.0)
        rig.shooter.setFlywheelPower(-0.8)
        assertEquals(-0.8, rig.power1, 0.0)
        assertFalse(rig.shooter.feedReady)
    }

    @Test fun `invalid numeric input never produces NaN motor power`() {
        ShooterPidTuning.kP = Double.NaN
        val rig = Rig()
        rig.step()
        assertEquals(0.0, rig.power1, 0.0)
        assertFalse(rig.shooter.feedReady)
        restoreGains()
        val badProfile = Rig(0.0, 28.0)
        badProfile.step()
        assertEquals(0.0, badProfile.power1, 0.0)
        assertFalse(badProfile.shooter.feedReady)
    }
}
