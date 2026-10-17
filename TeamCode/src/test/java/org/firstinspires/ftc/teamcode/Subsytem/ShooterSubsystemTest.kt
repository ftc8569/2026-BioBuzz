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
import org.firstinspires.ftc.teamcode.diagnostics.LoopProfiler
import org.firstinspires.ftc.teamcode.diagnostics.LoopStage
import org.junit.After
import org.junit.Before
import org.junit.Assert.*
import org.junit.Test
import org.mockito.ArgumentMatchers.anyDouble
import org.mockito.Mockito.*

class ShooterSubsystemTest {
    @Before fun useTestModel() = restoreModel()

    @After fun restoreModel() {
        ShooterLqrTuning.kV = ShooterLqrTuning.DEFAULT_KV
        ShooterLqrTuning.kA = ShooterLqrTuning.DEFAULT_KA
        ShooterLqrTuning.rpmTolerance = ShooterLqrTuning.DEFAULT_RPM_TOLERANCE
        ShooterLqrTuning.voltageTolerance = ShooterLqrTuning.DEFAULT_VOLTAGE_TOLERANCE
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
        fun step(ms: Long = 0, power: Double = 0.64, mode: ControlMode = ControlMode.VELOCITY_LQR) {
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

    @Test fun `profiling captures shooter hardware reads without adding voltage reads`() {
        val rig = Rig()
        var clock = 0L
        val profiler = LoopProfiler { clock }
        rig.shooter.loopProfiler = profiler
        `when`(rig.motor1.currentPosition).thenAnswer { clock += 3_000_000; 0 }
        rig.shooter.updateShot(0.5, ControlMode.MANUAL_POWER)
        assertEquals(0.5, rig.power1, 0.0)
        assertEquals(0, rig.voltageReads)
        // The worker uses the same profiler, with an explicitly separate background stage.
        rig.shooter.batteryVoltage.pollOnce()
        clock = 1_000_000_000
        val samples = profiler.snapshotIfDue()!!.associateBy { it.stage }
        assertEquals(3.0, samples.getValue(LoopStage.SHOOTER_ENCODERS).maxMs, 0.0)
        for (stage in listOf(LoopStage.SHOOTER_OUTPUT, LoopStage.SHOOTER_MODES,
                LoopStage.BLOCKER_OUTPUT, LoopStage.VOLTAGE_POLL)) {
            assertTrue(samples.getValue(stage).calls > 0)
        }
    }

    @Test fun `LQR equilibrium voltage tracks battery voltage without changing RPM target`() {
        val rig = Rig()
        rig.rpm(2880.0)
        for (voltage in listOf(12.0, 10.0, 14.0)) {
            rig.voltage = voltage
            rig.step(20)
            assertEquals(ShooterLqrTuning.kV * 2880.0 / voltage, rig.power1, 1e-9)
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

    @Test fun `Panels model changes affect shared LQR output on next loop`() {
        val rig = Rig()
        rig.rpm(2880.0)
        rig.step()
        val previous = rig.power1
        ShooterLqrTuning::class.java.getField("kV").setDouble(null, ShooterLqrTuning.kV * 1.1)
        rig.step(20)
        assertEquals(previous * 1.1, rig.power1, 1e-9)
        assertEquals(rig.power1, rig.power2, 0.0)
        verify(rig.motor1).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }

    @Test fun `invalid LQR model stops output and closes latched feeding`() {
        val rig = Rig()
        rig.rpm(2880.0)
        rig.step()
        assertTrue(rig.shooter.feedReady)
        ShooterLqrTuning.kA = 0.0
        rig.step(20)
        assertEquals(0.0, rig.power1, 0.0)
        assertEquals(0.0, rig.power2, 0.0)
        assertFalse(rig.shooter.feedReady)
        assertFalse(rig.shooter.blockerOpen)
        assertTrue(rig.shooter.speedStatus.startsWith("LQR disabled"))
    }

    @Test fun `transfer latches after RPM one reaches five percent and survives speed drops`() {
        val rig = Rig(60.0, 60.0)
        rig.rpm(2735.0, 2880.0)
        rig.step()
        assertFalse(rig.shooter.feedReady)
        rig.rpm(3025.0, 2880.0)
        rig.step(20)
        assertFalse(rig.shooter.feedReady)
        rig.rpm(2736.0, 0.0) // Flywheel 2 is ignored; RPM 1 is exactly -5%.
        rig.step(20)
        assertTrue(rig.shooter.feedReady)
        assertTrue(rig.shooter.blockerOpen)
        rig.rpm(3024.0, 6000.0) // RPM 1 exactly +5%.
        rig.step(20)
        assertTrue(rig.shooter.feedReady)
        rig.rpm(2735.0, 2880.0)
        rig.step(20)
        assertTrue(rig.shooter.feedReady)
        assertTrue(rig.shooter.blockerOpen)
        assertFalse(rig.shooter.atSpeed)
        assertEquals(0.64, FeedControls.intakePower(false, 0.0, 0.64, 0.64,
            ControlMode.VELOCITY_LQR, rig.shooter.feedReady, false), 0.0)
        assertEquals(0.0, FeedControls.intakePower(true, 0.0, 0.64, 0.64,
            ControlMode.VELOCITY_LQR, rig.shooter.feedReady, false), 0.0)
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
        rig.shooter.setControlMode(ControlMode.VELOCITY_LQR)
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
        val rig = Rig()
        rig.step()
        rig.step(10000)
        assertTrue(rig.power1 > 0.0)
        assertEquals(rig.power1, rig.power2, 0.0)
        assertFalse(rig.shooter.feedReady)
        rig.rpm(2880.0, 0.0)
        rig.step(20)
        assertTrue(rig.shooter.feedReady)
        rig.rpm(0.0, 2880.0)
        rig.step(20)
        assertTrue(rig.power1 > 0.0)
        assertTrue(rig.shooter.feedReady)
    }

    @Test fun `both motors use RPM one even when RPM two disagrees`() {
        val rig = Rig()
        rig.rpm(2800.0, 6000.0)
        rig.step()
        val first = rig.power1
        assertEquals(first, rig.power2, 0.0)
        rig.speed2 = Double.NaN
        rig.step(20)
        assertEquals(first, rig.power1, 1e-9)
        assertEquals(first, rig.power2, 1e-9)
    }

    @Test fun `invalid LQR weights keep both motors off while the button remains held`() {
        val rig = Rig()
        rig.step()
        ShooterLqrTuning.rpmTolerance = 0.0
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
        rig.step(20, mode = ControlMode.VELOCITY_LQR)
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
        ShooterLqrTuning.kV = Double.NaN
        val rig = Rig()
        rig.step()
        assertEquals(0.0, rig.power1, 0.0)
        assertFalse(rig.shooter.feedReady)
        restoreModel()
        val badProfile = Rig(0.0, 28.0)
        badProfile.step()
        assertEquals(0.0, badProfile.power1, 0.0)
        assertFalse(badProfile.shooter.feedReady)
    }
    @Test fun `invalid flywheel one feedback closes previously latched feeding`() {
        val rig = Rig()
        rig.rpm(2880.0)
        rig.step()
        assertTrue(rig.shooter.feedReady)
        rig.speed1 = Double.NaN
        rig.step(20)
        assertEquals(0.0, rig.power1, 0.0)
        assertFalse(rig.shooter.feedReady)
        assertFalse(rig.shooter.blockerOpen)
    }

    @Test fun `legacy PID mode name delegates to LQR for old autonomous callers`() {
        val rig = Rig()
        rig.rpm(2880.0)
        rig.shooter.updateShot(0.64, ControlMode.VELOCITY_PID)
        assertEquals(ControlMode.VELOCITY_LQR, rig.shooter.controlMode)
        assertEquals(ShooterLqrTuning.kV * 2880.0 / 12.0, rig.power1, 1e-9)
    }

    @Test fun `LQR uses observed interval and resets to nominal interval after release`() {
        val rig = Rig()
        rig.rpm(2800.0)
        rig.step(20)
        rig.step(66)
        val oracle = FlywheelLqr()
        oracle.calculate(2800.0, 2880.0, 0.066, ShooterLqrTuning.snapshot())
        assertEquals(oracle.feedbackGain, rig.shooter.lqrFeedbackGain, 1e-12)
        rig.shooter.setFlywheelPower(0.0)
        rig.step(5000)
        oracle.calculate(2800.0, 2880.0, 0.020, ShooterLqrTuning.snapshot())
        assertEquals(oracle.feedbackGain, rig.shooter.lqrFeedbackGain, 1e-12)
    }

}
