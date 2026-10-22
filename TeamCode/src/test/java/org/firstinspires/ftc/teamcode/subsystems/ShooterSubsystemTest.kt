package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.teamcode.control.FlywheelLqr
import org.firstinspires.ftc.teamcode.control.ShooterController.ControlMode
import org.firstinspires.ftc.teamcode.control.ShooterLqrTuning
import org.firstinspires.ftc.teamcode.diagnostics.LoopProfiler
import org.firstinspires.ftc.teamcode.diagnostics.LoopStage
import org.firstinspires.ftc.teamcode.hardware.BioBuzzShooterMotor
import org.firstinspires.ftc.teamcode.subsystems.FeedControls
import org.junit.After
import org.junit.Assert.*
import org.junit.Before
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

    @Test fun `100 Hz LQR ticks use fresh feedback without repeated mode or telemetry reads`() {
        val rig = Rig()
        clearInvocations(rig.motor1, rig.motor2, rig.blocker)
        val oracle = FlywheelLqr()
        repeat(100) { index ->
            val rpm = 2700.0 + index
            rig.rpm(rpm, 0.0)
            rig.step(10)
            val dt = if (index == 0) 0.020 else 0.010
            val expected = (oracle.calculate(rpm, 2880.0, dt, ShooterLqrTuning.snapshot())!!/12.0).coerceIn(0.0,1.0)
            assertEquals(expected, rig.power1, 1e-9)
            assertEquals(expected, rig.power2, 1e-9)
            assertEquals(rpm, rig.shooter.wheels[0].rpm, 1e-9)
            assertEquals(expected, rig.shooter.flywheelPower, 1e-9)
            assertEquals(expected, rig.shooter.flywheelPower, 1e-9)
            assertEquals(0.78, rig.shooter.control.blockerPosition, 0.0)
            if (index > 0) assertEquals(100.0, rig.shooter.control.lqrFrequencyHz, 1e-9)
        }
        for (motor in listOf(rig.motor1, rig.motor2)) {
            verify(motor, times(100)).velocity
            verify(motor, times(100)).currentPosition
            verify(motor, never()).setMode(any(DcMotor.RunMode::class.java))
            verify(motor, never()).power
        }
        verify(rig.blocker, never()).position
        rig.shooter.setFlywheelPower(0.0)
        assertEquals(0.0, rig.shooter.control.lqrFrequencyHz, 0.0)
        assertEquals(0.0, rig.shooter.flywheelPower, 0.0)
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

    @Test fun `manual feeding waits three seconds and a new preset restarts the wait`() {
        val rig = Rig()
        rig.step(mode = ControlMode.MANUAL_POWER)
        rig.step(2999, mode = ControlMode.MANUAL_POWER)
        assertFalse(rig.shooter.control.feedReady)
        assertFalse(rig.shooter.control.blockerOpen)
        rig.step(1, mode = ControlMode.MANUAL_POWER)
        assertTrue(rig.shooter.control.feedReady)
        assertTrue(rig.shooter.control.blockerOpen)
        assertFalse(rig.shooter.control.atSpeed) // Timed readiness never claims measured speed.
        rig.step(1, power = 0.7, mode = ControlMode.MANUAL_POWER)
        assertFalse(rig.shooter.control.feedReady)
        assertFalse(rig.shooter.control.blockerOpen)
    }

    @Test fun `force stop tries both flywheels and blocker and retains every failure`() {
        val rig = Rig()
        rig.step(mode = ControlMode.MANUAL_POWER)
        rig.shooter.setManualBlockerOpen(true)
        val motorFailure = IllegalStateException("flywheel disconnected")
        val servoFailure = IllegalStateException("blocker disconnected")
        clearInvocations(rig.motor1, rig.motor2, rig.blocker)
        doThrow(motorFailure).`when`(rig.motor1).setPower(0.0)
        doThrow(servoFailure).`when`(rig.blocker).setPosition(0.78)
        try {
            rig.shooter.forceStop()
            fail("Expected the first hardware failure")
        } catch (failure: IllegalStateException) {
            assertSame(motorFailure, failure)
            assertArrayEquals(arrayOf(servoFailure), failure.suppressed)
        }
        verify(rig.motor1).power = 0.0
        verify(rig.motor2).power = 0.0
        verify(rig.blocker).position = 0.78
        verify(rig.motor1, never()).velocity
        verify(rig.motor2, never()).velocity
        assertFalse(rig.shooter.control.blockerOpen)
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
        for (stage in listOf(LoopStage.SHOOTER_OUTPUT,
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
            assertEquals(2880.0, rig.shooter.control.targetRpm, 1e-9)
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
        assertEquals(6000.0, profile.maxRPM, 0.0)
        val device = BioBuzzShooterMotor::class.java.getAnnotation(DeviceProperties::class.java)!!
        assertEquals("BioBuzzShooter1To1", device.xmlTag)
        val rig = Rig(profile.ticksPerRev, profile.ticksPerRev)
        rig.speed1 = 50.0 * 2786.0 / 60.0
        rig.speed2 = -rig.speed1
        rig.step()
        assertEquals(4975.0, rig.shooter.wheels[0].rpm, 1e-9)
        assertEquals(4975.0, rig.shooter.wheels[1].rpm, 1e-9)
        assertEquals(2880.0, rig.shooter.control.targetRpm, 1e-9)
        assertEquals(1344.0, rig.shooter.wheels[0].targetTicksPerSecond(rig.shooter.control.targetRpm), 1e-9)
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

    @Test fun `held shooting observes every live model field and recovers after invalid tuning`() {
        val rig = Rig()
        rig.rpm(2800.0)
        rig.step()
        val changes = listOf<() -> Unit>(
            { ShooterLqrTuning.kV *= .95 },
            { ShooterLqrTuning.kA *= .8 },
            { ShooterLqrTuning.rpmTolerance *= 1.5 },
            { ShooterLqrTuning.voltageTolerance *= .9 }
        )
        for (change in changes) {
            change()
            repeat(2) {
                rig.step(20)
                val volts = FlywheelLqr().calculate(
                    2800.0, 2880.0, .020, ShooterLqrTuning.snapshot())!!
                assertEquals((volts / 12.0).coerceIn(0.0, 1.0), rig.power1, 1e-9)
                assertEquals(rig.power1, rig.power2, 0.0)
            }
        }
        rig.rpm(2880.0)
        val invalidations = listOf<() -> Unit>(
            { ShooterLqrTuning.kV = Double.NaN },
            { ShooterLqrTuning.kA = 0.0 },
            { ShooterLqrTuning.rpmTolerance = -1.0 },
            { ShooterLqrTuning.voltageTolerance = Double.POSITIVE_INFINITY }
        )
        for (invalidate in invalidations) {
            invalidate()
            repeat(2) {
                rig.step(20)
                assertEquals(0.0, rig.power1, 0.0)
                assertEquals(0.0, rig.power2, 0.0)
                assertFalse(rig.shooter.control.feedReady)
                assertFalse(rig.shooter.control.blockerOpen)
            }
            restoreModel()
            rig.step(20)
            assertTrue(rig.shooter.control.feedReady)
            assertTrue(rig.shooter.control.blockerOpen)
            assertEquals(ShooterLqrTuning.kV * 2880.0 / 12.0, rig.power1, 1e-9)
        }
    }

    @Test fun `invalid LQR model stops output and closes latched feeding`() {
        val rig = Rig()
        rig.rpm(2880.0)
        rig.step()
        assertTrue(rig.shooter.control.feedReady)
        ShooterLqrTuning.kA = 0.0
        rig.step(20)
        assertEquals(0.0, rig.power1, 0.0)
        assertEquals(0.0, rig.power2, 0.0)
        assertFalse(rig.shooter.control.feedReady)
        assertFalse(rig.shooter.control.blockerOpen)
        assertTrue(rig.shooter.control.speedStatus.startsWith("LQR disabled"))
    }

    @Test fun `transfer latches after RPM one reaches one percent and survives speed drops`() {
        val rig = Rig(60.0, 60.0)
        rig.rpm(2735.0, 2880.0)
        rig.step()
        assertFalse(rig.shooter.control.feedReady)
        rig.rpm(3025.0, 2880.0)
        rig.step(20)
        assertFalse(rig.shooter.control.feedReady)
        rig.rpm(2852.0, 0.0) // Flywheel 2 is ignored; RPM 1 is just inside -1%.
        rig.step(20)
        assertTrue(rig.shooter.control.feedReady)
        assertTrue(rig.shooter.control.blockerOpen)
        rig.rpm(2908.0, 6000.0) // RPM 1 just inside +1%.
        rig.step(20)
        assertTrue(rig.shooter.control.feedReady)
        rig.rpm(2735.0, 2880.0)
        rig.step(20)
        assertTrue(rig.shooter.control.feedReady)
        assertTrue(rig.shooter.control.blockerOpen)
        assertFalse(rig.shooter.control.atSpeed)
        assertEquals(0.64, FeedControls.intakePower(false, 0.0, 0.64, 0.64,
            ControlMode.VELOCITY_LQR, rig.shooter.control.feedReady, false), 0.0)
        assertEquals(0.0, FeedControls.intakePower(true, 0.0, 0.64, 0.64,
            ControlMode.VELOCITY_LQR, rig.shooter.control.feedReady, false), 0.0)
        rig.rpm(3025.0, 2880.0)
        rig.step(20)
        assertTrue(rig.shooter.control.feedReady)
        rig.rpm(2880.0, 2880.0)
        rig.step(20)
        assertTrue(rig.shooter.control.feedReady)
    }

    @Test fun `release reverse and mode changes reset the feed latch`() {
        val rig = Rig()
        rig.rpm(2880.0)
        rig.step()
        assertTrue(rig.shooter.control.feedReady)
        rig.shooter.setFlywheelPower(0.0)
        assertFalse(rig.shooter.control.feedReady)
        assertFalse(rig.shooter.control.blockerOpen)
        rig.rpm(2000.0)
        rig.step(20)
        assertFalse(rig.shooter.control.feedReady)
        rig.rpm(2880.0)
        rig.step(20)
        assertTrue(rig.shooter.control.feedReady)
        rig.shooter.setFlywheelPower(-0.8)
        assertFalse(rig.shooter.control.feedReady)
        rig.rpm(2880.0)
        rig.step(20)
        assertTrue(rig.shooter.control.feedReady)
        rig.shooter.setControlMode(ControlMode.MANUAL_POWER)
        assertFalse(rig.shooter.control.feedReady)
        rig.shooter.setControlMode(ControlMode.VELOCITY_LQR)
        rig.rpm(2000.0)
        rig.step(20)
        assertFalse(rig.shooter.control.feedReady)
    }

    @Test fun `each motor uses its own encoder counts and a preset change resets readiness`() {
        val rig = Rig(28.0, 56.0)
        rig.rpm(2880.0)
        rig.step()
        assertEquals(2880.0, rig.shooter.wheels[0].rpm, 1e-9)
        assertEquals(2880.0, rig.shooter.wheels[1].rpm, 1e-9)
        assertTrue(rig.shooter.control.feedReady)
        rig.step(20, power = 0.8)
        assertEquals(3600.0, rig.shooter.control.targetRpm, 0.0)
        assertFalse(rig.shooter.control.feedReady)
    }

    @Test fun `zero RPM never triggers a timed motor shutdown`() {
        val rig = Rig()
        rig.step()
        rig.step(10000)
        assertTrue(rig.power1 > 0.0)
        assertEquals(rig.power1, rig.power2, 0.0)
        assertFalse(rig.shooter.control.feedReady)
        rig.rpm(2880.0, 0.0)
        rig.step(20)
        assertTrue(rig.shooter.control.feedReady)
        rig.rpm(0.0, 2880.0)
        rig.step(20)
        assertTrue(rig.power1 > 0.0)
        assertTrue(rig.shooter.control.feedReady)
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
        assertFalse(rig.shooter.control.feedReady)
    }

    @Test fun `manual mode runs raw power and switching or releasing closes feeding`() {
        val rig = Rig()
        rig.step(mode = ControlMode.MANUAL_POWER)
        assertEquals(0.64, rig.power1, 0.0)
        rig.shooter.setManualBlockerOpen(true)
        assertTrue(rig.shooter.control.blockerOpen)
        rig.step(20, mode = ControlMode.VELOCITY_LQR)
        assertFalse(rig.shooter.control.blockerOpen)
        rig.shooter.setFlywheelPower(0.0)
        assertEquals(0.0, rig.power1, 0.0)
        assertEquals(0.0, rig.power2, 0.0)
        assertEquals(0.0, rig.shooter.control.targetRpm, 0.0)
        rig.shooter.setFlywheelPower(-0.8)
        assertEquals(-0.8, rig.power1, 0.0)
        assertFalse(rig.shooter.control.feedReady)
    }

    @Test fun `invalid numeric input never produces NaN motor power`() {
        ShooterLqrTuning.kV = Double.NaN
        val rig = Rig()
        rig.step()
        assertEquals(0.0, rig.power1, 0.0)
        assertFalse(rig.shooter.control.feedReady)
        restoreModel()
        val badProfile = Rig(0.0, 28.0)
        badProfile.step()
        assertEquals(0.0, badProfile.power1, 0.0)
        assertFalse(badProfile.shooter.control.feedReady)
    }
    @Test fun `invalid flywheel one feedback closes previously latched feeding`() {
        val rig = Rig()
        rig.rpm(2880.0)
        rig.step()
        assertTrue(rig.shooter.control.feedReady)
        rig.speed1 = Double.NaN
        rig.step(20)
        assertEquals(0.0, rig.power1, 0.0)
        assertFalse(rig.shooter.control.feedReady)
        assertFalse(rig.shooter.control.blockerOpen)
    }

    @Test fun `LQR mode uses feedforward voltage at the target RPM`() {
        val rig = Rig()
        rig.rpm(2880.0)
        rig.shooter.updateShot(0.64, ControlMode.VELOCITY_LQR)
        assertEquals(ControlMode.VELOCITY_LQR, rig.shooter.control.controlMode)
        assertEquals(ShooterLqrTuning.kV * 2880.0 / 12.0, rig.power1, 1e-9)
    }

    @Test fun `LQR uses observed interval and resets to nominal interval after release`() {
        val rig = Rig()
        rig.rpm(2800.0)
        rig.step(20)
        rig.step(66)
        val oracle = FlywheelLqr()
        oracle.calculate(2800.0, 2880.0, 0.066, ShooterLqrTuning.snapshot())
        assertEquals(oracle.feedbackGain, rig.shooter.control.lqrFeedbackGain, 1e-12)
        rig.shooter.setFlywheelPower(0.0)
        rig.step(5000)
        oracle.calculate(2800.0, 2880.0, 0.020, ShooterLqrTuning.snapshot())
        assertEquals(oracle.feedbackGain, rig.shooter.control.lqrFeedbackGain, 1e-12)
    }

}
