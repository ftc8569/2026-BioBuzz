package org.firstinspires.ftc.teamcode.autos

import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Subsytem.ShooterLqrTuning
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.junit.Assert.*
import org.junit.Test
import org.mockito.ArgumentCaptor
import org.mockito.Mockito.*

class BioBuzzAutoTest {
    @Test fun `four imported paths collect only on path two then shoot and latch feeding`() {
        val follower = mock(Follower::class.java)
        val map = mock(HardwareMap::class.java)
        val motors = listOf("shooter1", "shooter2", "intake1", "intake2")
            .associateWith { mock(DcMotorEx::class.java) }
        val blocker = mock(Servo::class.java)
        val profile = mock(MotorConfigurationType::class.java)
        `when`(profile.ticksPerRev).thenReturn(28.0)
        for ((name, motor) in motors) {
            `when`(map.get(DcMotorEx::class.java, name)).thenReturn(motor)
            `when`(motor.motorType).thenReturn(profile)
        }
        `when`(map.get(Servo::class.java, "blocker")).thenReturn(blocker)
        `when`(follower.pose()).thenReturn(Pose(0.0, 0.0, 0.0))
        `when`(follower.isBusy).thenReturn(true)
        val auto = BioBuzzAuto()
        auto.hardwareMap = map
        auto.telemetry = mock(Telemetry::class.java)
        val gains = doubleArrayOf(ShooterLqrTuning.kV, ShooterLqrTuning.kA,
            ShooterLqrTuning.rpmTolerance, ShooterLqrTuning.voltageTolerance)
        ShooterLqrTuning.kV = ShooterLqrTuning.DEFAULT_KV
        ShooterLqrTuning.kA = ShooterLqrTuning.DEFAULT_KA
        ShooterLqrTuning.rpmTolerance = ShooterLqrTuning.DEFAULT_RPM_TOLERANCE
        ShooterLqrTuning.voltageTolerance = ShooterLqrTuning.DEFAULT_VOLTAGE_TOLERANCE
        try {
            mockStatic(Constants::class.java).use { constants ->
                constants.`when`<Follower> { Constants.create(map) }.thenReturn(follower)
                auto.init()
            }
            val initialPose = ArgumentCaptor.forClass(Pose::class.java)
            verify(follower).setPose(initialPose.capture())
            assertEquals(56.0, initialPose.value.x(), 1e-9)
            assertEquals(11.461151079136723, initialPose.value.y(), 1e-9)
            assertEquals(Math.PI / 2, initialPose.value.heading(), 1e-9)
            auto.start()
            auto.loop() // Path 1: no collecting or shooting.
            verify(motors.getValue("intake1"), never()).power = 1.0
            verify(motors.getValue("shooter1"), never()).power = 1.0

            `when`(follower.isBusy).thenReturn(false)
            auto.loop() // Enter path 2: intake starts in the same loop.
            `when`(follower.isBusy).thenReturn(true)
            repeat(3) {
                clearInvocations(*motors.values.toTypedArray())
                auto.loop()
                for (name in listOf("intake1", "intake2"))
                    verify(motors.getValue(name)).power = 1.0
                for (name in listOf("shooter1", "shooter2"))
                    verify(motors.getValue(name)).power = 0.0
            }
            `when`(follower.isBusy).thenReturn(false)
            for (stage in 3..4) {
                clearInvocations(*motors.values.toTypedArray())
                auto.loop()
                for (motor in motors.values) verify(motor).power = 0.0
            }
            auto.loop() // Finish path 4: start shooting, wait to feed.
            for (name in listOf("shooter1", "shooter2"))
                verify(motors.getValue(name)).power = 1.0
            verify(follower).hold(any(Pose::class.java))
            val followed = ArgumentCaptor.forClass(Path::class.java)
            verify(follower, times(4)).follow(followed.capture())
            val ends = listOf(
                doubleArrayOf(43.580575539568336, 9.1251798561151, 180.0),
                doubleArrayOf(13.01654676258992, 7.334532374100731, 180.0),
                doubleArrayOf(12.999280575539576, 14.346762589928069, 270.0),
                doubleArrayOf(57.44892086330934, 9.968345323740998, 270.0))
            for ((path, end) in followed.allValues.zip(ends)) {
                assertEquals(end[0], path.endPose().x(), 1e-9)
                assertEquals(end[1], path.endPose().y(), 1e-9)
                assertEquals(Math.toRadians(end[2]), path.endPose().heading(), 1e-9)
            }
            // Check each heading range, including the exported 90-degree initial pose
            // followed by path 1's independent 270 -> 180-degree interpolation.
            val starts = listOf(270.0, 180.0, 180.0, 270.0)
            for ((index, path) in followed.allValues.withIndex()) {
                assertEquals(Math.toRadians(starts[index]), path.heading(0.0), 1e-9)
                assertEquals(Math.toRadians((starts[index] + ends[index][2]) / 2),
                    path.heading(0.5), 1e-9)
            }
            // RPM 2 remains zero: only RPM 1 opens the gate, and feeding stays latched.
            `when`(motors.getValue("shooter1").velocity).thenReturn(3150.0 * 28 / 60)
            auto.loop()
            verify(blocker).position = 0.65
            `when`(motors.getValue("shooter1").velocity).thenReturn(0.0)
            clearInvocations(*motors.values.toTypedArray())
            auto.loop()
            for (name in listOf("intake1", "intake2"))
                verify(motors.getValue(name)).power = 0.70
            clearInvocations(*motors.values.toTypedArray(), blocker)
            auto.stop()
            for (motor in motors.values) verify(motor).power = 0.0
            verify(blocker, atLeastOnce()).position = 0.78
            verify(follower).stop()
        } finally {
            auto.stop()
            ShooterLqrTuning.kV = gains[0]
            ShooterLqrTuning.kA = gains[1]
            ShooterLqrTuning.rpmTolerance = gains[2]
            ShooterLqrTuning.voltageTolerance = gains[3]
        }
    }
}
