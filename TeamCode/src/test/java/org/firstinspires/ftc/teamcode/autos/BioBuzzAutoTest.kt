package org.firstinspires.ftc.teamcode.autos

import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Subsytem.ShooterLqrTuning
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.junit.Assert.*
import org.junit.Test
import kotlin.math.abs
import org.mockito.ArgumentMatchers.doubleThat
import org.mockito.ArgumentCaptor
import org.mockito.Mockito.*

class BioBuzzAutoTest {
    @Test fun `basicred shoots for two seconds after readiness then follows its path once and stops`() =
        verifyBasicAuto(blue = false)

    @Test fun `basicblue uses the same speed and timing then follows its exported path once and stops`() =
        verifyBasicAuto(blue = true)

    private fun verifyBasicAuto(blue: Boolean) {
        val follower = mock(Follower::class.java)
        val map = mock(HardwareMap::class.java)
        val hub = mock(LynxModule::class.java)
        `when`(map.getAll(LynxModule::class.java)).thenReturn(listOf(hub))
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
        var now = 0L
        val auto = if (blue) BasicBlueAuto { now } else BioBuzzAuto { now }
        val startX = if (blue) 83.07841726618705 else 57.01798561151079
        val startY = if (blue) 132.19424460431654 else 9.221582733812928
        val endX = if (blue) 132.9597122302158 else 8.96906474820144
        val endY = if (blue) 41.497122302158274 else 97.8935251798561
        val pathHeading = Math.toRadians(if (blue) 90.0 else 270.0)
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
            verify(hub).bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
            val initialPose = ArgumentCaptor.forClass(Pose::class.java)
            verify(follower).setPose(initialPose.capture())
            assertEquals(startX, initialPose.value.x(), 1e-9)
            assertEquals(startY, initialPose.value.y(), 1e-9)
            assertEquals(Math.PI / 2, initialPose.value.heading(), 1e-9)
            auto.start()
            auto.loop() // Spin up while stationary: the two-second timer has not started.
            now = 10_000_000_000L
            auto.loop()
            verify(follower, never()).follow(any(Path::class.java))
            // Initial LQR spin-up commands forward power at the shared 3240 RPM target.
            for (name in listOf("shooter1", "shooter2"))
                verify(motors.getValue(name), atLeastOnce()).power = doubleThat { it > 0.0 && it <= 1.0 }
            verify(auto.telemetry, atLeastOnce()).addData("Shooter target RPM", 3240.0)
            verify(motors.getValue("intake1"), never()).power = 0.72
            `when`(motors.getValue("shooter1").velocity).thenReturn(3240.0 * 28 / 60)
            auto.loop() // RPM 1 reaches target; begin the full two seconds of feeding.
            verify(motors.getValue("intake1")).power = 0.72
            `when`(motors.getValue("shooter1").velocity).thenReturn(0.0)
            now += 1_999_000_000L
            auto.loop() // An RPM dip does not reset feeding or start driving early.
            verify(follower, never()).follow(any(Path::class.java))
            now += 1_000_000L
            auto.loop() // Exactly two seconds: stop shooting and start path 1.
            verify(follower).follow(any(Path::class.java))
            val followed = ArgumentCaptor.forClass(Path::class.java)
            verify(follower).follow(followed.capture())
            val path = followed.value
            assertEquals(startX, path.get(0.0).x(), 1e-9)
            assertEquals(startY, path.get(0.0).y(), 1e-9)
            assertEquals(endX, path.endPose().x(), 1e-9)
            assertEquals(endY, path.endPose().y(), 1e-9)
            for (t in listOf(0.0, 0.5, 1.0))
                assertEquals(pathHeading, path.heading(t), 1e-9)
            // During the entire path the mechanisms stay off; no second shot is started.
            clearInvocations(*motors.values.toTypedArray(), blocker)
            repeat(3) { now += 1_000_000_000L; auto.loop() }
            verifyNoInteractions(*motors.values.toTypedArray(), blocker)
            verify(follower, times(1)).follow(any(Path::class.java))
            clearInvocations(follower)
            `when`(follower.isBusy).thenReturn(false)
            auto.loop()
            for (motor in motors.values) verify(motor).power = 0.0
            verify(blocker, atLeastOnce()).position = 0.78
            verify(follower).stop()
            verify(hub, atLeastOnce()).clearBulkCache()
            verify(hub).bulkCachingMode = LynxModule.BulkCachingMode.AUTO
            verify(follower, never()).follow(any(Path::class.java))
            clearInvocations(*motors.values.toTypedArray(), follower)
            auto.loop() // Completion cannot restart shooting or the path.
            verifyNoInteractions(*motors.values.toTypedArray(), follower)
            auto.stop() // A later Driver Station STOP remains safe.
        } finally {
            auto.stop()
            ShooterLqrTuning.kV = gains[0]
            ShooterLqrTuning.kA = gains[1]
            ShooterLqrTuning.rpmTolerance = gains[2]
            ShooterLqrTuning.voltageTolerance = gains[3]
        }
    }
}
