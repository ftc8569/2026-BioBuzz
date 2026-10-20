package org.firstinspires.ftc.teamcode.autos

import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.ShooterLqrTuning
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.junit.Assert.*
import org.junit.Test
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sin
import kotlin.math.cos
import org.mockito.ArgumentMatchers.doubleThat
import org.mockito.ArgumentMatchers.anyDouble
import org.mockito.ArgumentCaptor
import org.mockito.Mockito.*

class BioBuzzAutoTest {
    private enum class AutoKind { BASIC_RED, BASIC_BLUE, RED_PRELOAD_FAR, RED_PRELOAD_CLOSE, BLUE_PRELOAD_FAR, BLUE_PRELOAD_CLOSE }
    @Test fun `basicred shoots for two seconds after readiness then follows its path once and stops`() =
        verifyBasicAuto(AutoKind.BASIC_RED)

    @Test fun `basicblue uses the same speed and timing then follows its exported path once and stops`() =
        verifyBasicAuto(AutoKind.BASIC_BLUE)

    @Test fun `preload far park shoots before following its exported curve and stops`() =
        verifyBasicAuto(AutoKind.RED_PRELOAD_FAR)

    @Test fun `red preload close shoots before following its curve and stops`() =
        verifyBasicAuto(AutoKind.RED_PRELOAD_CLOSE)

    @Test fun `blue preload far rotates the red curve and shoots before parking`() =
        verifyBasicAuto(AutoKind.BLUE_PRELOAD_FAR)

    @Test fun `blue preload close rotates the red curve and shoots before parking`() =
        verifyBasicAuto(AutoKind.BLUE_PRELOAD_CLOSE)

    @Test fun `STOP before Start leaves preload blocker untouched`() =
        verifyBasicAuto(AutoKind.RED_PRELOAD_CLOSE, stopBeforeStart = true)

    private fun verifyBasicAuto(kind: AutoKind, stopBeforeStart: Boolean = false) {
        val blue = kind in setOf(AutoKind.BASIC_BLUE, AutoKind.BLUE_PRELOAD_FAR, AutoKind.BLUE_PRELOAD_CLOSE)
        val preloadFarPark = kind in setOf(AutoKind.RED_PRELOAD_FAR, AutoKind.BLUE_PRELOAD_FAR)
        val preloadClosePark = kind in setOf(AutoKind.RED_PRELOAD_CLOSE, AutoKind.BLUE_PRELOAD_CLOSE)
        val rotate = blue && (preloadFarPark || preloadClosePark)
        fun coordinate(red: Double): Double = if (rotate) 141.5 - red else red
        val shooterPreset = ShooterSubsystem.DEFAULT_SHOOTER_PRESET
        val targetRpm = shooterPreset * ShooterSubsystem.MAX_MOTOR_RPM
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
        val auto = when {
            kind == AutoKind.BLUE_PRELOAD_FAR -> BluePreloadParkFar { now }
            kind == AutoKind.BLUE_PRELOAD_CLOSE -> BluePreloadParkClose { now }
            preloadFarPark -> RedPreloadParkFar { now }
            preloadClosePark -> RedPreloadParkClose { now }
            blue -> BasicBlueAuto { now }
            else -> BioBuzzAuto { now }
        }
        val expectedName = when (kind) {
            AutoKind.RED_PRELOAD_FAR -> "RedPreloadParkFar"
            AutoKind.RED_PRELOAD_CLOSE -> "RedPreloadParkClose"
            AutoKind.BLUE_PRELOAD_FAR -> "BluePreloadParkFar"
            AutoKind.BLUE_PRELOAD_CLOSE -> "BluePreloadParkClose"
            else -> null
        }
        if (expectedName != null)
            assertEquals(expectedName, auto.javaClass.getAnnotation(Autonomous::class.java).name)
        val startX = if (preloadFarPark || preloadClosePark) 58.4934 else if (blue) 83.07841726618705 else 57.01798561151079
        val startY = if (preloadClosePark) 6.6 else if (preloadFarPark) 9.0389 else if (blue) 132.19424460431654 else 9.221582733812928
        val endX = if (preloadClosePark) 11.3267 else if (preloadFarPark) 12.3656 else if (blue) 132.9597122302158 else 8.96906474820144
        val endY = if (preloadClosePark) 94.1791 else if (preloadFarPark) 117.243 else if (blue) 41.497122302158274 else 97.8935251798561
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
            verify(blocker, never()).setPosition(anyDouble())
            verify(hub).bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
            val initialPose = ArgumentCaptor.forClass(Pose::class.java)
            verify(follower).setPose(initialPose.capture())
            assertEquals(coordinate(startX), initialPose.value.x(), 1e-9)
            assertEquals(coordinate(startY), initialPose.value.y(), 1e-9)
            assertEquals(if ((preloadFarPark || preloadClosePark) && !blue) 3 * Math.PI / 2 else Math.PI / 2,
                initialPose.value.heading(), 1e-9)
            if (stopBeforeStart) {
                auto.stop()
                verify(blocker, never()).setPosition(anyDouble())
                return
            }
            auto.start()
            verify(blocker).position = 0.78
            auto.loop() // Spin up while stationary: the two-second timer has not started.
            now = 10_000_000_000L
            auto.loop()
            verify(follower, never()).follow(any(Path::class.java))
            // Initial LQR spin-up commands forward power at the shared preset RPM target.
            for (name in listOf("shooter1", "shooter2"))
                verify(motors.getValue(name), atLeastOnce()).power = doubleThat { it > 0.0 && it <= 1.0 }
            verify(auto.telemetry, atLeastOnce()).addData("Shooter target RPM", targetRpm)
            verify(motors.getValue("intake1"), never()).power = shooterPreset
            `when`(motors.getValue("shooter1").velocity).thenReturn(targetRpm * 28 / 60)
            auto.loop() // RPM 1 reaches target; open the blocker before feeding.
            verify(motors.getValue("intake1"), never()).power = shooterPreset
            now += 199_000_000L
            auto.loop()
            verify(motors.getValue("intake1"), never()).power = shooterPreset
            verify(follower, never()).follow(any(Path::class.java))
            now += 1_000_000L
            auto.loop() // Exactly 200 ms after opening: begin two full seconds of feeding.
            verify(motors.getValue("intake1")).power = shooterPreset
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
            assertEquals(coordinate(startX), path.get(0.0).x(), 1e-9)
            assertEquals(coordinate(startY), path.get(0.0).y(), 1e-9)
            assertEquals(coordinate(endX), path.endPose().x(), 1e-9)
            assertEquals(coordinate(endY), path.endPose().y(), 1e-9)
            for (t in listOf(0.0, 0.25, 0.5, 1.0)) {
                val expectedHeading = if (preloadClosePark)
                    Math.toRadians(270.0 - 180.0 * path.curve.pathCompletion(t) + if (rotate) 180.0 else 0.0)
                    else pathHeading
                val error = path.heading(t) - expectedHeading
                assertEquals(0.0, atan2(sin(error), cos(error)), 1e-9)
            }
            if (preloadClosePark) {
                assertEquals(coordinate(37.3911921875), path.get(0.25).x(), 1e-9)
                assertEquals(coordinate(22.070396875), path.get(0.25).y(), 1e-9)
                assertEquals(coordinate(34.4334625), path.get(0.5).x(), 1e-9)
                assertEquals(coordinate(52.892825), path.get(0.5).y(), 1e-9)
            }
            if (preloadFarPark) {
                assertEquals(coordinate(37.34898125), path.get(0.25).x(), 1e-9)
                assertEquals(coordinate(26.7322640625), path.get(0.25).y(), 1e-9)
                assertEquals(coordinate(34.407475), path.get(0.5).x(), 1e-9)
                assertEquals(coordinate(64.8075625), path.get(0.5).y(), 1e-9)
            }
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
