package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.junit.Assert.*
import org.junit.Test
import org.mockito.ArgumentMatchers.anyDouble
import org.mockito.Mockito.*

/** Run the installed Pedro follower, localizer, drivetrain and LQR; mock hardware only. */
class BasicAutoIntegrationTest {
    @Test fun `real Pedro and LQR run red lifecycle from init through first drive update`() = runAuto(false)
    @Test fun `real Pedro and LQR run blue lifecycle from init through first drive update`() = runAuto(true)

    private fun runAuto(blue: Boolean) {
        val map = mock(HardwareMap::class.java)
        val pinpoint = mock(GoBildaPinpointDriver::class.java)
        val motors = listOf("driveFL", "driveFR", "driveBL", "driveBR", "shooter1", "shooter2", "intake1", "intake2")
            .associateWith { mock(DcMotorEx::class.java) }
        val powers = mutableMapOf<String, Double>()
        // Reproduce motors left in position mode by a previous OpMode.
        val modes = motors.mapValues { DcMotor.RunMode.RUN_TO_POSITION }.toMutableMap()
        val profile = mock(MotorConfigurationType::class.java)
        `when`(profile.ticksPerRev).thenReturn(28.0)
        for ((name, motor) in motors) {
            `when`(map.get(DcMotorEx::class.java, name)).thenReturn(motor)
            `when`(motor.motorType).thenReturn(profile)
            doAnswer { modes[name] = it.getArgument(0); null }.`when`(motor).setMode(any(DcMotor.RunMode::class.java))
            doAnswer {
                check(modes[name] == DcMotor.RunMode.RUN_WITHOUT_ENCODER) { "$name still in position mode" }
                val power = it.getArgument<Double>(0)
                assertTrue("$name received invalid power $power", power.isFinite() && power in -1.0..1.0)
                powers[name] = power
                null
            }.`when`(motor).setPower(anyDouble())
        }
        `when`(map.get(Servo::class.java, "blocker")).thenReturn(mock(Servo::class.java))
        `when`(map.get(GoBildaPinpointDriver::class.java, "pinpoint")).thenReturn(pinpoint)
        var pose = Pose2D(DistanceUnit.INCH, 0.0, 0.0, AngleUnit.RADIANS, 0.0)
        doAnswer { pose = it.getArgument(0); null }.`when`(pinpoint).setPosition(any(Pose2D::class.java))
        `when`(pinpoint.getPosX(DistanceUnit.INCH)).thenAnswer { pose.getX(DistanceUnit.INCH) }
        `when`(pinpoint.getPosY(DistanceUnit.INCH)).thenAnswer { pose.getY(DistanceUnit.INCH) }
        `when`(pinpoint.getHeading(AngleUnit.RADIANS)).thenAnswer { pose.getHeading(AngleUnit.RADIANS) }
        var now = 0L
        val auto = if (blue) BasicBlueAuto { now } else BioBuzzAuto { now }
        auto.hardwareMap = map
        auto.telemetry = mock(Telemetry::class.java)
        try {
            auto.init()
            auto.start()
            auto.loop()
            assertTrue(powers.getValue("shooter1") > 0.0)
            for (name in listOf("driveFL", "driveFR", "driveBL", "driveBR"))
                assertEquals(0.0, powers.getOrDefault(name, 0.0), 0.0)
            `when`(motors.getValue("shooter1").velocity).thenReturn(3240.0 * 28.0 / 60.0)
            now += 10_000_000L
            auto.loop()
            now += 100_000_000L
            auto.loop()
            now += 2_000_000_000L
            auto.loop()
            auto.loop() // First real Foresight path calculation.
            assertTrue(powers.filterKeys { it.startsWith("drive") }.values.any { it != 0.0 })
        } finally {
            auto.stop()
        }
        for (power in powers.values) assertEquals(0.0, power, 0.0)
    }
}
