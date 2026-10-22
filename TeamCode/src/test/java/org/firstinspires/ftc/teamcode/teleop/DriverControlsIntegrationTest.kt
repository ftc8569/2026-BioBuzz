package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.control.ShooterController.ControlMode
import org.junit.Assert.*
import org.junit.Test
import org.mockito.Mockito.*

/** Run actual driver logic, subsystems and Pedro against the fake physical devices. */
class DriverControlsIntegrationTest {
    private class Rig : AutoCloseable {
        val robot = RobotTestRig()
        val powers = DoubleArray(4)
        val g1 = robot.g1
        val g2 = robot.g2
        init {
            robot.motors.take(4).forEachIndexed { i, motor ->
                doAnswer {
                    powers[i] = it.getArgument(0)
                    assertTrue(powers[i].isFinite() && powers[i] in -1.0..1.0)
                    null
                }.`when`(motor).setPower(anyDouble())
            }
        }
        fun pose(x: Double, y: Double, heading: Double = 0.0) {
            `when`(robot.physical.getPosX(DistanceUnit.INCH)).thenReturn(x)
            `when`(robot.physical.getPosY(DistanceUnit.INCH)).thenReturn(y)
            `when`(robot.physical.getHeading(AngleUnit.RADIANS)).thenReturn(heading)
        }
        fun update() {
            robot.displayNow += 50_000_000
            robot.driver.update(g1, g2)
        }
        fun expect(vararg expected: Double) = assertArrayEquals(expected, powers, 1e-7)
        override fun close() = robot.hardware.close()
    }

    @Test fun `joystick supports forward reverse both strafes turns and normalized combinations`() {
        Rig().use { r ->
            val cases = listOf(
                doubleArrayOf(1.0, 0.0, 0.0) to doubleArrayOf(.9, .9, .9, .9),
                doubleArrayOf(-1.0, 0.0, 0.0) to doubleArrayOf(-.9, -.9, -.9, -.9),
                doubleArrayOf(0.0, 1.0, 0.0) to doubleArrayOf(.9, -.9, -.9, .9),
                doubleArrayOf(0.0, -1.0, 0.0) to doubleArrayOf(-.9, .9, .9, -.9),
                doubleArrayOf(0.0, 0.0, 1.0) to doubleArrayOf(.9, -.9, .9, -.9),
                doubleArrayOf(0.0, 0.0, -1.0) to doubleArrayOf(-.9, .9, -.9, .9),
                doubleArrayOf(1.0, 1.0, 0.0) to doubleArrayOf(.9, 0.0, 0.0, .9),
                doubleArrayOf(1.0, 1.0, 1.0) to doubleArrayOf(.9, -.3, .3, .3)
            )
            for ((input, expected) in cases) {
                r.g1.left_stick_y = -input[0].toFloat()
                r.g1.left_stick_x = input[1].toFloat()
                r.g1.right_stick_x = input[2].toFloat()
                r.update()
                r.expect(*expected)
            }
        }
    }

    @Test fun `strafe resumes immediately after manual and LQR shooting hold`() {
        for (manual in listOf(false, true)) Rig().use { r ->
            r.g1.left_stick_x = 1f
            r.update()
            r.expect(.9, -.9, -.9, .9)
            r.g1.y = manual
            r.g1.right_bumper = !manual
            r.update()
            assertTrue(r.robot.hardware.hold.active)
            r.expect(0.0, 0.0, 0.0, 0.0)
            r.g1.y = false
            r.g1.right_bumper = false
            r.update()
            assertFalse(r.robot.hardware.hold.active)
            r.expect(.9, -.9, -.9, .9)
        }
    }

    @Test fun `hive save is one press aim cancels with stick and stops within tolerance`() {
        Rig().use { r ->
            r.pose(0.0, 10.0)
            r.g1.x = true
            r.update()
            r.pose(0.0, 0.0)
            r.g2.y = true // Holding X must not overwrite the saved hive.
            r.update()
            r.expect(-.315, .315, -.315, .315)
            verify(r.robot.ds).addData("Blue Hive Heading (rad)", Math.PI / 2)
            r.g1.right_stick_x = .2f
            r.update()
            r.expect(.18, -.18, .18, -.18)
            r.g1.right_stick_x = 0f
            r.update() // Holding Y must not restart the cancelled aim.
            r.expect(0.0, 0.0, 0.0, 0.0)
            r.g2.y = false
            r.update()
            r.g2.y = true
            r.update()
            r.expect(-.315, .315, -.315, .315)
            r.pose(0.0, 0.0, Math.PI / 2 - .02)
            r.update()
            r.expect(0.0, 0.0, 0.0, 0.0)
        }
    }

    @Test fun `aim without a saved hive leaves strafing under driver control`() {
        Rig().use { r ->
            r.g2.y = true
            r.g1.left_stick_x = -1f
            r.update()
            r.expect(-.9, .9, .9, -.9)
        }
    }

    @Test fun `changed triggers and stop button take effect on the next driver loop`() {
        Rig().use { r ->
            val intake = DoubleArray(2)
            r.robot.motors.drop(6).forEachIndexed { index, motor ->
                doAnswer { intake[index] = it.getArgument(0); null }
                    .`when`(motor).setPower(anyDouble())
            }
            fun expectIntake(power: Double) {
                r.update()
                assertArrayEquals(doubleArrayOf(power, power), intake, 1e-7)
            }
            r.g1.right_trigger = .75f
            r.g1.left_trigger = 1f
            expectIntake(.75)
            r.g1.right_trigger = 0f
            r.g1.left_trigger = .4f
            expectIntake(-.4)
            r.g1.a = true
            expectIntake(0.0)
            r.g1.a = false
            expectIntake(-.4)
            r.g1.left_trigger = .01f
            expectIntake(0.0)
        }
    }

    @Test fun `both gamepads retain held preset edges and manual reverse priority`() {
        Rig().use { r ->
            val control = r.robot.hardware.shooter.control
            r.g1.y = true
            r.g2.dpad_up = true
            r.g2.b = true
            repeat(3) { r.update() }
            assertEquals(ControlMode.MANUAL_POWER, control.controlMode)
            assertEquals(.725, control.normalizedPower, 1e-9)
            assertTrue(control.blockerOpen)
            r.g2.dpad_up = false
            r.update()
            r.g1.dpad_up = true
            r.update()
            assertEquals(.730, control.normalizedPower, 1e-9)
            r.g1.left_bumper = true
            r.g1.left_stick_x = 1f
            r.update()
            assertEquals(-.8, control.normalizedPower, 1e-9)
            assertFalse(r.robot.hardware.hold.active)
            r.expect(.9, -.9, -.9, .9)
            r.g1.y = false
            r.g1.left_bumper = false
            r.g2.b = false
            r.update()
            assertEquals(ControlMode.VELOCITY_LQR, control.controlMode)
            assertEquals(0.0, control.normalizedPower, 0.0)
            assertFalse(control.blockerOpen)
        }
    }
}
