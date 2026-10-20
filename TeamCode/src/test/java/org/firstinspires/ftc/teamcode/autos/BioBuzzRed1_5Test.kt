package org.firstinspires.ftc.teamcode.autos

import com.pedropathing.config.ConfigVar
import com.pedropathing.drivetrain.Drivetrain
import com.pedropathing.follower.Follower
import com.pedropathing.ivy.Scheduler
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.autos.ShootCommand
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem
import org.junit.Assert.*
import org.junit.Test
import org.mockito.ArgumentMatchers.anyDouble
import org.mockito.Mockito.*

class BioBuzzRed1_5Test {
    private class Rig(val auto: BioBuzzRed1_5 = BioBuzzRed1_5()) {
        val follower = mock(Follower::class.java)
        val map = mock(HardwareMap::class.java)
        val shotsDone = mutableSetOf<Int>()
        val paths = mutableListOf<Path>()
        var following = false
        var intakePower = 0.0
        var startPose: Pose? = null
        init {
            for (name in listOf("driveFL", "driveFR", "driveBL", "driveBR", "shooter1", "shooter2", "intake1", "intake2"))
                `when`(map.get(DcMotorEx::class.java, name)).thenReturn(mock(DcMotorEx::class.java))
            `when`(map.get(Servo::class.java, "blocker")).thenReturn(mock(Servo::class.java))
            Follower::class.java.getField("drivetrain").apply { isAccessible = true }
                .set(follower, mock(Drivetrain::class.java))
            Follower::class.java.getField("holdEnd").apply { isAccessible = true }
                .set(follower, ConfigVar.of(true))
            `when`(follower.following()).thenAnswer { following }
            `when`(follower.pose()).thenAnswer { startPose }
            doAnswer { startPose = it.getArgument(0); null }.`when`(follower).setPose(any(Pose::class.java))
            doAnswer { paths += it.getArgument<Path>(0); following = true; null }
                .`when`(follower).follow(any(Path::class.java))
            auto.hardwareMap = map
            auto.telemetry = mock(Telemetry::class.java)
        }
        fun tick() = auto.loop()
        fun until(condition: () -> Boolean) {
            repeat(12) { if (condition()) return; tick() }
            assertTrue("Auto did not reach the expected step", condition())
        }
    }

    private fun withAuto(auto: BioBuzzRed1_5 = BioBuzzRed1_5(), test: (Rig) -> Unit) {
        val rig = Rig(auto)
        mockConstruction(ShooterSubsystem::class.java).use {
            mockConstruction(IntakeSubsystem::class.java) { intake, _ ->
                doAnswer { rig.intakePower = it.getArgument(0); null }
                    .`when`(intake).setPower(anyDouble())
                doAnswer { rig.intakePower = 0.0; null }.`when`(intake).forceStop()
            }.use {
                mockConstruction(ShootCommand::class.java) { command, context ->
                    `when`(command.requirements()).thenReturn(emptySet())
                    `when`(command.done()).thenAnswer { context.count in rig.shotsDone }
                }.use {
                    mockStatic(Constants::class.java).use { factory ->
                        factory.`when`<Follower> { Constants.create(rig.map) }.thenReturn(rig.follower)
                        try { rig.auto.init(); test(rig) }
                        finally { rig.auto.stop(); Scheduler.reset() }
                    }
                }
            }
        }
    }

    @Test fun `shoot then collect on path one drive path two shoot then drive path three`() = withAuto { rig ->
        assertEquals(58.4934, rig.startPose!!.x(), 1e-9)
        assertEquals(9.0389, rig.startPose!!.y(), 1e-9)
        rig.auto.start()
        repeat(3) { rig.tick() }
        assertTrue(rig.paths.isEmpty())
        rig.shotsDone += 1
        rig.until { rig.paths.size == 1 }
        assertEquals(1.0, rig.intakePower, 0.0)
        // Reaching the curve's parametric end alone must not advance the routine.
        `when`(rig.follower.atParametricEnd()).thenReturn(true)
        repeat(3) { rig.tick() }
        assertEquals(1, rig.paths.size)
        rig.following = false
        rig.until { rig.paths.size == 2 }
        assertEquals(0.0, rig.intakePower, 0.0)
        rig.following = false
        repeat(4) { rig.tick() }
        assertEquals(2, rig.paths.size)
        assertEquals(0.0, rig.intakePower, 0.0)
        rig.shotsDone += 2
        rig.until { rig.paths.size == 3 }
        assertEquals(0.0, rig.intakePower, 0.0)

        val (one, two, three) = rig.paths
        assertEquals(10.7034, one.get(1.0).x(), 1e-9)
        assertEquals(8.5727, one.get(1.0).y(), 1e-9)
        assertEquals(34.50185, one.get(0.5).x(), 1e-9)
        assertEquals(10.8717, one.get(0.5).y(), 1e-9)
        assertAngle(-90.0, one.heading(0.0))
        assertAngle(-165.0, one.heading(1.0))
        assertEquals(33.92818671875, two.get(0.25).x(), 1e-9)
        assertEquals(58.101080859375, two.get(0.25).y(), 1e-9)
        assertEquals(36.2916875, two.get(0.5).x(), 1e-9)
        assertEquals(100.91414375, two.get(0.5).y(), 1e-9)
        assertAngle(-115.0, two.heading(0.0))
        assertAngle(90.0, two.heading(1.0))
        assertEquals(46.6136, three.get(0.5).x(), 1e-9)
        assertEquals(121.822525, three.get(0.5).y(), 1e-9)
        assertEquals(12.3128, three.get(1.0).x(), 1e-9)
        assertEquals(115.8693, three.get(1.0).y(), 1e-9)
        // Reverse tangent faces opposite the initial curve derivative.
        assertAngle(Math.toDegrees(atan2(119.3084 - 132.804, 57.986 - 58.1696)) + 180,
            three.heading(0.0))
        rig.following = false
        repeat(4) { rig.tick() }
        assertEquals(3, rig.paths.size)
        assertEquals(0.0, rig.intakePower, 0.0)
    }

    @Test fun `cancel during collection stops intake and follower`() = withAuto { rig ->
        rig.auto.start()
        rig.shotsDone += 1
        rig.until { rig.paths.size == 1 }
        assertEquals(1.0, rig.intakePower, 0.0)
        rig.auto.stop()
        assertEquals(0.0, rig.intakePower, 0.0)
        verify(rig.follower, atLeastOnce()).stop()
    }

    @Test fun `blue rotates all three red curves and headings while preserving the sequence`() {
        var redStart: Pose? = null
        var redPaths: List<Path> = emptyList()
        withAuto { rig ->
            redStart = rig.startPose
            advanceToThirdPath(rig)
            redPaths = rig.paths.toList()
        }
        withAuto(BioBuzzAutoBlue()) { rig ->
            assertEquals(83.0066, rig.startPose!!.x(), 1e-9)
            assertEquals(132.4611, rig.startPose!!.y(), 1e-9)
            assertAngle(90.0, rig.startPose!!.heading())
            assertEquals(141.5 - redStart!!.x(), rig.startPose!!.x(), 1e-9)
            advanceToThirdPath(rig)
            for ((red, blue) in redPaths.zip(rig.paths)) {
                for (t in listOf(0.0, 0.25, 0.5, 0.75, 1.0)) {
                    assertEquals(141.5 - red.get(t).x(), blue.get(t).x(), 1e-9)
                    assertEquals(141.5 - red.get(t).y(), blue.get(t).y(), 1e-9)
                    assertAngle(Math.toDegrees(red.heading(t)) + 180.0, blue.heading(t))
                }
            }
        }
    }

    private fun advanceToThirdPath(rig: Rig) {
        rig.auto.start()
        repeat(3) { rig.tick() }
        assertTrue(rig.paths.isEmpty())
        rig.shotsDone += 1
        rig.until { rig.paths.size == 1 }
        assertEquals(1.0, rig.intakePower, 0.0)
        rig.following = false
        rig.until { rig.paths.size == 2 }
        assertEquals(0.0, rig.intakePower, 0.0)
        rig.following = false
        repeat(4) { rig.tick() }
        assertEquals(2, rig.paths.size)
        rig.shotsDone += 2
        rig.until { rig.paths.size == 3 }
        assertEquals(0.0, rig.intakePower, 0.0)
    }

    private fun assertAngle(degrees: Double, actual: Double) {
        val difference = actual - Math.toRadians(degrees)
        assertEquals(0.0, atan2(sin(difference), cos(difference)), 1e-9)
    }
}
