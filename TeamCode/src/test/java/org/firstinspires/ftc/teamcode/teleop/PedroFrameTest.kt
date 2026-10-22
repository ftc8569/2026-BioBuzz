package org.firstinspires.ftc.teamcode.teleop

import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.localization.MotionState
import com.pedropathing.math.Pose
import com.pedropathing.math.Velocity
import com.pedropathing.tuning.autotune.TunerRegistrar
import com.pedropathing.tuning.autotune.TunerScanner
import org.firstinspires.ftc.teamcode.subsystems.BufferedMecanum
import org.firstinspires.ftc.teamcode.subsystems.SnapshotLocalizer
import org.firstinspires.ftc.teamcode.pedro.Tuning
import org.junit.Assert.*
import org.junit.Test

class PedroFrameTest {
    @Test fun `installed tuner scanner discovers every configured procedure`() {
        TunerRegistrar.deregisterAll()
        try {
            // Discovery constructs procedures; it never executes their hardware routines.
            val loader = checkNotNull(Tuning::class.java.classLoader)
            TunerScanner.INSTANCE.scan(loader, Tuning::class.java)
            val procedures = TunerRegistrar.getProcedures()
            assertEquals(4, procedures.size)
            assertEquals(setOf("Mecanum Tuner", "Pinpoint Tuner", "Foresight Tuner", "Tests"),
                procedures.map { it.name }.toSet())
            assertTrue(procedures.all { it.procedure.description.isNotBlank() })
        } finally {
            TunerRegistrar.deregisterAll()
        }
    }

    @Test fun `snapshot localizer keeps captured pose and velocity through repeated Pedro updates`() {
        val localizer = SnapshotLocalizer()
        val state = MotionState.ofVelocity(Pose(4.0, 8.0, 0.5), Velocity(1.0, 2.0, 0.1))
        localizer.capture(state)
        repeat(4) { localizer.update() }
        assertSame(state, localizer.state())
        assertEquals(2.0, localizer.velocity().vy, 0.0)
    }
    @Test fun `Pedro drive preserves its mixing and normalization without writing hardware`() {
        var outputs = DoubleArray(4)
        val drive = BufferedMecanum { powers, _ -> outputs = powers }
        drive.drive(DrivePowers(0.5, 0.25, 0.1), false)
        assertArrayEquals(doubleArrayOf(0.15, 0.85, 0.65, 0.35), outputs, 1e-9)
        drive.drive(DrivePowers(1.0, 1.0, 1.0), false)
        assertArrayEquals(doubleArrayOf(-1.0/3, 1.0, 1.0/3, 1.0/3), outputs, 1e-9)
        drive.stop()
        assertArrayEquals(DoubleArray(4), outputs, 0.0)
    }

    @Test fun `scaling respects both wheel limits without replacing commanded outputs`() {
        var outputs = DoubleArray(4)
        val drive = BufferedMecanum { powers, _ -> outputs = powers }
        drive.drive(DrivePowers(.5, .25, .1), false)
        val cases = listOf(
            Triple(DrivePowers(.25, 0.0, 0.0), DrivePowers(1.0, 0.0, 0.0), .75),
            Triple(DrivePowers(-.25, 0.0, 0.0), DrivePowers(-1.0, 0.0, 0.0), .75),
            Triple(DrivePowers(0.0, .5, 0.0), DrivePowers(0.0, .75, 0.0), 2.0 / 3),
            Triple(DrivePowers(0.0, 0.0, -.75), DrivePowers(0.0, 0.0, -.5), .5),
            Triple(DrivePowers(0.0, 0.0, 0.0), DrivePowers(1.0, 1.0, 1.0), 1.0 / 3),
            Triple(DrivePowers(.25, 0.0, 0.0), DrivePowers(0.0, 0.0, 0.0), 1.0)
        )
        for ((current, delta, expected) in cases) {
            assertEquals(expected, drive.maxScaling(current, delta), 1e-9)
            assertArrayEquals(doubleArrayOf(.15, .85, .65, .35), outputs, 1e-9)
        }
    }
}
