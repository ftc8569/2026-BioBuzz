package org.firstinspires.ftc.teamcode.teleop

import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.localization.MotionState
import com.pedropathing.math.Pose
import com.pedropathing.math.Velocity
import org.firstinspires.ftc.teamcode.hardware.BufferedMecanum
import org.firstinspires.ftc.teamcode.hardware.SnapshotLocalizer
import org.junit.Assert.*
import org.junit.Test

class PedroFrameTest {
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
}
