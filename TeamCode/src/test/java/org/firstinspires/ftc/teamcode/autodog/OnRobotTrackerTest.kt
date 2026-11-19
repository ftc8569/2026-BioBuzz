package org.firstinspires.ftc.teamcode.autodog

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.Subsytem.DriveSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.IntakeSubsystem
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import org.mockito.Mockito.mock
import org.mockito.Mockito.verify
import kotlin.math.PI
import kotlin.math.hypot

/** The pieces of the on-robot tracker that the golden replay does not reach directly. */
class OnRobotTrackerTest {
    private val intrinsics = CameraIntrinsics.GLOBAL_SHUTTER_720P
    private val mount = CameraMount(x = 0.15, y = 0.02, z = 0.28, pitchRad = Math.toRadians(28.0), yawRad = 0.05)

    @Test
    fun `projecting a ball's pixel returns the ball's field position`() {
        val r = 1.8 * FieldFrame.INCH
        val projector = Projector(intrinsics, mount, r, 4.0)
        projector.setPose(1.0, 2.0, 0.7)
        for ((bx, by) in listOf(1.8 to 2.6, 2.4 to 3.1, 1.5 to 3.4, 2.9 to 2.4)) {
            val px = projector.fieldToPixel(bx, by, r)
            assertNotNull("ball at $bx,$by should be in front of the camera", px)
            val back = projector.project(px!![0], px[1], 1.5)
            assertNotNull(back)
            assertEquals(bx, back!!.x, 2e-3)
            assertEquals(by, back.y, 2e-3)
        }
    }

    @Test
    fun `a ball straight ahead on the floor lands below the image center`() {
        // Camera pitched down 28 degrees at 0.28 m: the optical axis meets the floor about
        // 0.53 m ahead, so a ball 1.5 m ahead is above center and one 0.3 m ahead below.
        val level = CameraMount(x = 0.0, y = 0.0, z = 0.28, pitchRad = Math.toRadians(28.0))
        val projector = Projector(intrinsics, level, 0.0, 4.0)
        projector.setPose(0.0, 0.0, 0.0)
        val far = projector.fieldToPixel(1.5, 0.0, 0.0)!!
        val near = projector.fieldToPixel(0.3, 0.0, 0.0)!!
        val left = projector.fieldToPixel(1.5, 0.5, 0.0)!!
        assertTrue(far[1] < intrinsics.cy)
        assertTrue(near[1] > intrinsics.cy)
        assertTrue("a ball to the robot's left is left in the image", left[0] < far[0])
    }

    @Test
    fun `rays above the horizon and beyond max range are rejected`() {
        val projector = Projector(intrinsics, mount, 0.0356, 4.0)
        projector.setPose(1.0, 1.0, 0.0)
        assertNull("top of the image looks above the horizon", projector.project(640.0, 5.0, 1.5))
        val far = projector.fieldToPixel(1.0 + 6.0, 1.0, 0.0356)!!
        assertNull(projector.project(far[0], far[1], 1.5))
    }

    @Test
    fun `far balls are less certain than near ones`() {
        val projector = Projector(intrinsics, mount, 0.0356, 4.0)
        projector.setPose(0.0, 0.0, 0.0)
        val near = projector.fieldToPixel(0.8, 0.0, 0.0356)!!.let { projector.project(it[0], it[1], 1.5)!! }
        val far = projector.fieldToPixel(3.0, 0.0, 0.0356)!!.let { projector.project(it[0], it[1], 1.5)!! }
        assertTrue(far.cov[0] > 10 * near.cov[0])
    }

    @Test
    fun `intrinsics scale with resolution`() {
        val half = intrinsics.scaled(640, 360)
        assertEquals(intrinsics.fx / 2, half.fx, 1e-9)
        assertEquals(intrinsics.cy / 2, half.cy, 1e-9)
        assertEquals(640, half.width)
    }

    @Test
    fun `pose buffer interpolates, extrapolates briefly and refuses stale times`() {
        val poses = PoseBuffer()
        assertNull(poses.at(1.0))
        poses.add(1.00, 0.0, 0.0, PI - 0.1)
        poses.add(1.02, 0.2, 0.0, -PI + 0.1) // heading crosses +/- pi
        val mid = poses.at(1.01)!!
        assertEquals(0.1, mid.x, 1e-9)
        assertEquals(PI, kotlin.math.abs(mid.theta), 1e-9)
        val ahead = poses.at(1.03)!!
        assertEquals(0.3, ahead.x, 1e-9)
        assertNull("more than 50 ms past the newest pose", poses.at(1.08))
        assertNull("older than the oldest pose", poses.at(0.9))
        poses.add(1.01, 9.0, 9.0, 0.0) // out of order: dropped
        assertEquals(2, poses.size)
    }

    @Test
    fun `tracker confirms a still ball and reports it as the target in field coordinates`() {
        val cfg = TrackerConfig.biobuzz()
        val tracker = BallTracker(intrinsics, mount, cfg)
        val r = cfg.ballRadii.getValue(TrackerConfig.RED_NECTAR)
        val camera = Projector(intrinsics, mount, r, 10.0)
        camera.setPose(1.0, 1.0, 0.3)
        val px = camera.fieldToPixel(2.0, 1.6, r)!!
        var t = 10.0
        repeat(5) {
            tracker.poses.add(t - 0.005, 1.0, 1.0, 0.3)
            tracker.poses.add(t + 0.005, 1.0, 1.0, 0.3)
            tracker.tick(t, listOf(PixelDetection(px[0], px[1], TrackerConfig.RED_NECTAR)))
            t += 0.033
        }
        val target = tracker.target(t, TrackerPose(1.0, 1.0, 0.3))
        assertNotNull(target)
        assertEquals(2.0, target!!.x, 5e-3)
        assertEquals(1.6, target.y, 5e-3)
        assertEquals(TrackerConfig.RED_NECTAR, target.color)
        assertEquals(hypot(1.0, 0.6), target.distance, 5e-3)
    }

    @Test
    fun `a frame with no pose coasts and a ball seen once is not a target`() {
        val tracker = BallTracker(intrinsics, mount)
        val result = tracker.tick(5.0, listOf(PixelDetection(640.0, 500.0, TrackerConfig.POLLEN)))
        assertNull(result.pose)
        assertEquals(0, tracker.tracks.size)

        tracker.poses.add(5.0, 1.0, 1.0, 0.0)
        tracker.tick(5.0, listOf(PixelDetection(640.0, 500.0, TrackerConfig.POLLEN)))
        assertEquals(1, tracker.tracks.size)
        assertNull("one sighting is not confirmed", tracker.target(5.0))
    }

    @Test
    fun `field gate can be turned off for an off-field floor`() {
        val px = Projector(intrinsics, mount, 0.0356, 4.0).apply { setPose(0.2, 0.2, PI) }
            .fieldToPixel(-1.0, 0.2, 0.0356)!! // outside the field
        fun tracksWith(gate: Boolean): Int {
            val tracker = BallTracker(intrinsics, mount, TrackerConfig.biobuzz().apply { gateField = gate })
            tracker.tick(1.0, listOf(PixelDetection(px[0], px[1], TrackerConfig.POLLEN)), TrackerPose(0.2, 0.2, PI))
            return tracker.tracks.size
        }
        assertEquals(0, tracksWith(true))
        assertEquals(1, tracksWith(false))
    }

    @Test
    fun `frame time uses the camera stamp when it is plausible`() {
        val now = 5_000_000_000L
        val good = FrameMath.frameTime(now, now - 40_000_000L, 0.0)
        assertTrue(good.stampUsed)
        assertEquals(4.96, good.t, 1e-9)

        for (bad in listOf(0L, now + 1_000_000L, now - 2_000_000_000L)) {
            val f = FrameMath.frameTime(now, bad, 0.0)
            assertFalse(f.stampUsed)
            assertEquals(5.0, f.t, 1e-9)
        }
        assertEquals(4.93, FrameMath.frameTime(now, now - 40_000_000L, 0.03).t, 1e-9)
    }

    @Test
    fun `pixel bookkeeping for shrunk and upside-down images`() {
        assertEquals(10.0, FrameMath.fullResolution(10.0, 1), 1e-12)
        // pixel 0 of a half-size image covers full-size pixels 0 and 1: center 0.5
        assertEquals(0.5, FrameMath.fullResolution(0.0, 2), 1e-12)
        assertEquals((1279.0 to 719.0), FrameMath.rotated180(0.0, 0.0, 1280, 720))
        assertEquals((0.0 to 0.0), FrameMath.rotated180(1279.0, 719.0, 1280, 720))
    }

    @Test
    fun `tuning builds the mount and intrinsics the tracker uses`() {
        val m = AutoDogTuning.mount()
        assertEquals(AutoDogTuning.camHeightM, m.z, 1e-12)
        assertEquals(Math.toRadians(AutoDogTuning.camPitchDownDeg), m.pitchRad, 1e-12)
        assertEquals(1280, AutoDogTuning.intrinsics().width)
    }

    private fun gamepad(a: Boolean, b: Boolean = false) = Gamepad().apply {
        this.a = a
        this.b = b
        left_stick_y = -0.5f
    }

    @Test
    fun `holding A with a target chases with the intake on`() {
        val drive = mock(DriveSubsystem::class.java)
        val intake = mock(IntakeSubsystem::class.java)
        val driver = ChaseDriver(drive, intake)
        driver.step(gamepad(a = true), 1.0, 1.0, 0.0, 2.0 to 1.0)
        assertEquals("CHASE", driver.mode)
        assertTrue(driver.power.forward > 0)
        verify(intake).setPower(AutoDogTuning.intakePower)
        verify(drive).drive(driver.power.forward, -driver.power.left, -driver.power.turn)
    }

    @Test
    fun `holding A with no target stops, and A plus B searches`() {
        val drive = mock(DriveSubsystem::class.java)
        val intake = mock(IntakeSubsystem::class.java)
        val driver = ChaseDriver(drive, intake)
        driver.step(gamepad(a = true), 1.0, 1.0, 0.0, null)
        assertEquals("HOLD (no target)", driver.mode)
        assertEquals(DrivePower.STOP, driver.power)
        verify(drive).drive(0.0, -0.0, -0.0)

        driver.step(gamepad(a = true, b = true), 1.0, 1.0, 0.0, null)
        assertEquals("SEARCH", driver.mode)
        assertEquals(AutoDogTuning.searchTurn, driver.power.turn, 1e-12)
    }

    @Test
    fun `releasing A hands the sticks back even when a target exists`() {
        val drive = mock(DriveSubsystem::class.java)
        val intake = mock(IntakeSubsystem::class.java)
        val driver = ChaseDriver(drive, intake)
        driver.step(gamepad(a = false), 1.0, 1.0, 0.0, 2.0 to 1.0)
        assertEquals("manual", driver.mode)
        verify(drive).drive(0.5, 0.0, 0.0)
        verify(intake).setPower(0.0)
    }
}
