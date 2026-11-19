package org.firstinspires.ftc.teamcode.autodog

import org.firstinspires.ftc.teamcode.vision.Mat3
import org.firstinspires.ftc.teamcode.vision.Transform3
import org.firstinspires.ftc.teamcode.vision.Vec3
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.abs

class BallVisionTest {

    // Same convention as HiveCameraMount: camera right/down/out -> robot forward/left/up.
    private val levelForwardCamera = Mat3.rows(
        0.0, 0.0, 1.0,
        -1.0, 0.0, 0.0,
        0.0, -1.0, 0.0,
    )

    /** Camera 0.30 m up, 0.10 m ahead of center, pitched 20 degrees down. */
    private fun mount(pitchDownDeg: Double = 20.0) = Transform3(
        Vec3(0.10, 0.0, 0.30),
        Mat3.rotY(Math.toRadians(pitchDownDeg)) * levelForwardCamera,
    )

    private val ballR = 1.5 * 0.0254
    private val w = 640.0
    private val h = 480.0

    /** Forward model: where a ball center (robot frame) lands in the image. */
    private fun pixelOf(p: BallProjector, cam: Transform3, rx: Double, ry: Double): PixelBall {
        val c = cam.r.transpose() * (Vec3(rx, ry, ballR) - cam.p)
        val fx = p.focalX(w)
        val fy = p.focalY(h)
        val range = c.norm()
        return PixelBall('P', w / 2 + fx * c.x / c.z, h / 2 + fy * c.y / c.z, fx * ballR / range)
    }

    @Test
    fun `parses the SnapScript output`() {
        val f = LimelightBallFrame.parse(doubleArrayOf(1.0, 640.0, 480.0, 2.0, 1.0, 10.0, 20.0, 5.0, 2.0, 30.0, 40.0, 6.0))
        assertNotNull(f)
        assertEquals(2, f!!.balls.size)
        assertEquals(PixelBall('P', 10.0, 20.0, 5.0), f.balls[0])
        assertEquals('N', f.balls[1].color)
    }

    @Test
    fun `rejects other pipelines and truncated output`() {
        assertNull(LimelightBallFrame.parse(null))
        assertNull(LimelightBallFrame.parse(DoubleArray(8)))                                     // python pipeline never ran
        assertNull(LimelightBallFrame.parse(doubleArrayOf(1.0, 640.0, 480.0, 2.0, 1.0, 1.0, 1.0, 1.0))) // says 2, has 1
        assertEquals(0, LimelightBallFrame.parse(doubleArrayOf(1.0, 640.0, 480.0, 0.0))!!.balls.size)
    }

    @Test
    fun `projects pixels back onto the floor`() {
        val cam = mount()
        val p = BallProjector(robotToCamera = cam, ballRadiusM = ballR)
        for ((x, y) in listOf(0.8 to 0.0, 1.2 to 0.3, 0.6 to -0.25, 2.0 to 0.5)) {
            val hit = p.project(pixelOf(p, cam, x, y), w, h)
            assertNotNull("ball at $x, $y", hit)
            assertEquals(x, hit!!.first, 1e-9)
            assertEquals(y, hit.second, 1e-9)
        }
    }

    @Test
    fun `rejects rays above the horizon, out of range, or the wrong size`() {
        val cam = mount(pitchDownDeg = 0.0)
        val p = BallProjector(robotToCamera = cam, ballRadiusM = ballR, maxRangeM = 3.0)
        assertNull(p.project(PixelBall('P', 320.0, 100.0, 10.0), w, h)) // above center on a level camera
        val far = pixelOf(p, cam, 5.0, 0.0)
        assertNull(p.project(far, w, h))
        val near = pixelOf(p, cam, 1.0, 0.0)
        assertNotNull(p.project(near, w, h))
        assertNull(p.project(near.copy(radiusPx = near.radiusPx * 3), w, h)) // too big: ball in the air or a robot
        p.radiusTolerance = 0.0
        assertNotNull(p.project(near.copy(radiusPx = near.radiusPx * 3), w, h))
    }

    @Test
    fun `pose history interpolates across the heading wrap`() {
        val hist = PoseHistory(capacity = 4)
        assertNull(hist.at(0.0))
        hist.add(1.0, PlanarPose(0.0, 0.0, PI - 0.1))
        hist.add(2.0, PlanarPose(1.0, 2.0, -PI + 0.1))
        val mid = hist.at(1.5)!!
        assertEquals(0.5, mid.x, 1e-12)
        assertEquals(1.0, mid.y, 1e-12)
        assertEquals(PI, abs(mid.theta), 1e-9) // through 180, not through 0
        assertEquals(1.0, hist.at(9.0)!!.x, 0.0) // future -> newest
        assertNull(hist.at(0.5))                 // older than history
        for (t in 3..6) hist.add(t.toDouble(), PlanarPose(t.toDouble(), 0.0, 0.0))
        assertNull(hist.at(2.5))                 // overwritten by the ring buffer
        assertEquals(4.5, hist.at(4.5)!!.x, 1e-12)
    }

    @Test
    fun `tracker keeps an id and learns rolling speed`() {
        val tr = BallTracker()
        var t = 0.0
        repeat(30) { i ->
            tr.update(t, listOf(FieldBall('P', 1.0 + 0.5 * t, 2.0), FieldBall('N', 3.0, 1.0)))
            t += 0.03
        }
        assertEquals(2, tr.tracks.size)
        val p = tr.tracks.first { it.color == 'P' }
        assertEquals(1, p.id)
        assertEquals(0.5, p.vx, 0.05)
        assertEquals(0.0, p.vy, 0.05)
        assertEquals(0.0, tr.tracks.first { it.color == 'N' }.vx, 1e-9)
    }

    @Test
    fun `tracker needs two sightings and forgets lost balls`() {
        val tr = BallTracker(dropAfterS = 0.5)
        tr.update(0.0, listOf(FieldBall('P', 1.0, 1.0)))
        assertTrue(tr.confirmed().isEmpty())     // one frame could be glare
        tr.update(0.03, listOf(FieldBall('P', 1.01, 1.0)))
        assertEquals(1, tr.confirmed().size)
        tr.update(0.4, emptyList())
        assertEquals(1, tr.tracks.size)
        tr.update(0.6, emptyList())
        assertTrue(tr.tracks.isEmpty())
    }

    @Test
    fun `different colors never merge`() {
        val tr = BallTracker()
        tr.update(0.0, listOf(FieldBall('P', 1.0, 1.0)))
        tr.update(0.03, listOf(FieldBall('N', 1.0, 1.0)))
        assertEquals(2, tr.tracks.size)
    }

    @Test
    fun `selector sticks with its ball until another is clearly closer`() {
        val a = BallTrack(1, 'P', 1.0, 0.0, 0.0)
        val b = BallTrack(2, 'N', 1.2, 0.0, 0.0)
        val sel = TargetSelector(switchMarginM = 0.3)
        val robot = PlanarPose(0.0, 0.0, 0.0)
        assertEquals(1, sel.select(listOf(a, b), robot, 0.2, null, 0.0)!!.id)
        b.x = 0.9 // a bit closer now, inside the margin: keep a
        assertEquals(1, sel.select(listOf(a, b), robot, 0.2, null, 0.0)!!.id)
        b.x = 0.5 // clearly closer: switch
        assertEquals(2, sel.select(listOf(a, b), robot, 0.2, null, 0.0)!!.id)
        assertEquals(1, sel.select(listOf(a, b), robot, 0.2, 'P', 0.0)!!.id) // color filter
        assertNull(sel.select(emptyList(), robot, 0.2, null, 0.0))
    }

    @Test
    fun `onboard tracker places balls with the pose from capture time`() {
        val cam = mount()
        val proj = BallProjector(robotToCamera = cam, ballRadiusM = ballR)
        val vision = OnboardBallTracker(proj)
        // Robot at (1, 1) facing +y when the frame was taken; ball 1 m ahead -> field (1, 2).
        val pixel = pixelOf(proj, cam, 1.0, 0.0)
        vision.addPose(0.00, PlanarPose(1.0, 1.0, PI / 2))
        // By the time the frame reaches the hub the robot has turned 0.5 rad.
        vision.addPose(0.05, PlanarPose(1.0, 1.0, PI / 2 + 0.5))
        assertTrue(vision.onFrame(1.0, 0.0, BallFrame(w, h, listOf(pixel))))
        val track = vision.tracker.tracks.single()
        assertEquals(1.0, track.x, 1e-9)
        assertEquals(2.0, track.y, 1e-9)
        assertTrue(!vision.onFrame(1.0, 0.0, BallFrame(w, h, listOf(pixel)))) // same frame twice is ignored
        assertNull(vision.target(0.05, PlanarPose(1.0, 1.0, PI / 2), 0.2, null)) // one sighting is not enough
        // A frame captured before the pose history starts can't be placed.
        assertTrue(!vision.onFrame(2.0, -1.0, BallFrame(w, h, listOf(pixel))))
    }

    @Test
    fun `onboard tracker locks onto a still ball`() {
        val cam = mount()
        val proj = BallProjector(robotToCamera = cam, ballRadiusM = ballR)
        val vision = OnboardBallTracker(proj)
        val robot = PlanarPose(1.0, 1.0, PI / 2)
        val pixel = pixelOf(proj, cam, 1.0, 0.0)
        for (i in 0..5) {
            val time = i * 0.03
            vision.addPose(time, robot)
            vision.onFrame(i.toDouble(), time, BallFrame(w, h, listOf(pixel)))
        }
        val t = vision.target(0.15, robot, 0.2, null)!!
        assertEquals(1.0, t.x, 1e-6)
        assertEquals(2.0, t.y, 1e-6)
        assertEquals(0.0, t.age(0.15), 1e-12)
    }

    @Test
    fun `field gate drops balls off the field`() {
        val cam = mount()
        val proj = BallProjector(robotToCamera = cam, ballRadiusM = ballR)
        val vision = OnboardBallTracker(proj)
        val robot = PlanarPose(0.2, 0.2, PI) // at a corner, facing out of the field
        vision.addPose(0.0, robot)
        vision.onFrame(1.0, 0.0, BallFrame(w, h, listOf(pixelOf(proj, cam, 1.0, 0.0))))
        assertEquals(0, vision.lastDetections)
        vision.fieldGate = false
        vision.onFrame(2.0, 0.0, BallFrame(w, h, listOf(pixelOf(proj, cam, 1.0, 0.0))))
        assertEquals(1, vision.lastDetections)
    }

    @Test
    fun `chase mode truth table`() {
        assertEquals(ChaseMode.CHASE, chooseChaseMode(aHeld = true, bHeld = false, freshTarget = true))
        assertEquals(ChaseMode.CHASE, chooseChaseMode(aHeld = true, bHeld = true, freshTarget = true))
        assertEquals(ChaseMode.SEARCH, chooseChaseMode(aHeld = true, bHeld = true, freshTarget = false))
        assertEquals(ChaseMode.HOLD, chooseChaseMode(aHeld = true, bHeld = false, freshTarget = false))
        assertEquals(ChaseMode.MANUAL, chooseChaseMode(aHeld = false, bHeld = true, freshTarget = true))
    }

    @Test
    fun `only CHASE drives toward the ball or runs the intake`() {
        val c = ChaseController()
        val robot = PlanarPose(0.0, 0.0, 0.0)
        val target = AutoDogTarget(0.0, true, 1, 1.0, 0.0, 0.0, 0.0, 'P', 0.0)
        val chase = c.command(ChaseMode.CHASE, target, 0.0, robot, 1.0, 0.25)
        assertTrue(chase.power.forward > 0)
        assertEquals(1.0, chase.intakePower, 0.0)
        assertEquals(ChaseCommand(DrivePower(0.0, 0.0, 0.25), 0.0), c.command(ChaseMode.SEARCH, null, 0.0, robot, 1.0, 0.25))
        assertEquals(ChaseCommand(DrivePower.STOP, 0.0), c.command(ChaseMode.HOLD, null, 0.0, robot, 1.0, 0.25))
    }
}
