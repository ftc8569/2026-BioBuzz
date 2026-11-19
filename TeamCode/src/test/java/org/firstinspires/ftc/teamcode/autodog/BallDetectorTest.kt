package org.firstinspires.ftc.teamcode.autodog

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.BeforeClass
import org.junit.Test
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc

/**
 * Runs the real detector on drawn frames, using desktop OpenCV (org.openpnp:opencv).
 * Frames are RGBA, which is what VisionPortal hands a processor on the robot.
 */
class BallDetectorTest {
    companion object {
        @BeforeClass
        @JvmStatic
        fun loadOpenCv() = nu.pattern.OpenCV.loadLocally()

        // RGBA
        val GREY = Scalar(110.0, 110.0, 110.0, 255.0)
        val YELLOW = Scalar(240.0, 210.0, 30.0, 255.0)
        val RED = Scalar(210.0, 40.0, 40.0, 255.0)
        val BLUE = Scalar(30.0, 70.0, 200.0, 255.0)
        val GREEN = Scalar(40.0, 190.0, 60.0, 255.0)
    }

    private fun frame(draw: (Mat) -> Unit): Mat =
        Mat(720, 1280, CvType.CV_8UC4, GREY).also(draw)

    private fun ball(img: Mat, u: Int, v: Int, r: Int, color: Scalar) =
        Imgproc.circle(img, Point(u.toDouble(), v.toDouble()), r, color, -1)

    @Test
    fun `finds each ball color at its center`() {
        val img = frame {
            ball(it, 300, 400, 30, YELLOW)
            ball(it, 640, 500, 40, RED)
            ball(it, 1000, 300, 25, BLUE)
        }
        val blobs = BallDetector().detect(img).sortedBy { it.u }
        assertEquals(listOf('Y', 'R', 'B'), blobs.map { it.color })
        val expected = listOf(Triple(300.0, 400.0, 30.0), Triple(640.0, 500.0, 40.0), Triple(1000.0, 300.0, 25.0))
        for ((blob, want) in blobs.zip(expected)) {
            assertEquals(want.first, blob.u, 1.5)
            assertEquals(want.second, blob.v, 1.5)
            assertEquals(want.third, blob.radius, 2.0)
            assertTrue("fill ${blob.fill}", blob.fill > 0.85)
        }
    }

    @Test
    fun `ignores tape strips, other colors and specks`() {
        val img = frame {
            // alliance tape: long red and blue strips
            Imgproc.rectangle(it, Point(100.0, 600.0), Point(900.0, 612.0), RED, -1)
            Imgproc.rectangle(it, Point(50.0, 100.0), Point(60.0, 500.0), BLUE, -1)
            ball(it, 700, 300, 30, GREEN)
            ball(it, 400, 200, 2, YELLOW) // 2 px speck
        }
        assertEquals(emptyList<Char>(), BallDetector().detect(img).map { it.color })
    }

    @Test
    fun `sees a far ball only a few pixels across`() {
        // POLLEN 3 m away is about 9 px across at 1280x720
        val img = frame { ball(it, 640, 200, 5, YELLOW) }
        val blobs = BallDetector().detect(img)
        assertEquals(1, blobs.size)
        assertEquals(640.0, blobs[0].u, 1.5)
        assertEquals(200.0, blobs[0].v, 1.5)
    }

    @Test
    fun `shrunk detection reports full-resolution pixels`() {
        val img = frame { ball(it, 900, 500, 40, BLUE) }
        val saved = AutoDogVision.shrink
        try {
            AutoDogVision.shrink = 2
            val blob = BallDetector().detect(img).single()
            assertEquals(900.0, blob.u, 2.0)
            assertEquals(500.0, blob.v, 2.0)
            assertEquals(40.0, blob.radius, 3.0)
        } finally {
            AutoDogVision.shrink = saved
        }
    }

    @Test
    fun `a three-channel RGB frame works too`() {
        val img = Mat(720, 1280, CvType.CV_8UC3, Scalar(110.0, 110.0, 110.0))
        Imgproc.circle(img, Point(640.0, 360.0), 30, Scalar(210.0, 40.0, 40.0), -1)
        assertEquals(listOf('R'), BallDetector().detect(img).map { it.color })
    }

    /** Camera frame of balls on the floor, drawn through the same camera model the tracker inverts. */
    private fun scene(camera: Projector, balls: List<Triple<Double, Double, Char>>, cfg: TrackerConfig): Mat = frame {
        for ((bx, by, color) in balls) {
            val r = cfg.ballRadii.getValue(color)
            val px = camera.fieldToPixel(bx, by, r)!!
            val edge = camera.fieldToPixel(bx, by, 2 * r)!! // top of the ball
            val radiusPx = Math.round(Math.hypot(px[0] - edge[0], px[1] - edge[1])).toInt()
            val paint = when (color) { 'Y' -> YELLOW; 'R' -> RED; else -> BLUE }
            ball(it, Math.round(px[0]).toInt(), Math.round(px[1]).toInt(), radiusPx, paint)
        }
    }

    @Test
    fun `camera frame to chase target, upright and upside down`() {
        val intrinsics = CameraIntrinsics.GLOBAL_SHUTTER_720P
        val mount = AutoDogTuning.mount()
        val cfg = TrackerConfig.biobuzz()
        val pose = TrackerPose(1.0, 1.2, 0.4)
        val balls = listOf(Triple(1.9, 1.9, 'Y'), Triple(2.6, 1.4, 'R'), Triple(2.2, 2.9, 'B'))
        val camera = Projector(intrinsics, mount, 0.0, 10.0).apply { setPose(pose.x, pose.y, pose.theta) }
        val upright = scene(camera, balls, cfg)
        val flipped = Mat().also { org.opencv.core.Core.rotate(upright, it, org.opencv.core.Core.ROTATE_180) }

        for (upsideDown in listOf(false, true)) {
            val detector = BallDetector()
            val tracker = BallTracker(intrinsics, mount, cfg)
            var t = 3.0
            repeat(4) {
                val blobs = detector.detect(if (upsideDown) flipped else upright)
                val detections = blobs.map {
                    if (upsideDown) {
                        val (u, v) = FrameMath.rotated180(it.u, it.v, 1280, 720)
                        PixelDetection(u, v, it.color)
                    } else {
                        PixelDetection(it.u, it.v, it.color)
                    }
                }
                tracker.tick(t, detections, pose)
                t += 0.033
            }
            val where = if (upsideDown) "upside down" else "upright"
            assertEquals("$where: confirmed tracks", 3, tracker.confirmedTracks().size)
            for ((bx, by, color) in balls) {
                val track = tracker.confirmedTracks().minByOrNull { Math.hypot(it.x[0] - bx, it.x[1] - by) }!!
                assertEquals("$where $color x", bx, track.x[0], 0.04)
                assertEquals("$where $color y", by, track.x[1], 0.04)
                assertEquals(color, tracker.colorOf(track.id))
            }
            val target = tracker.target(t, pose)!!
            assertEquals("$where: nearest ball is the POLLEN", 'Y', target.color)
        }
    }
}
