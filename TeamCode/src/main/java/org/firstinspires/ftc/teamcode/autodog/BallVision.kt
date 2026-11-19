package org.firstinspires.ftc.teamcode.autodog

import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc

/** A round colored blob in the camera image. u, v and radius are full-resolution pixels as the camera delivers them. */
class Blob(val u: Double, val v: Double, val radius: Double, val color: Char, val fill: Double, val area: Double)

/**
 * Finds POLLEN (yellow) and NECTAR (red, blue) as round HSV blobs. Port of
 * BioBuzzAutoDog/python/autodog/detect.py. Thresholds come from AutoDogVision, so
 * they can be tuned live from Panels.
 *
 * Field tape and walls are also red and blue, so a blob must be round: its area
 * against its minimum enclosing circle rejects long strips.
 */
class BallDetector {
    private val small = Mat()
    private val hsv = Mat()
    private val mask = Mat()
    private val mask2 = Mat()
    private val hierarchy = Mat()
    private var kernel: Mat? = null
    private var kernelPx = -1

    /** frame is the RGBA (or RGB) image a VisionProcessor receives. */
    fun detect(frame: Mat, v: AutoDogVision = AutoDogVision): List<Blob> {
        val shrink = v.shrink.coerceIn(1, 4)
        val src = if (shrink == 1) frame else small.also {
            Imgproc.resize(frame, it, Size(), 1.0 / shrink, 1.0 / shrink, Imgproc.INTER_AREA)
        }
        Imgproc.cvtColor(src, hsv, Imgproc.COLOR_RGB2HSV)
        if (v.openPx != kernelPx) {
            kernel?.release()
            kernel = if (v.openPx > 0) {
                Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, Size(v.openPx.toDouble(), v.openPx.toDouble()))
            } else {
                null
            }
            kernelPx = v.openPx
        }
        // Area limits are given for the full image; a shrunk image has shrink^2 fewer pixels.
        val minArea = v.minAreaPx / (shrink * shrink)
        val maxArea = v.maxAreaFraction * src.rows() * src.cols()
        val found = ArrayList<Blob>()

        fun range(hLo: Int, hHi: Int, sMin: Int, vMin: Int, out: Mat) = Core.inRange(
            hsv, Scalar(hLo.toDouble(), sMin.toDouble(), vMin.toDouble()), Scalar(hHi.toDouble(), 255.0, 255.0), out)

        fun collect(color: Char) {
            kernel?.let { Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, it) }
            val contours = ArrayList<MatOfPoint>()
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
            val center = Point()
            val radius = FloatArray(1)
            for (contour in contours) {
                val area = Imgproc.contourArea(contour)
                if (area > minArea && area < maxArea) {
                    val points = MatOfPoint2f(*contour.toArray())
                    Imgproc.minEnclosingCircle(points, center, radius)
                    points.release()
                    val r = radius[0].toDouble()
                    val fill = if (r > 0) area / (Math.PI * r * r) else 0.0
                    if (fill >= v.minFill) {
                        found.add(Blob(
                            FrameMath.fullResolution(center.x, shrink), FrameMath.fullResolution(center.y, shrink),
                            r * shrink, color, fill, area * shrink * shrink))
                    }
                }
                contour.release()
            }
        }

        range(v.yellowHMin, v.yellowHMax, v.yellowSMin, v.yellowVMin, mask)
        collect(TrackerConfig.POLLEN)
        // red wraps around H = 0
        range(0, v.redHLowMax, v.redSMin, v.redVMin, mask)
        range(v.redHHighMin, 180, v.redSMin, v.redVMin, mask2)
        Core.bitwise_or(mask, mask2, mask)
        collect(TrackerConfig.RED_NECTAR)
        range(v.blueHMin, v.blueHMax, v.blueSMin, v.blueVMin, mask)
        collect(TrackerConfig.BLUE_NECTAR)
        return found
    }
}

/** What the camera thread hands the OpMode loop for one frame. */
class FrameBalls(
    val seq: Long,
    /** Capture time (s, System.nanoTime clock). */
    val t: Double,
    val stampUsed: Boolean,
    /** Capture to end of detection (s). */
    val latencyS: Double,
    val processMs: Double,
    /** Ball centers in the upright image, which is what the intrinsics describe. */
    val detections: List<PixelDetection>,
    val blobs: List<Blob>,
)

/**
 * VisionPortal processor for the ball camera. It runs the detector on the camera
 * thread and publishes the newest frame's detections in `latest`. The OpMode loop
 * reads that and runs the tracker, so the tracker itself stays single threaded.
 */
class BallVisionProcessor : VisionProcessor {
    private val detector = BallDetector()
    private var seq = 0L
    private val paint = Paint().apply {
        style = Paint.Style.STROKE
        isAntiAlias = true
    }

    @Volatile var latest: FrameBalls? = null
        private set
    @Volatile var width = 0
        private set
    @Volatile var height = 0
        private set

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
        this.width = width
        this.height = height
    }

    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any? {
        val start = System.nanoTime()
        val time = FrameMath.frameTime(start, captureTimeNanos, AutoDogTuning.captureLatencyS)
        val blobs = detector.detect(frame)
        val w = frame.cols()
        val h = frame.rows()
        val upsideDown = AutoDogTuning.camUpsideDown
        val detections = blobs.map {
            if (upsideDown) {
                val (u, v) = FrameMath.rotated180(it.u, it.v, w, h)
                PixelDetection(u, v, it.color)
            } else {
                PixelDetection(it.u, it.v, it.color)
            }
        }
        val end = System.nanoTime()
        val balls = FrameBalls(
            ++seq, time.t, time.stampUsed, end / 1e9 - time.t, (end - start) / 1e6, detections, blobs)
        latest = balls
        return balls
    }

    /** Circles each detected ball on the Driver Station camera stream. */
    override fun onDrawFrame(
        canvas: Canvas,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?,
    ) {
        val balls = userContext as? FrameBalls ?: return
        paint.strokeWidth = 3 * scaleCanvasDensity
        for (b in balls.blobs) {
            paint.color = when (b.color) {
                TrackerConfig.POLLEN -> Color.YELLOW
                TrackerConfig.RED_NECTAR -> Color.RED
                else -> Color.CYAN
            }
            canvas.drawCircle(
                b.u.toFloat() * scaleBmpPxToCanvasPx, b.v.toFloat() * scaleBmpPxToCanvasPx,
                (b.radius.toFloat() + 2f) * scaleBmpPxToCanvasPx, paint)
        }
    }
}
