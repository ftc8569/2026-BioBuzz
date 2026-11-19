package org.firstinspires.ftc.teamcode.autodog

/**
 * Tunables for the on-robot ball tracker. This is a port of
 * BioBuzzAutoDog/python/autodog/config.py and keeps its defaults; that file's
 * comments say where each value comes from. Meters, seconds and radians.
 *
 * Tracker field frame: origin at a field corner, x and y on the floor, z up,
 * heading counterclockwise from +x. FieldFrame converts from the team frame.
 */

/** Pinhole intrinsics plus OpenCV distortion (k1, k2, p1, p2, k3) for one resolution. */
class CameraIntrinsics(
    val fx: Double,
    val fy: Double,
    val cx: Double,
    val cy: Double,
    val dist: DoubleArray = DoubleArray(5),
    val width: Int = 1280,
    val height: Int = 720,
) {
    /** Same lens at another resolution: pixels scale, distortion does not. */
    fun scaled(newWidth: Int, newHeight: Int): CameraIntrinsics {
        val sx = newWidth.toDouble() / width
        val sy = newHeight.toDouble() / height
        return CameraIntrinsics(fx * sx, fy * sy, cx * sx, cy * sy, dist.copyOf(), newWidth, newHeight)
    }

    companion object {
        /**
         * The wide global shutter camera at 1280x720, from the chessboard images in
         * BioBuzzAutoDog/python/calib_imgs (RMS 0.26 px). Those images were taken with
         * the camera upside down and rotated 180 degrees before solving, so these
         * values are for an upright image. Recalibrate if the camera or lens changes.
         */
        val GLOBAL_SHUTTER_720P = CameraIntrinsics(
            fx = 393.0838, fy = 393.0318, cx = 666.1668, cy = 440.4564,
            dist = doubleArrayOf(0.0019424, -0.0195761, 0.0007091, 0.0002034, 0.0025904),
            width = 1280, height = 720,
        )
    }
}

/**
 * Camera pose in the robot frame (x forward, y left, z up). pitchRad > 0 tilts the
 * lens down toward the floor, yawRad > 0 turns it left, rollRad rotates the image
 * about the optical axis.
 */
data class CameraMount(
    val x: Double = 0.15,
    val y: Double = 0.0,
    val z: Double = 0.25,
    val pitchRad: Double = Math.toRadians(20.0),
    val yawRad: Double = 0.0,
    val rollRad: Double = 0.0,
)

class TrackerConfig {
    // Field and ball geometry
    var fieldWidth = 144 * FieldFrame.INCH
    var fieldHeight = 144 * FieldFrame.INCH
    var fieldMargin = 0.10
    /** Ball center height for a color with no entry in ballRadii. */
    var ballRadius = 1.4 * FieldFrame.INCH
    /** Per-color radius: each color is projected onto the plane at its own center height. */
    var ballRadii: Map<Char, Double> = emptyMap()
    /** False skips the field-bounds gate, for a floor where odometry does not start at a known spot. */
    var gateField = true
    /** Polygons (tracker frame, [x, y] vertices) where the floor is not flat. Empty by default. */
    var rampPolygons: List<List<DoubleArray>> = emptyList()
    var rampMargin = 0.20
    /** Robot footprint in the robot frame: xMin, xMax, yMin, yMax. */
    var robotFootprint = doubleArrayOf(
        -9 * FieldFrame.INCH, 9 * FieldFrame.INCH, -9 * FieldFrame.INCH, 9 * FieldFrame.INCH)

    // Projection and measurement noise
    var maxRange = 4.0
    var pixelSigma = 1.5
    var jacobianStepPx = 0.5
    var poseSigmaXy = 0.005
    var poseSigmaTheta = Math.toRadians(0.3)

    // Kalman filter
    var accelSigma = 3.0
    var initVelocityVar = 1.0

    // Association and track management
    var gateChi2 = 9.21
    var maxFramesUnseen = 15
    var tentativeMaxFramesUnseen = 2
    var dropTracksUnderRobot = true
    var maxPositionTrace = 1.0
    var confirmHits = 3

    /** Keep the current target unless another confirmed track is closer by more than this. */
    var targetSwitchMargin = 0.20

    companion object {
        const val POLLEN = 'Y'
        const val RED_NECTAR = 'R'
        const val BLUE_NECTAR = 'B'

        /** BIOBUZZ: yellow POLLEN is 2.8 in across, red and blue NECTAR 3.6 in (manual 9.8). */
        fun biobuzz(): TrackerConfig = TrackerConfig().apply {
            ballRadius = 1.4 * FieldFrame.INCH
            ballRadii = mapOf(
                POLLEN to 1.4 * FieldFrame.INCH,
                RED_NECTAR to 1.8 * FieldFrame.INCH,
                BLUE_NECTAR to 1.8 * FieldFrame.INCH,
            )
        }
    }
}
