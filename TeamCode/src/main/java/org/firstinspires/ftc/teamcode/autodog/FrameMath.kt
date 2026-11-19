package org.firstinspires.ftc.teamcode.autodog

/** Frame bookkeeping for the ball camera that needs no OpenCV, so it can be unit tested. */
object FrameMath {
    /** A capture stamp is believed when it is at most this far in the past (s). */
    const val MAX_STAMP_AGE_S = 0.5

    /** Capture time of a frame in seconds on the System.nanoTime clock, and whether the camera's stamp was used. */
    class FrameTime(val t: Double, val stampUsed: Boolean, val stampAgeS: Double)

    /**
     * The SDK stamps webcam frames on the System.nanoTime clock. If a stamp is
     * missing or implausible (in the future, or older than MAX_STAMP_AGE_S), fall
     * back to the time the frame reached the processor. extraLatencyS is then
     * subtracted either way: it is the tunable for delay the stamp does not cover.
     */
    fun frameTime(nowNanos: Long, captureNanos: Long, extraLatencyS: Double): FrameTime {
        val age = (nowNanos - captureNanos) / 1e9
        val ok = captureNanos > 0 && age >= 0.0 && age <= MAX_STAMP_AGE_S
        val base = if (ok) captureNanos else nowNanos
        return FrameTime(base / 1e9 - extraLatencyS, ok, age)
    }

    /**
     * Pixel in the full camera image for a pixel found in an image shrunk by an
     * integer factor (pixel centers line up as in cv::resize).
     */
    fun fullResolution(coord: Double, shrink: Int): Double = (coord + 0.5) * shrink - 0.5

    /** The same pixel after turning the image 180 degrees, for a camera mounted upside down. */
    fun rotated180(u: Double, v: Double, width: Int, height: Int): Pair<Double, Double> =
        (width - 1 - u) to (height - 1 - v)
}
