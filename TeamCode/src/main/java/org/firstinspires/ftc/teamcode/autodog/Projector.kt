package org.firstinspires.ftc.teamcode.autodog

import kotlin.math.cos
import kotlin.math.sin

/** A detection projected onto the ball plane. cov is the 2x2 covariance [xx, xy, yx, yy] in m^2. */
class Projection(
    val x: Double,
    val y: Double,
    val cov: DoubleArray,
    val u: Double,
    val v: Double,
    /** Ray length from the camera to the hit point (m). */
    val range: Double,
)

/**
 * Projects pixels onto the plane z = planeHeight in the tracker field frame, for one
 * robot pose. Port of python/autodog/geometry.py (design doc 2.1 to 2.4).
 *
 * Frames: field (x/y on the floor, z up), robot (x forward, y left, z up) and camera
 * (OpenCV: x right, y down, z forward). Matrices are 3x3, row major.
 */
class Projector(
    private val intrinsics: CameraIntrinsics,
    mount: CameraMount,
    val planeHeight: Double,
    val maxRange: Double,
) {
    private val rRc: DoubleArray
    private val tRc = doubleArrayOf(mount.x, mount.y, mount.z)
    private var rFc = DoubleArray(9)
    private var oF = DoubleArray(3)
    var poseX = 0.0; private set
    var poseY = 0.0; private set
    var poseTheta = 0.0; private set

    init {
        // Camera axes in robot axes for a level, forward-facing camera:
        // camera x (right) -> robot -y, camera y (down) -> robot -z, camera z -> robot +x.
        val robotFromCv = doubleArrayOf(
            0.0, 0.0, 1.0,
            -1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
        )
        rRc = mul3(mul3(mul3(rotZ(mount.yawRad), rotY(mount.pitchRad)), rotX(mount.rollRad)), robotFromCv)
        setPose(0.0, 0.0, 0.0)
    }

    /** Camera rotation and origin in the field frame for a robot pose. */
    private fun cameraInField(x: Double, y: Double, theta: Double, rOut: DoubleArray, oOut: DoubleArray) {
        val rz = rotZ(theta)
        mul3(rz, rRc).copyInto(rOut)
        oOut[0] = rz[0] * tRc[0] + rz[1] * tRc[1] + rz[2] * tRc[2] + x
        oOut[1] = rz[3] * tRc[0] + rz[4] * tRc[1] + rz[5] * tRc[2] + y
        oOut[2] = rz[6] * tRc[0] + rz[7] * tRc[1] + rz[8] * tRc[2]
    }

    fun setPose(x: Double, y: Double, theta: Double) {
        poseX = x
        poseY = y
        poseTheta = theta
        cameraInField(x, y, theta, rFc, oF)
    }

    /**
     * Undistorted, normalized image coordinates of a pixel. Same fixed-point
     * iteration (5 steps) as cv2.undistortPoints, which the Python tracker uses.
     */
    fun normalized(u: Double, v: Double): DoubleArray {
        val k1 = intrinsics.dist[0]
        val k2 = intrinsics.dist[1]
        val p1 = intrinsics.dist[2]
        val p2 = intrinsics.dist[3]
        val k3 = intrinsics.dist[4]
        val x0 = (u - intrinsics.cx) / intrinsics.fx
        val y0 = (v - intrinsics.cy) / intrinsics.fy
        var x = x0
        var y = y0
        repeat(5) {
            val r2 = x * x + y * y
            val radial = 1.0 + ((k3 * r2 + k2) * r2 + k1) * r2
            if (radial <= 0.0) return doubleArrayOf(x, y)
            val dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
            val dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
            x = (x0 - dx) / radial
            y = (y0 - dy) / radial
        }
        return doubleArrayOf(x, y)
    }

    /** Ray through normalized (xn, yn) to the plane: [x, y, range], or null on a miss. */
    private fun hit(xn: Double, yn: Double, r: DoubleArray, o: DoubleArray): DoubleArray? {
        val dx = r[0] * xn + r[1] * yn + r[2]
        val dy = r[3] * xn + r[4] * yn + r[5]
        val dz = r[6] * xn + r[7] * yn + r[8]
        val s = (planeHeight - o[2]) / dz
        if (s.isNaN() || s.isInfinite() || s <= 0.0 || s > maxRange) return null
        return doubleArrayOf(o[0] + s * dx, o[1] + s * dy, s)
    }

    /**
     * Projects one detection and propagates pixel noise through the projection
     * (central differences of +/- step pixels). Pose noise adds sigmaXy^2 I plus
     * sigmaTheta^2 j j^T with j = d(point)/d(theta). Null when the ray misses the
     * plane or is longer than maxRange.
     */
    fun project(
        u: Double,
        v: Double,
        pixelSigma: Double,
        step: Double = 0.5,
        poseSigmaXy: Double = 0.0,
        poseSigmaTheta: Double = 0.0,
    ): Projection? {
        val n0 = normalized(u, v)
        val center = hit(n0[0], n0[1], rFc, oF) ?: return null
        val uPlus = normalized(u + step, v).let { hit(it[0], it[1], rFc, oF) } ?: return null
        val uMinus = normalized(u - step, v).let { hit(it[0], it[1], rFc, oF) } ?: return null
        val vPlus = normalized(u, v + step).let { hit(it[0], it[1], rFc, oF) } ?: return null
        val vMinus = normalized(u, v - step).let { hit(it[0], it[1], rFc, oF) } ?: return null

        // Jacobian d(x, y) / d(u, v)
        val jxu = (uPlus[0] - uMinus[0]) / (2 * step)
        val jyu = (uPlus[1] - uMinus[1]) / (2 * step)
        val jxv = (vPlus[0] - vMinus[0]) / (2 * step)
        val jyv = (vPlus[1] - vMinus[1]) / (2 * step)
        val var2 = pixelSigma * pixelSigma
        val cov = doubleArrayOf(
            var2 * (jxu * jxu + jxv * jxv), var2 * (jxu * jyu + jxv * jyv),
            var2 * (jxu * jyu + jxv * jyv), var2 * (jyu * jyu + jyv * jyv),
        )
        if (poseSigmaXy > 0) {
            cov[0] += poseSigmaXy * poseSigmaXy
            cov[3] += poseSigmaXy * poseSigmaXy
        }
        if (poseSigmaTheta > 0) {
            val dth = 1e-3
            val r = DoubleArray(9)
            val o = DoubleArray(3)
            cameraInField(poseX, poseY, poseTheta + dth, r, o)
            val plus = hit(n0[0], n0[1], r, o)
            cameraInField(poseX, poseY, poseTheta - dth, r, o)
            val minus = hit(n0[0], n0[1], r, o)
            if (plus != null && minus != null) {
                val jx = (plus[0] - minus[0]) / (2 * dth)
                val jy = (plus[1] - minus[1]) / (2 * dth)
                val s2 = poseSigmaTheta * poseSigmaTheta
                cov[0] += s2 * jx * jx
                cov[1] += s2 * jx * jy
                cov[2] += s2 * jx * jy
                cov[3] += s2 * jy * jy
            }
        }
        return Projection(center[0], center[1], cov, u, v, center[2])
    }

    /** Field point -> pixel (the forward camera model), or null behind the camera. For tests and overlays. */
    fun fieldToPixel(x: Double, y: Double, z: Double): DoubleArray? {
        val px = x - oF[0]
        val py = y - oF[1]
        val pz = z - oF[2]
        // R_FC^T (p - o)
        val cx = rFc[0] * px + rFc[3] * py + rFc[6] * pz
        val cy = rFc[1] * px + rFc[4] * py + rFc[7] * pz
        val cz = rFc[2] * px + rFc[5] * py + rFc[8] * pz
        if (cz <= 1e-6) return null
        val xn = cx / cz
        val yn = cy / cz
        val d = intrinsics.dist
        val r2 = xn * xn + yn * yn
        val radial = 1.0 + ((d[4] * r2 + d[1]) * r2 + d[0]) * r2
        val xd = xn * radial + 2 * d[2] * xn * yn + d[3] * (r2 + 2 * xn * xn)
        val yd = yn * radial + d[2] * (r2 + 2 * yn * yn) + 2 * d[3] * xn * yn
        return doubleArrayOf(intrinsics.fx * xd + intrinsics.cx, intrinsics.fy * yd + intrinsics.cy)
    }

    private companion object {
        fun rotX(a: Double): DoubleArray {
            val c = cos(a)
            val s = sin(a)
            return doubleArrayOf(1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c)
        }

        fun rotY(a: Double): DoubleArray {
            val c = cos(a)
            val s = sin(a)
            return doubleArrayOf(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c)
        }

        fun rotZ(a: Double): DoubleArray {
            val c = cos(a)
            val s = sin(a)
            return doubleArrayOf(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0)
        }

        fun mul3(a: DoubleArray, b: DoubleArray): DoubleArray {
            val out = DoubleArray(9)
            for (i in 0 until 3) for (j in 0 until 3) {
                out[3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j]
            }
            return out
        }
    }
}
