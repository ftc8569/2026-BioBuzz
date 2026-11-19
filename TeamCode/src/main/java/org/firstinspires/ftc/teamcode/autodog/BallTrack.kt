package org.firstinspires.ftc.teamcode.autodog

import kotlin.math.max

/**
 * One ball: a constant-velocity Kalman filter over (x, y, vx, vy) in the tracker
 * field frame. Port of python/autodog/kalman.py (design doc 2.6). The measurement
 * is the ball's (x, y), so H picks the first two states. Matrices are row major.
 */
class BallTrack(
    val id: Int,
    zx: Double,
    zy: Double,
    /** 2x2 measurement covariance [xx, xy, yx, yy] of the first detection. */
    r: DoubleArray,
    /** Time the state refers to (s). */
    var t: Double,
    velocityVar: Double,
) {
    /** State: x, y, vx, vy. */
    var x = doubleArrayOf(zx, zy, 0.0, 0.0)
        private set
    /** 4x4 covariance. */
    var p = DoubleArray(16)
        private set
    var hits = 1
        private set
    var framesUnseen = 0
    var ageFrames = 1

    init {
        p[0] = r[0]; p[1] = r[1]
        p[4] = r[2]; p[5] = r[3]
        p[10] = velocityVar
        p[15] = velocityVar
    }

    fun confirmed(confirmHits: Int) = hits >= confirmHits

    /** Trace of the position covariance (m^2). */
    val positionTrace: Double get() = p[0] + p[5]

    /** State and covariance at time t, without changing the track. */
    fun predicted(time: Double, accelSigma: Double): Pair<DoubleArray, DoubleArray> {
        val dt = max(0.0, time - t)
        if (dt == 0.0) return x.copyOf() to p.copyOf()
        val f = doubleArrayOf(
            1.0, 0.0, dt, 0.0,
            0.0, 1.0, 0.0, dt,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        )
        val xNew = doubleArrayOf(x[0] + dt * x[2], x[1] + dt * x[3], x[2], x[3])
        val pNew = mul(mul(f, p, 4, 4, 4), transpose(f, 4, 4), 4, 4, 4)
        // Process noise for white acceleration
        val q = accelSigma * accelSigma
        val a = q * dt * dt * dt * dt / 4
        val b = q * dt * dt * dt / 2
        val c = q * dt * dt
        pNew[0] += a; pNew[2] += b
        pNew[5] += a; pNew[7] += b
        pNew[8] += b; pNew[10] += c
        pNew[13] += b; pNew[15] += c
        return xNew to pNew
    }

    /** Time update. Never runs backwards. */
    fun predict(time: Double, accelSigma: Double) {
        if (time <= t) return
        val (xNew, pNew) = predicted(time, accelSigma)
        x = xNew
        p = pNew
        t = time
    }

    /** Inverse of S = H P H^T + R, as [a, b, c, d]. */
    private fun innovationInverse(r: DoubleArray): DoubleArray {
        val s00 = p[0] + r[0]
        val s01 = p[1] + r[1]
        val s10 = p[4] + r[2]
        val s11 = p[5] + r[3]
        val det = s00 * s11 - s01 * s10
        return doubleArrayOf(s11 / det, -s01 / det, -s10 / det, s00 / det)
    }

    /** Squared Mahalanobis distance from the predicted position to a measurement. */
    fun mahalanobis2(zx: Double, zy: Double, r: DoubleArray): Double {
        val si = innovationInverse(r)
        val y0 = zx - x[0]
        val y1 = zy - x[1]
        return y0 * (si[0] * y0 + si[1] * y1) + y1 * (si[2] * y0 + si[3] * y1)
    }

    /** Measurement update in Joseph form, which keeps P symmetric positive semidefinite. */
    fun update(zx: Double, zy: Double, r: DoubleArray) {
        val si = innovationInverse(r)
        val y0 = zx - x[0]
        val y1 = zy - x[1]
        // K = (S^-1 H P)^T, 4x2
        val k = DoubleArray(8)
        for (i in 0 until 4) {
            k[2 * i] = si[0] * p[i] + si[1] * p[4 + i]
            k[2 * i + 1] = si[2] * p[i] + si[3] * p[4 + i]
        }
        val xNew = DoubleArray(4) { x[it] + k[2 * it] * y0 + k[2 * it + 1] * y1 }
        // I - K H
        val ikh = DoubleArray(16)
        for (i in 0 until 4) {
            ikh[4 * i + i] = 1.0
            ikh[4 * i] -= k[2 * i]
            ikh[4 * i + 1] -= k[2 * i + 1]
        }
        val pNew = mul(mul(ikh, p, 4, 4, 4), transpose(ikh, 4, 4), 4, 4, 4)
        val krk = mul(mul(k, r, 4, 2, 2), transpose(k, 4, 2), 4, 2, 4)
        for (i in 0 until 16) pNew[i] += krk[i]
        x = xNew
        p = pNew
        hits += 1
        framesUnseen = 0
    }

    private companion object {
        /** a (rows x inner) times b (inner x cols). */
        fun mul(a: DoubleArray, b: DoubleArray, rows: Int, inner: Int, cols: Int): DoubleArray {
            val out = DoubleArray(rows * cols)
            for (i in 0 until rows) for (j in 0 until cols) {
                var sum = 0.0
                for (m in 0 until inner) sum += a[i * inner + m] * b[m * cols + j]
                out[i * cols + j] = sum
            }
            return out
        }

        fun transpose(a: DoubleArray, rows: Int, cols: Int): DoubleArray {
            val out = DoubleArray(rows * cols)
            for (i in 0 until rows) for (j in 0 until cols) out[j * rows + i] = a[i * cols + j]
            return out
        }
    }
}
