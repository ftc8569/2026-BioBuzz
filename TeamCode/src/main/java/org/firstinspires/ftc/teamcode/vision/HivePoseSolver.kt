package org.firstinspires.ftc.teamcode.vision

import kotlin.math.*

/** Field axes: +X toward blue, +Y away from the audience, +Z up; origin at field center. */
data class Vec3(val x: Double, val y: Double, val z: Double) {
    operator fun plus(b: Vec3) = Vec3(x + b.x, y + b.y, z + b.z)
    operator fun minus(b: Vec3) = Vec3(x - b.x, y - b.y, z - b.z)
    operator fun times(s: Double) = Vec3(x * s, y * s, z * s)
    fun norm() = sqrt(x * x + y * y + z * z)
}

/** Row-major proper rotation matrix. */
class Mat3(private val m: DoubleArray) {
    operator fun times(v: Vec3) = Vec3(
        m[0] * v.x + m[1] * v.y + m[2] * v.z,
        m[3] * v.x + m[4] * v.y + m[5] * v.z,
        m[6] * v.x + m[7] * v.y + m[8] * v.z
    )
    operator fun times(b: Mat3) = Mat3(DoubleArray(9) { i ->
        val row = i / 3; val col = i % 3
        (0..2).sumOf { k -> m[row * 3 + k] * b.m[k * 3 + col] }
    })
    fun transpose() = Mat3(DoubleArray(9) { i -> m[(i % 3) * 3 + i / 3] })
    fun yaw() = atan2(m[3], m[0])
    fun log(): Vec3 {
        val cosine = ((m[0] + m[4] + m[8] - 1.0) / 2.0).coerceIn(-1.0, 1.0)
        val sineVector = Vec3(m[7] - m[5], m[2] - m[6], m[3] - m[1]) * 0.5
        val sine = sineVector.norm()
        val angle = atan2(sine, cosine)
        if (cosine < -0.9999) {
            val diagonal = doubleArrayOf(m[0], m[4], m[8])
            val largest = diagonal.indices.maxByOrNull { diagonal[it] }!!
            val a = sqrt(((diagonal[largest] + 1.0) / 2.0).coerceAtLeast(0.0))
            if (a < 1e-10) return Vec3(Math.PI, 0.0, 0.0)
            val axis = when (largest) {
                0 -> Vec3(a, (m[1] + m[3]) / (4 * a), (m[2] + m[6]) / (4 * a))
                1 -> Vec3((m[1] + m[3]) / (4 * a), a, (m[5] + m[7]) / (4 * a))
                else -> Vec3((m[2] + m[6]) / (4 * a), (m[5] + m[7]) / (4 * a), a)
            }
            return axis * (angle / axis.norm())
        }
        return if (sine < 1e-10) sineVector else sineVector * (angle / sine)
    }
    companion object {
        val IDENTITY = Mat3(doubleArrayOf(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0))
        fun rows(vararg x: Double) = Mat3(x.copyOf())
        fun rotX(a: Double): Mat3 { val c = cos(a); val s = sin(a); return rows(1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c) }
        fun rotY(a: Double): Mat3 { val c = cos(a); val s = sin(a); return rows(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c) }
        fun rotZ(a: Double): Mat3 { val c = cos(a); val s = sin(a); return rows(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0) }
        fun yawPitchRoll(yaw: Double, pitch: Double, roll: Double) = rotZ(yaw) * rotY(pitch) * rotX(roll)
    }
}

data class Transform3(val p: Vec3, val r: Mat3) {
    operator fun times(b: Transform3) = Transform3(p + r * b.p, r * b.r)
    fun inverse(): Transform3 { val rt = r.transpose(); return Transform3(rt * (p * -1.0), rt) }
}

data class RobotPose(val xMeters: Double, val yMeters: Double, val headingRadians: Double) {
    fun asTransform() = Transform3(Vec3(xMeters, yMeters, 0.0), Mat3.rotZ(headingRadians))
}

data class TagObservation(val id: Int, val robotToTag: Transform3)

data class HivePoseEstimate(
    val robot: RobotPose,
    /** Rotation relative to the official CAD's initial HIVE position. */
    val redHiveAngleRadians: Double?,
    val blueHiveAngleRadians: Double?,
    val tagCount: Int,
    val residualMeters: Double
)

/**
 * Nominal positions and orientations of the four sticker assemblies in FIRST's 2026-27 STEP CAD.
 * CAD coordinates have Y up and Z toward the audience; convert to our field axes here.
 * Individual tag centers are spaced 4.5 in, with the asymmetry shown in manual fig. 9-15.
 */
object HiveFieldModel {
    private const val INCH = 0.0254
    private const val PIVOT_HEIGHT = 43.95 * INCH
    private val cadToField = Mat3.rows(1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0)
    // STEP sticker X/Y point opposite Limelight's legacy target X/Y (right/down as viewed).
    private val stickerToTagAxes = Mat3.rotZ(Math.PI)
    private data class Cluster(val startId: Int, val cadOrigin: Vec3, val cadRotation: Mat3)
    private val clusters = listOf(
        Cluster(30, Vec3(-0.53954, 0.8737950292, -0.3444707674), Mat3.rows(1.0, 0.0, 0.0, 0.0, 0.5, -sqrt(3.0) / 2, 0.0, sqrt(3.0) / 2, 0.5)),
        Cluster(34, Vec3(-0.10774, 1.29338, 0.38227), Mat3.rows(-1.0, 0.0, 0.0, 0.0, -0.5, -sqrt(3.0) / 2, 0.0, -sqrt(3.0) / 2, 0.5)),
        Cluster(38, Vec3(0.5399623190, 0.8737950292, 0.3444707674), Mat3.rows(-1.0, 0.0, 0.0, 0.0, 0.5, -sqrt(3.0) / 2, 0.0, -sqrt(3.0) / 2, -0.5)),
        Cluster(42, Vec3(0.10816, 1.29338, -0.38227), Mat3.rows(1.0, 0.0, 0.0, 0.0, -0.5, -sqrt(3.0) / 2, 0.0, sqrt(3.0) / 2, -0.5))
    )
    private val localXInches = doubleArrayOf(6.5, 2.0, -2.5, -7.0)

    fun isRed(id: Int) = id in 30..37
    fun isBlue(id: Int) = id in 38..45

    fun tagPose(id: Int, hiveAngleRadians: Double): Transform3? {
        val cluster = clusters.firstOrNull { id in it.startId..it.startId + 3 } ?: return null
        val offset = Vec3(localXInches[id - cluster.startId] * INCH, 0.0, 0.0)
        val cadCenter = cluster.cadOrigin + cluster.cadRotation * offset
        val nominal = Transform3(cadToField * cadCenter, cadToField * cluster.cadRotation * stickerToTagAxes)
        val pivot = Vec3(0.0, 0.0, PIVOT_HEIGHT)
        val rotation = Mat3.rotX(hiveAngleRadians)
        return Transform3(pivot + rotation * (nominal.p - pivot), rotation * nominal.r)
    }
}

/** One independent nonlinear fit per camera frame. No state is retained between calls. */
class HivePoseSolver {
    fun solve(rawObservations: List<TagObservation>): HivePoseEstimate? = solveInternal(rawObservations, true)

    private fun solveInternal(rawObservations: List<TagObservation>, mayDropOutlier: Boolean): HivePoseEstimate? {
        val observations = rawObservations.filter { (HiveFieldModel.isRed(it.id) || HiveFieldModel.isBlue(it.id)) &&
            listOf(it.robotToTag.p.x, it.robotToTag.p.y, it.robotToTag.p.z).all(Double::isFinite) }
        if (observations.isEmpty()) return null
        val red = observations.any { HiveFieldModel.isRed(it.id) }
        val blue = observations.any { HiveFieldModel.isBlue(it.id) }
        val dimension = 3 + red.compareTo(false) + blue.compareTo(false)
        val redIndex = if (red) 3 else -1
        val blueIndex = if (blue) if (red) 4 else 3 else -1
        var best: DoubleArray? = null
        var bestCost = Double.POSITIVE_INFINITY
        val redSeeds = if (red) listOf(0.0, 1.05) else listOf(0.0)
        val blueSeeds = if (blue) listOf(-1.05, 0.0) else listOf(0.0)
        for (rq in redSeeds) for (bq in blueSeeds) {
            val first = observations.first()
            val nominal = HiveFieldModel.tagPose(first.id, if (HiveFieldModel.isRed(first.id)) rq else bq)!!
            val robot3d = nominal * first.robotToTag.inverse()
            val x = DoubleArray(dimension)
            x[0] = robot3d.p.x; x[1] = robot3d.p.y; x[2] = robot3d.r.yaw()
            if (red) x[redIndex] = rq
            if (blue) x[blueIndex] = bq
            optimize(x, observations, redIndex, blueIndex)
            val cost = cost(x, observations, redIndex, blueIndex)
            if (cost < bestCost) { bestCost = cost; best = x }
        }
        val x = best ?: return null
        if (mayDropOutlier && observations.size >= 3) {
            val worst = observations.indices.maxByOrNull { i ->
                sqrt(residual(x, observations[i], redIndex, blueIndex).sumOf { it * it })
            }!!
            val worstError = sqrt(residual(x, observations[worst], redIndex, blueIndex).sumOf { it * it })
            if (worstError > 0.15) {
                val reduced = observations.filterIndexed { i, _ -> i != worst }
                solveInternal(reduced, false)?.let { return it }
            }
        }
        val rms = sqrt(bestCost / observations.size)
        if (!rms.isFinite() || rms > 0.18 || abs(x[0]) > 2.0 || abs(x[1]) > 2.0) return null
        return HivePoseEstimate(RobotPose(x[0], x[1], wrap(x[2])),
            if (red) x[redIndex] else null, if (blue) x[blueIndex] else null,
            observations.size, rms)
    }

    private fun residual(x: DoubleArray, o: TagObservation, ri: Int, bi: Int): DoubleArray {
        val angle = if (HiveFieldModel.isRed(o.id)) x[ri] else x[bi]
        val predicted = RobotPose(x[0], x[1], x[2]).asTransform() * o.robotToTag
        val model = HiveFieldModel.tagPose(o.id, angle)!!
        val dp = predicted.p - model.p
        val dr = (predicted.r.transpose() * model.r).log() * 0.5
        return doubleArrayOf(dp.x, dp.y, dp.z, dr.x, dr.y, dr.z)
    }

    private fun cost(x: DoubleArray, obs: List<TagObservation>, ri: Int, bi: Int): Double {
        return obs.sumOf { o ->
            val norm = sqrt(residual(x, o, ri, bi).sumOf { it * it })
            if (norm <= 0.10) norm * norm else 0.20 * norm - 0.01
        }
    }

    private fun optimize(x: DoubleArray, obs: List<TagObservation>, ri: Int, bi: Int) {
        var damping = 1e-3
        repeat(35) {
            val n = x.size
            val h = Array(n) { DoubleArray(n) }
            val g = DoubleArray(n)
            for (o in obs) {
                val r = residual(x, o, ri, bi)
                val norm = sqrt(r.sumOf { it * it })
                val weight = if (norm <= 0.10) 1.0 else 0.10 / norm
                val jac = Array(n) { j ->
                    val shifted = x.copyOf(); shifted[j] += 1e-5
                    val rr = residual(shifted, o, ri, bi)
                    DoubleArray(6) { k -> (rr[k] - r[k]) / 1e-5 }
                }
                for (j in 0 until n) {
                    g[j] += weight * (0 until 6).sumOf { k -> jac[j][k] * r[k] }
                    for (k in 0 until n) h[j][k] += weight * (0 until 6).sumOf { t -> jac[j][t] * jac[k][t] }
                }
            }
            for (j in x.indices) h[j][j] += damping
            val step = linearSolve(h, DoubleArray(x.size) { -g[it] }) ?: return
            val candidate = DoubleArray(x.size) { x[it] + step[it] }
            if (ri >= 0) candidate[ri] = candidate[ri].coerceIn(-0.20, 1.30)
            if (bi >= 0) candidate[bi] = candidate[bi].coerceIn(-1.30, 0.20)
            if (cost(candidate, obs, ri, bi) < cost(x, obs, ri, bi)) {
                candidate.copyInto(x)
                damping = (damping / 3).coerceAtLeast(1e-8)
                if (step.maxOf { abs(it) } < 1e-7) return
            } else damping = (damping * 10).coerceAtMost(1e8)
        }
    }

    private fun linearSolve(a: Array<DoubleArray>, b: DoubleArray): DoubleArray? {
        val n = b.size
        for (i in 0 until n) {
            val pivot = (i until n).maxByOrNull { abs(a[it][i]) } ?: return null
            if (abs(a[pivot][i]) < 1e-12) return null
            val row = a[i]; a[i] = a[pivot]; a[pivot] = row
            val value = b[i]; b[i] = b[pivot]; b[pivot] = value
            val scale = a[i][i]
            for (k in i until n) a[i][k] /= scale
            b[i] /= scale
            for (j in 0 until n) if (j != i) {
                val f = a[j][i]
                for (k in i until n) a[j][k] -= f * a[i][k]
                b[j] -= f * b[i]
            }
        }
        return b
    }

    private fun wrap(a: Double) = atan2(sin(a), cos(a))
}
