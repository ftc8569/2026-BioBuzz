package org.firstinspires.ftc.teamcode.autodog

import org.firstinspires.ftc.teamcode.vision.Transform3
import org.firstinspires.ftc.teamcode.vision.Vec3
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.tan

/*
 * Onboard ball tracking: the competition version of Auto Dog, with no laptop.
 *
 * limelight/autodog_balls.py finds balls in the image and returns pixel circles.
 * This file turns them into field positions (tracker frame: corner origin,
 * meters, CCW heading, the same frame as AutoDogLink) and tracks them, so the
 * rest of Auto Dog (AutoDogTarget, ChaseController) is unchanged.
 *
 * Pure Kotlin with no FTC classes, so all of it runs in JVM unit tests.
 */

/** One ball circle from the SnapScript, in image pixels. */
data class PixelBall(val color: Char, val u: Double, val v: Double, val radiusPx: Double)

data class BallFrame(val width: Double, val height: Double, val balls: List<PixelBall>)

object LimelightBallFrame {
    const val FORMAT_VERSION = 1.0
    const val HEADER = 4
    const val FIELDS_PER_BALL = 4

    fun colorFor(code: Double): Char? = when (code) {
        1.0 -> 'P'
        2.0 -> 'N'
        else -> null
    }

    /** Parses llpython from autodog_balls.py, or returns null if it isn't that format. */
    fun parse(out: DoubleArray?): BallFrame? {
        if (out == null || out.size < HEADER || out[0] != FORMAT_VERSION) return null
        val width = out[1]
        val height = out[2]
        val n = out[3].toInt()
        if (width <= 0 || height <= 0 || n < 0 || out.size < HEADER + n * FIELDS_PER_BALL) return null
        val balls = (0 until n).mapNotNull { i ->
            val o = HEADER + i * FIELDS_PER_BALL
            val color = colorFor(out[o]) ?: return@mapNotNull null
            if (!(out[o + 1].isFinite() && out[o + 2].isFinite() && out[o + 3] > 0)) return@mapNotNull null
            PixelBall(color, out[o + 1], out[o + 2], out[o + 3])
        }
        return BallFrame(width, height, balls)
    }
}

data class PlanarPose(val x: Double, val y: Double, val theta: Double) {
    /** A point given in this pose's robot frame (x forward, y left), in the outer frame. */
    fun transform(rx: Double, ry: Double): Pair<Double, Double> {
        val c = cos(theta)
        val s = sin(theta)
        return Pair(x + c * rx - s * ry, y + s * rx + c * ry)
    }
}

/**
 * Pinhole camera from the field of view, so it scales to any stream resolution.
 * Ray axes match Limelight's camera frame (right, down, out), which
 * HiveCameraMount.robotToCamera() maps into the robot frame.
 */
class BallProjector(
    var hfovDeg: Double = 54.5,
    var vfovDeg: Double = 42.0,
    var robotToCamera: Transform3,
    var ballRadiusM: Double = 1.5 * 0.0254,
    /** Reject if the seen radius differs from the expected one by more than this factor; <= 1 disables. */
    var radiusTolerance: Double = 1.8,
    var maxRangeM: Double = 3.0,
) {
    fun focalX(width: Double) = (width / 2) / tan(Math.toRadians(hfovDeg) / 2)
    fun focalY(height: Double) = (height / 2) / tan(Math.toRadians(vfovDeg) / 2)

    /** Ball center in the robot frame (meters, x forward, y left), or null if rejected. */
    fun project(ball: PixelBall, width: Double, height: Double): Pair<Double, Double>? {
        val fx = focalX(width)
        val ray = Vec3((ball.u - width / 2) / fx, (ball.v - height / 2) / focalY(height), 1.0)
        val origin = robotToCamera.p
        val dir = robotToCamera.r * ray
        // Intersect with the plane of ball centers, one radius above the floor.
        if (dir.z >= -1e-9) return null
        val t = (ballRadiusM - origin.z) / dir.z
        if (t <= 0) return null
        val hit = origin + dir * t
        val range = (hit - origin).norm()
        if (range > maxRangeM) return null
        if (radiusTolerance > 1.0) {
            // Rough expected size; a ball in the air or a same-colored object fails this.
            val expected = fx * ballRadiusM / range
            val ratio = ball.radiusPx / expected
            if (ratio > radiusTolerance || ratio < 1 / radiusTolerance) return null
        }
        return Pair(hit.x, hit.y)
    }
}

/** Recent robot poses, so a frame captured 30-60 ms ago is placed with the pose from then. */
class PoseHistory(private val capacity: Int = 128) {
    private val t = DoubleArray(capacity)
    private val px = DoubleArray(capacity)
    private val py = DoubleArray(capacity)
    private val pth = DoubleArray(capacity)
    private var count = 0
    private var next = 0

    fun add(time: Double, pose: PlanarPose) {
        if (count > 0 && time <= t[idx(count - 1)]) return
        t[next] = time; px[next] = pose.x; py[next] = pose.y; pth[next] = pose.theta
        next = (next + 1) % capacity
        if (count < capacity) count++
    }

    /** i = 0 is the oldest sample. */
    private fun idx(i: Int) = (next - count + i + capacity) % capacity

    /** Pose at [time], interpolated. Null when [time] is older than the history or there is none. */
    fun at(time: Double): PlanarPose? {
        if (count == 0) return null
        val newest = idx(count - 1)
        if (time >= t[newest]) return PlanarPose(px[newest], py[newest], pth[newest])
        if (time < t[idx(0)]) return null
        for (i in count - 2 downTo 0) {
            val a = idx(i)
            if (t[a] <= time) {
                val b = idx(i + 1)
                val f = (time - t[a]) / (t[b] - t[a])
                val dth = wrap(pth[b] - pth[a])
                return PlanarPose(px[a] + f * (px[b] - px[a]), py[a] + f * (py[b] - py[a]), wrap(pth[a] + f * dth))
            }
        }
        return null
    }

    companion object {
        fun wrap(a: Double) = atan2(sin(a), cos(a))
    }
}

data class FieldBall(val color: Char, val x: Double, val y: Double)

class BallTrack(val id: Int, val color: Char, var x: Double, var y: Double, var lastSeen: Double) {
    var vx = 0.0
    var vy = 0.0
    var hits = 1
}

/**
 * Nearest-neighbor tracker with an alpha-beta filter per ball. It is simpler
 * than the laptop's Kalman filter, but enough to hold an ID and estimate
 * rolling speed.
 */
class BallTracker(
    var gateM: Double = 0.35,
    var alpha: Double = 0.6,
    var beta: Double = 0.3,
    var dropAfterS: Double = 0.6,
    var confirmHits: Int = 2,
    var maxSpeed: Double = 3.0,
) {
    private var nextId = 1
    val tracks = mutableListOf<BallTrack>()

    fun confirmed() = tracks.filter { it.hits >= confirmHits }

    fun update(time: Double, detections: List<FieldBall>) {
        tracks.removeAll { time - it.lastSeen > dropAfterS }
        data class Pair3(val track: BallTrack, val det: Int, val d: Double)
        val pairs = ArrayList<Pair3>()
        for (track in tracks) {
            val dt = time - track.lastSeen
            if (dt <= 0) continue
            val ex = track.x + track.vx * dt
            val ey = track.y + track.vy * dt
            detections.forEachIndexed { i, det ->
                if (det.color != track.color) return@forEachIndexed
                val d = hypot(det.x - ex, det.y - ey)
                if (d <= gateM) pairs += Pair3(track, i, d)
            }
        }
        pairs.sortBy { it.d }
        val usedTracks = HashSet<Int>()
        val usedDets = HashSet<Int>()
        for (p in pairs) {
            if (p.track.id in usedTracks || p.det in usedDets) continue
            usedTracks += p.track.id
            usedDets += p.det
            correct(p.track, detections[p.det], time)
        }
        detections.forEachIndexed { i, det ->
            if (i !in usedDets) tracks += BallTrack(nextId++, det.color, det.x, det.y, time)
        }
    }

    private fun correct(track: BallTrack, det: FieldBall, time: Double) {
        val dt = time - track.lastSeen
        val ex = track.x + track.vx * dt
        val ey = track.y + track.vy * dt
        val rx = det.x - ex
        val ry = det.y - ey
        track.x = ex + alpha * rx
        track.y = ey + alpha * ry
        track.vx += beta * rx / dt
        track.vy += beta * ry / dt
        val speed = hypot(track.vx, track.vy)
        if (speed > maxSpeed) {
            track.vx *= maxSpeed / speed
            track.vy *= maxSpeed / speed
        }
        track.lastSeen = time
        track.hits++
    }
}

/** Picks the ball nearest the intake, and only switches when another is clearly closer. */
class TargetSelector(var switchMarginM: Double = 0.3) {
    var currentId: Int? = null
        private set

    fun select(tracks: List<BallTrack>, robot: PlanarPose, intakeOffset: Double, colorFilter: Char?, now: Double): BallTrack? {
        val (ix, iy) = robot.transform(intakeOffset, 0.0)
        fun dist(b: BallTrack): Double {
            val dt = (now - b.lastSeen).coerceIn(0.0, 0.5)
            return hypot(b.x + b.vx * dt - ix, b.y + b.vy * dt - iy)
        }
        val candidates = tracks.filter { colorFilter == null || it.color == colorFilter }
        val best = candidates.minByOrNull(::dist)
        val current = candidates.firstOrNull { it.id == currentId }
        val chosen = if (current != null && best != null && dist(current) <= dist(best) + switchMarginM) current else best
        currentId = chosen?.id
        return chosen
    }
}

/** Frame in, AutoDogTarget out: the onboard replacement for the laptop. */
class OnboardBallTracker(
    val projector: BallProjector,
    val tracker: BallTracker = BallTracker(),
    val selector: TargetSelector = TargetSelector(),
    val history: PoseHistory = PoseHistory(),
) {
    var fieldGate = true
    var fieldMarginM = 0.1
    var lastDetections = 0
        private set
    var framesUsed = 0
        private set
    private var lastFrameStamp = Double.NaN

    fun addPose(time: Double, pose: PlanarPose) = history.add(time, pose)

    /**
     * Feeds one Limelight result. [frameStamp] identifies the frame so the same
     * result isn't counted twice. [captureTime] is on the same clock as addPose.
     * Returns false when the frame was skipped.
     */
    fun onFrame(frameStamp: Double, captureTime: Double, frame: BallFrame?): Boolean {
        if (frame == null || frameStamp == lastFrameStamp) return false
        val pose = history.at(captureTime) ?: return false
        lastFrameStamp = frameStamp
        val limit = FieldFrame.FIELD_SIZE_M + fieldMarginM
        val detections = frame.balls.mapNotNull { ball ->
            val (rx, ry) = projector.project(ball, frame.width, frame.height) ?: return@mapNotNull null
            val (fx, fy) = pose.transform(rx, ry)
            if (fieldGate && (fx < -fieldMarginM || fy < -fieldMarginM || fx > limit || fy > limit)) null
            else FieldBall(ball.color, fx, fy)
        }
        lastDetections = detections.size
        framesUsed++
        tracker.update(captureTime, detections)
        return true
    }

    fun target(now: Double, robot: PlanarPose, intakeOffset: Double, colorFilter: Char?): AutoDogTarget? {
        val track = selector.select(tracker.confirmed(), robot, intakeOffset, colorFilter, now) ?: return null
        return AutoDogTarget(
            tRc = track.lastSeen, hasTarget = true, trackId = track.id,
            x = track.x, y = track.y, vx = track.vx, vy = track.vy,
            color = track.color, receivedAt = now,
        )
    }

    companion object {
        /** AutoDogVisionTuning.colorFilter: 0 any, 1 POLLEN, 2 NECTAR. */
        fun colorFilter(code: Int): Char? = when (code) {
            1 -> 'P'
            2 -> 'N'
            else -> null
        }
    }
}
