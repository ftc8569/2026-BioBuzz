package org.firstinspires.ftc.teamcode.autodog

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin

/** One ball seen in a frame: its pixel center and color ('Y', 'R', 'B', or null). */
data class PixelDetection(val u: Double, val v: Double, val color: Char? = null)

/** Robot pose in the tracker field frame. */
data class TrackerPose(val x: Double, val y: Double, val theta: Double)

/** The ball to chase, predicted to the query time. */
class ChaseTarget(
    val trackId: Int,
    val t: Double,
    val x: Double,
    val y: Double,
    val vx: Double,
    val vy: Double,
    val color: Char?,
    /** From the robot center at the query time (m). */
    val distance: Double,
)

enum class Rejection { PROJECTION, FIELD, RAMP, ROBOT }

class TickResult(val t: Double, val pose: TrackerPose?) {
    val accepted = ArrayList<Projection>()
    val rejected = IntArray(Rejection.values().size)
    var matches = 0
    var spawned = 0
    var deleted = 0
    var target: ChaseTarget? = null
}

/**
 * Recent robot poses, interpolated to a frame's capture time. Port of PoseBuffer in
 * python/autodog/timesync.py (design doc 2.8). Times are any one monotonic clock.
 */
class PoseBuffer(private val capacity: Int = 500, private val maxExtrapolation: Double = 0.05) {
    private class Sample(val t: Double, val x: Double, val y: Double, val theta: Double)

    private val samples = ArrayDeque<Sample>()

    val size: Int get() = samples.size

    /** Appends a pose. Samples that are not newer than the last one are dropped. */
    fun add(t: Double, x: Double, y: Double, theta: Double) {
        if (samples.isNotEmpty() && t <= samples.last().t) return
        if (samples.size == capacity) samples.removeFirst()
        samples.addLast(Sample(t, x, y, theta))
    }

    fun latest(): TrackerPose? = samples.lastOrNull()?.let { TrackerPose(it.x, it.y, it.theta) }

    /**
     * Pose at time t, or null when the buffer cannot answer: empty, t older than
     * everything held, or t more than maxExtrapolation past the newest sample.
     * Between samples it interpolates (heading the short way around). Just past the
     * newest sample it extrapolates along the last two, because holding the newest
     * pose would lag a turning robot and smear every ball sideways.
     */
    fun at(t: Double): TrackerPose? {
        if (samples.isEmpty()) return null
        if (t < samples.first().t) return null
        val a: Sample
        val b: Sample
        if (t >= samples.last().t) {
            if (t - samples.last().t > maxExtrapolation) return null
            if (samples.size == 1) return latest()
            a = samples[samples.size - 2]
            b = samples[samples.size - 1]
        } else {
            // first index whose time is greater than t
            var lo = 0
            var hi = samples.size
            while (lo < hi) {
                val mid = (lo + hi) ushr 1
                if (samples[mid].t <= t) lo = mid + 1 else hi = mid
            }
            a = samples[lo - 1]
            b = samples[lo]
        }
        val u = (t - a.t) / (b.t - a.t)
        val dth = wrapAngle(b.theta - a.theta)
        return TrackerPose(a.x + u * (b.x - a.x), a.y + u * (b.y - a.y), wrapAngle(a.theta + u * dth))
    }

    companion object {
        /** Wraps to (-pi, pi]. */
        fun wrapAngle(a: Double) = atan2(sin(a), cos(a))
    }
}

/** Gates applied to a projected detection in field coordinates (design doc 2.5). */
object Gating {
    /** Even-odd rule. poly is a list of [x, y] vertices. */
    fun pointInPolygon(x: Double, y: Double, poly: List<DoubleArray>): Boolean {
        var inside = false
        val n = poly.size
        for (i in 0 until n) {
            val x1 = poly[i][0]
            val y1 = poly[i][1]
            val x2 = poly[(i + 1) % n][0]
            val y2 = poly[(i + 1) % n][1]
            if ((y1 > y) != (y2 > y)) {
                val xCross = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
                if (x < xCross) inside = !inside
            }
        }
        return inside
    }

    fun distanceToPolygonEdge(x: Double, y: Double, poly: List<DoubleArray>): Double {
        var best = Double.POSITIVE_INFINITY
        val n = poly.size
        for (i in 0 until n) {
            val ax = poly[i][0]
            val ay = poly[i][1]
            val dx = poly[(i + 1) % n][0] - ax
            val dy = poly[(i + 1) % n][1] - ay
            val seg2 = dx * dx + dy * dy
            val t = if (seg2 == 0.0) 0.0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg2))
            best = min(best, hypot(x - (ax + t * dx), y - (ay + t * dy)))
        }
        return best
    }

    /** Inside poly grown outward by margin. */
    fun inExpandedPolygon(x: Double, y: Double, poly: List<DoubleArray>, margin: Double) =
        pointInPolygon(x, y, poly) || (margin > 0 && distanceToPolygonEdge(x, y, poly) <= margin)

    /** The robot footprint rectangle in the field frame. */
    fun robotFootprint(cfg: TrackerConfig, pose: TrackerPose): List<DoubleArray> {
        val (xMin, xMax, yMin, yMax) = cfg.robotFootprint
        val c = cos(pose.theta)
        val s = sin(pose.theta)
        return listOf(xMin to yMin, xMax to yMin, xMax to yMax, xMin to yMax).map { (px, py) ->
            doubleArrayOf(pose.x + c * px - s * py, pose.y + s * px + c * py)
        }
    }

    /** Null when the point passes every gate, else the gate it failed. */
    fun gate(x: Double, y: Double, cfg: TrackerConfig, pose: TrackerPose): Rejection? {
        val m = cfg.fieldMargin
        if (cfg.gateField && !(x >= m && x <= cfg.fieldWidth - m && y >= m && y <= cfg.fieldHeight - m)) {
            return Rejection.FIELD
        }
        for (poly in cfg.rampPolygons) {
            if (inExpandedPolygon(x, y, poly, cfg.rampMargin)) return Rejection.RAMP
        }
        if (pointInPolygon(x, y, robotFootprint(cfg, pose))) return Rejection.ROBOT
        return null
    }
}

/**
 * The per-frame tracker tick and target selection. Port of
 * python/autodog/tracker.py (design doc 2.9); BallTrackerGoldenTest replays a
 * recorded Python run and checks this class produces the same tracks and targets.
 * Not thread safe: call everything from the OpMode loop.
 */
class BallTracker(
    private val intrinsics: CameraIntrinsics,
    private val mount: CameraMount,
    val cfg: TrackerConfig = TrackerConfig.biobuzz(),
    val poses: PoseBuffer = PoseBuffer(),
) {
    private val projectors = HashMap<Double, Projector>()
    private val colorVotes = HashMap<Int, LinkedHashMap<Char, Int>>()
    private var nextId = 1
    var tracks: List<BallTrack> = emptyList()
        private set
    var targetId: Int? = null
        private set

    private fun projectorFor(color: Char?): Projector {
        val r = color?.let { cfg.ballRadii[it] } ?: cfg.ballRadius
        return projectors.getOrPut(r) { Projector(intrinsics, mount, r, cfg.maxRange) }
    }

    private fun vote(trackId: Int, color: Char?) {
        if (color == null) return
        val votes = colorVotes.getOrPut(trackId) { LinkedHashMap() }
        votes[color] = (votes[color] ?: 0) + 1
    }

    /** The color most often seen for a track (the first seen wins a tie). */
    fun colorOf(trackId: Int): Char? = colorVotes[trackId]?.maxByOrNull { it.value }?.key

    fun confirmedTracks(): List<BallTrack> = tracks.filter { it.confirmed(cfg.confirmHits) }

    /**
     * Processes one frame captured at time t. pose is the robot pose at capture;
     * when omitted it is looked up in the pose buffer.
     */
    fun tick(t: Double, detections: List<PixelDetection>, pose: TrackerPose? = poses.at(t)): TickResult {
        val result = TickResult(t, pose)
        if (pose == null) {
            // No trustworthy pose for this frame. Projecting with a stale pose would
            // smear every ball in the direction of motion, so coast instead.
            for (track in tracks) track.predict(t, cfg.accelSigma)
            result.target = target(t, null)
            return result
        }

        // 1-4: project each detection to the field and gate it
        val kept = ArrayList<Pair<Projection, Char?>>()
        for (det in detections) {
            val projector = projectorFor(det.color)
            projector.setPose(pose.x, pose.y, pose.theta)
            val proj = projector.project(
                det.u, det.v, cfg.pixelSigma, cfg.jacobianStepPx, cfg.poseSigmaXy, cfg.poseSigmaTheta)
            if (proj == null) {
                result.rejected[Rejection.PROJECTION.ordinal]++
                continue
            }
            val why = Gating.gate(proj.x, proj.y, cfg, pose)
            if (why != null) {
                result.rejected[why.ordinal]++
                continue
            }
            kept.add(proj to det.color)
            result.accepted.add(proj)
        }

        // 5: predict every track to the frame time
        for (track in tracks) track.predict(t, cfg.accelSigma)

        // 6: associate. Gate at chi^2, then claim pairs cheapest first.
        class Pair3(val d2: Double, val track: Int, val det: Int)
        val pairs = ArrayList<Pair3>()
        for (i in tracks.indices) for (j in kept.indices) {
            val p = kept[j].first
            val d2 = tracks[i].mahalanobis2(p.x, p.y, p.cov)
            if (d2 <= cfg.gateChi2) pairs.add(Pair3(d2, i, j))
        }
        pairs.sortWith(compareBy<Pair3> { it.d2 }.thenBy { it.track }.thenBy { it.det })
        val trackUsed = BooleanArray(tracks.size)
        val detUsed = BooleanArray(kept.size)
        for (pair in pairs) {
            if (trackUsed[pair.track] || detUsed[pair.det]) continue
            trackUsed[pair.track] = true
            detUsed[pair.det] = true
            // 7: update matched tracks
            val track = tracks[pair.track]
            val p = kept[pair.det].first
            track.update(p.x, p.y, p.cov)
            track.ageFrames++
            vote(track.id, kept[pair.det].second)
            result.matches++
        }

        // 9: coast unmatched tracks
        for (i in tracks.indices) {
            if (!trackUsed[i]) {
                tracks[i].framesUnseen++
                tracks[i].ageFrames++
            }
        }

        // 8: spawn a track for each unmatched detection
        val all = ArrayList(tracks)
        for (j in kept.indices) {
            if (detUsed[j]) continue
            val p = kept[j].first
            val track = BallTrack(nextId++, p.x, p.y, p.cov, t, cfg.initVelocityVar)
            all.add(track)
            vote(track.id, kept[j].second)
            result.spawned++
        }

        // 10: delete stale tracks, and tracks under the robot (a ball just intaked)
        val footprint = if (cfg.dropTracksUnderRobot) Gating.robotFootprint(cfg, pose) else null
        val survivors = ArrayList<BallTrack>()
        for (track in all) {
            val limit = if (track.confirmed(cfg.confirmHits)) cfg.maxFramesUnseen else cfg.tentativeMaxFramesUnseen
            val stale = track.framesUnseen > limit
            val lost = track.positionTrace > cfg.maxPositionTrace
            val eaten = footprint != null && Gating.pointInPolygon(track.x[0], track.x[1], footprint)
            if (stale || lost || eaten) {
                result.deleted++
                colorVotes.remove(track.id)
            } else {
                survivors.add(track)
            }
        }
        tracks = survivors

        // 11: select the target
        result.target = target(t, pose)
        return result
    }

    /**
     * The closest confirmed track, predicted to time t. Keeps the current target
     * unless another is closer by more than cfg.targetSwitchMargin. With no pose
     * given it uses the buffer's pose at t, or failing that its newest pose.
     */
    fun target(t: Double, pose: TrackerPose? = null, color: Char? = null): ChaseTarget? {
        val robot = pose ?: poses.at(t) ?: poses.latest() ?: return null
        class Option(val dist: Double, val track: BallTrack, val x: DoubleArray)
        val options = ArrayList<Option>()
        for (track in confirmedTracks()) {
            if (color != null && colorOf(track.id) != color) continue
            val (x, _) = track.predicted(t, cfg.accelSigma)
            options.add(Option(hypot(x[0] - robot.x, x[1] - robot.y), track, x))
        }
        if (options.isEmpty()) {
            targetId = null
            return null
        }
        var best = options.minByOrNull { it.dist }!!
        val current = options.firstOrNull { it.track.id == targetId }
        if (current != null && current.dist - best.dist <= cfg.targetSwitchMargin) best = current
        targetId = best.track.id
        return ChaseTarget(
            best.track.id, t, best.x[0], best.x[1], best.x[2], best.x[3], colorOf(best.track.id), best.dist)
    }
}
