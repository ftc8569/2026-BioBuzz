package org.firstinspires.ftc.teamcode.autodog

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Replays a simulated chase recorded from the Python tracker
 * (BioBuzzAutoDog: python -m autodog.golden) and checks the Kotlin port agrees
 * after every frame: same gate decisions, same tracks, same targets.
 *
 * If the Python tracker changes, regenerate src/test/resources/autodog/tracker_golden.txt
 * and port the change here.
 */
class BallTrackerGoldenTest {
    private val tol = 1e-7

    private fun lines(): List<List<String>> {
        val stream = javaClass.getResourceAsStream("/autodog/tracker_golden.txt")
        assertNotNull("tracker_golden.txt missing from test resources", stream)
        return stream!!.bufferedReader().readLines()
            .filter { it.isNotBlank() && !it.startsWith("#") }
            .map { it.trim().split(" ") }
    }

    private fun color(token: String): Char? = if (token == "-") null else token[0]

    private fun checkTarget(where: String, expected: List<String>, actual: ChaseTarget?) {
        if (expected[0] == "none") {
            assertNull("$where: expected no target", actual)
            return
        }
        assertNotNull("$where: expected a target", actual)
        actual!!
        assertEquals("$where target id", expected[0].toInt(), actual.trackId)
        assertEquals("$where target x", expected[1].toDouble(), actual.x, tol)
        assertEquals("$where target y", expected[2].toDouble(), actual.y, tol)
        assertEquals("$where target vx", expected[3].toDouble(), actual.vx, tol)
        assertEquals("$where target vy", expected[4].toDouble(), actual.vy, tol)
        assertEquals("$where target color", color(expected[5]), actual.color)
        assertEquals("$where target distance", expected[6].toDouble(), actual.distance, tol)
    }

    @Test
    fun `kotlin tracker reproduces the python tracker on a recorded chase`() {
        val all = lines()
        var intrinsics: CameraIntrinsics? = null
        var mount: CameraMount? = null
        val cfg = TrackerConfig()
        val radii = HashMap<Char, Double>()
        val ramps = ArrayList<List<DoubleArray>>()
        var i = 0
        header@ while (i < all.size) {
            val f = all[i]
            when (f[0]) {
                "INTR" -> intrinsics = CameraIntrinsics(
                    f[1].toDouble(), f[2].toDouble(), f[3].toDouble(), f[4].toDouble(),
                    DoubleArray(5) { f[5 + it].toDouble() }, f[10].toInt(), f[11].toInt())
                "MOUNT" -> mount = CameraMount(
                    f[1].toDouble(), f[2].toDouble(), f[3].toDouble(),
                    f[4].toDouble(), f[5].toDouble(), f[6].toDouble())
                "RADIUS" -> radii[f[1][0]] = f[2].toDouble()
                "RAMP" -> ramps.add((1 until f.size step 2).map { doubleArrayOf(f[it].toDouble(), f[it + 1].toDouble()) })
                "CFG" -> when (f[1]) {
                    "field_margin" -> cfg.fieldMargin = f[2].toDouble()
                    "ball_radius" -> cfg.ballRadius = f[2].toDouble()
                    "ramp_margin" -> cfg.rampMargin = f[2].toDouble()
                    "max_range" -> cfg.maxRange = f[2].toDouble()
                    "pixel_sigma" -> cfg.pixelSigma = f[2].toDouble()
                    "jacobian_step_px" -> cfg.jacobianStepPx = f[2].toDouble()
                    "pose_sigma_xy" -> cfg.poseSigmaXy = f[2].toDouble()
                    "pose_sigma_theta" -> cfg.poseSigmaTheta = f[2].toDouble()
                    "accel_sigma" -> cfg.accelSigma = f[2].toDouble()
                    "init_velocity_var" -> cfg.initVelocityVar = f[2].toDouble()
                    "gate_chi2" -> cfg.gateChi2 = f[2].toDouble()
                    "max_frames_unseen" -> cfg.maxFramesUnseen = f[2].toInt()
                    "tentative_max_frames_unseen" -> cfg.tentativeMaxFramesUnseen = f[2].toInt()
                    "max_position_trace" -> cfg.maxPositionTrace = f[2].toDouble()
                    "confirm_hits" -> cfg.confirmHits = f[2].toInt()
                    "target_switch_margin" -> cfg.targetSwitchMargin = f[2].toDouble()
                    "field_size" -> {
                        cfg.fieldWidth = f[2].toDouble()
                        cfg.fieldHeight = f[3].toDouble()
                    }
                    "robot_footprint" -> cfg.robotFootprint = DoubleArray(4) { f[2 + it].toDouble() }
                    "gate_field" -> cfg.gateField = f[2] == "1"
                    "drop_tracks_under_robot" -> cfg.dropTracksUnderRobot = f[2] == "1"
                    else -> error("unknown CFG ${f[1]}")
                }
                else -> break@header
            }
            i++
        }
        cfg.ballRadii = radii
        cfg.rampPolygons = ramps
        val tracker = BallTracker(intrinsics!!, mount!!, cfg)

        var ticks = 0
        var targets = 0
        var tracksChecked = 0
        var result: TickResult? = null
        var trackRow = 0
        var where = ""
        while (i < all.size) {
            val f = all[i++]
            when (f[0]) {
                "POSE" -> tracker.poses.add(f[1].toDouble(), f[2].toDouble(), f[3].toDouble(), f[4].toDouble())
                "TICK" -> {
                    val t = f[1].toDouble()
                    val dets = (0 until f[2].toInt()).map {
                        val d = all[i++]
                        assertEquals("DET", d[0])
                        PixelDetection(d[1].toDouble(), d[2].toDouble(), color(d[3]))
                    }
                    where = "tick ${ticks++} (t=$t)"
                    result = tracker.tick(t, dets)
                    trackRow = 0
                }
                "RES" -> {
                    val r = result!!
                    assertEquals("$where accepted", f[1].toInt(), r.accepted.size)
                    assertEquals("$where rejected projection", f[2].toInt(), r.rejected[Rejection.PROJECTION.ordinal])
                    assertEquals("$where rejected field", f[3].toInt(), r.rejected[Rejection.FIELD.ordinal])
                    assertEquals("$where rejected ramp", f[4].toInt(), r.rejected[Rejection.RAMP.ordinal])
                    assertEquals("$where rejected robot", f[5].toInt(), r.rejected[Rejection.ROBOT.ordinal])
                    assertEquals("$where matches", f[6].toInt(), r.matches)
                    assertEquals("$where spawned", f[7].toInt(), r.spawned)
                    assertEquals("$where deleted", f[8].toInt(), r.deleted)
                    assertEquals("$where track count", f[9].toInt(), tracker.tracks.size)
                }
                "TRK" -> {
                    val track = tracker.tracks[trackRow++]
                    val w = "$where track ${f[1]}"
                    assertEquals("$w id", f[1].toInt(), track.id)
                    for (k in 0 until 4) assertEquals("$w state $k", f[2 + k].toDouble(), track.x[k], tol)
                    assertEquals("$w pxx", f[6].toDouble(), track.p[0], tol)
                    assertEquals("$w pyy", f[7].toDouble(), track.p[5], tol)
                    assertEquals("$w hits", f[8].toInt(), track.hits)
                    assertEquals("$w unseen", f[9].toInt(), track.framesUnseen)
                    tracksChecked++
                }
                "TTGT" -> checkTarget("$where tick", f.drop(1), result!!.target)
                "TGT" -> {
                    checkTarget("$where query", f.drop(2), tracker.target(f[1].toDouble()))
                    if (f[2] != "none") targets++
                }
                else -> error("unknown line ${f[0]}")
            }
        }
        // The recording must actually exercise the tracker.
        assertTrue("ticks $ticks", ticks > 300)
        assertTrue("tracks checked $tracksChecked", tracksChecked > 500)
        assertTrue("targets $targets", targets > 100)
    }
}
