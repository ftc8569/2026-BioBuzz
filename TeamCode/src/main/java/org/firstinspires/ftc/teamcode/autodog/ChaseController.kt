package org.firstinspires.ftc.teamcode.autodog

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sin

/** Robot-relative drive powers in [-1, 1]: forward, left, counterclockwise turn. */
data class DrivePower(val forward: Double, val left: Double, val turn: Double) {
    companion object {
        val STOP = DrivePower(0.0, 0.0, 0.0)
    }
}

/**
 * Turns toward a field-frame target and drives onto it with the intake in
 * front. Same law as the Python simulator (python/autodog/sim.py), which
 * collects every floor ball in testing:
 *
 *   turn    = clamp(kTurn * headingError)
 *   forward = min(maxForward, kForward * distance) * max(0, cos(headingError))^3
 *
 * The cos^3 term means the robot turns in place when the ball is off to the
 * side and only drives once roughly lined up, so it does not swing wide.
 * A small strafe term (mecanum) trims sideways error near the ball.
 */
class ChaseController(
    var kTurn: Double = 1.6,
    var kForward: Double = 1.2,
    var kStrafe: Double = 1.5,
    var maxForward: Double = 0.5,
    var maxStrafe: Double = 0.3,
    var maxTurn: Double = 0.5,
    /** Distance from robot center to the intake mouth (m). */
    var intakeOffset: Double = 0.20,
    /** Only strafe once the ball is this close (m). */
    var strafeRadius: Double = 0.6,
) {
    fun compute(robotX: Double, robotY: Double, robotTheta: Double, targetX: Double, targetY: Double): DrivePower {
        val dx = targetX - robotX
        val dy = targetY - robotY
        // target in the robot frame (x forward, y left)
        val c = cos(robotTheta)
        val s = sin(robotTheta)
        val fx = c * dx + s * dy
        val fy = -s * dx + c * dy
        val distance = hypot(fx, fy)
        val headingError = atan2(fy, fx)

        val turn = clamp(kTurn * headingError, maxTurn)
        val align = max(0.0, cos(headingError)).pow(3)
        val toIntake = max(0.0, distance - intakeOffset)
        val forward = min(maxForward, kForward * (toIntake + 0.1)) * align
        val left = if (distance < strafeRadius && abs(headingError) < PI / 3) {
            clamp(kStrafe * fy, maxStrafe)
        } else {
            0.0
        }
        return DrivePower(forward, left, turn)
    }

    private fun clamp(v: Double, limit: Double) = v.coerceIn(-limit, limit)
}
