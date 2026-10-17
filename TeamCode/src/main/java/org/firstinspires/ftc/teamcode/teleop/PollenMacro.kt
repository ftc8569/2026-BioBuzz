package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.teamcode.Subsytem.DriveSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.FieldLocation
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * TeleOp movement macro that drives right along the semicircle's flat side,
 * then follows the upper half-circle back to the left.
 *
 * The macro stays idle until start() is called and update() is called each loop.
 */
class PollenMacro(
    val r: Double = 10.0,
    val loopAmount: Int = 8
) {
    private val waypoints = mutableListOf<FieldLocation>()
    private var nextWaypoint = 0
    private var targetHeading = 0.0

    var startingXInches: Double? = null
        private set
    var isRunning: Boolean = false
        private set

    init {
        require(r.isFinite() && r > 0.0) { "r must be a positive finite distance" }
        require(loopAmount >= 0) { "loopAmount cannot be negative" }
    }

    /** Captures the current pose and prepares the repeated right-then-left route. */
    fun start(currentLocation: FieldLocation) {
        require(
            currentLocation.xInches.isFinite() &&
                currentLocation.yInches.isFinite() &&
                currentLocation.headingRadians.isFinite()
        ) { "Starting pose must be finite" }

        startingXInches = currentLocation.xInches
        targetHeading = currentLocation.headingRadians
        waypoints.clear()
        nextWaypoint = 0

        var cycleStartX = currentLocation.xInches
        repeat(loopAmount) {
            val centerX = cycleStartX
            val y = currentLocation.yInches

            // First travel right along the flat side.
            waypoints += FieldLocation(centerX + r, y, targetHeading)

            // Then follow the upper semicircle from its right endpoint to its left.
            for (step in 1..ARC_STEPS) {
                val angle = Math.PI * step / ARC_STEPS
                waypoints += FieldLocation(
                    xInches = centerX + r * cos(angle),
                    yInches = y + r * sin(angle),
                    headingRadians = targetHeading
                )
            }
            cycleStartX -= r
        }

        isRunning = waypoints.isNotEmpty()
    }

    /**
     * Call once per TeleOp loop while active. Returns true while the macro is moving.
     * The field-relative corrections are converted to the robot-relative drive inputs.
     */
    fun update(currentLocation: FieldLocation, drive: DriveSubsystem): Boolean {
        if (!isRunning) return false

        while (nextWaypoint < waypoints.size) {
            val target = waypoints[nextWaypoint]
            val deltaX = target.xInches - currentLocation.xInches
            val deltaY = target.yInches - currentLocation.yInches
            val distance = hypot(deltaX, deltaY)
            if (distance > POSITION_TOLERANCE_INCHES) {
                val speed = (distance * POSITION_GAIN).coerceIn(MIN_SPEED, MAX_SPEED)
                val headingCos = cos(currentLocation.headingRadians)
                val headingSin = sin(currentLocation.headingRadians)
                val forward = speed * (headingCos * deltaX + headingSin * deltaY) / distance
                val strafe = speed * (headingSin * deltaX - headingCos * deltaY) / distance
                val headingError = wrapAngle(targetHeading - currentLocation.headingRadians)
                val turn = (-TURN_GAIN * headingError).coerceIn(-MAX_TURN, MAX_TURN)
                drive.drive(forward, strafe, turn)
                return true
            }
            nextWaypoint++
        }

        isRunning = false
        drive.stop()
        return false
    }

    fun cancel(drive: DriveSubsystem) {
        isRunning = false
        waypoints.clear()
        nextWaypoint = 0
        drive.stop()
    }

    private fun wrapAngle(angle: Double): Double {
        var wrapped = angle
        while (wrapped > Math.PI) wrapped -= 2.0 * Math.PI
        while (wrapped < -Math.PI) wrapped += 2.0 * Math.PI
        return wrapped
    }

    private companion object {
        const val ARC_STEPS = 16
        const val POSITION_TOLERANCE_INCHES = 1.0
        const val POSITION_GAIN = 0.06
        const val MIN_SPEED = 0.12
        const val MAX_SPEED = 0.35
        const val TURN_GAIN = 0.8
        const val MAX_TURN = 0.35
    }
}
