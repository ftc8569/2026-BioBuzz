package org.firstinspires.ftc.teamcode.teleop

import dev.nextftc.bindings.button
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.subsystems.ShooterController.ControlMode

// Intake controls and feed timing; motors live in subsystems/IntakeSubsystem.kt.

/**
 * One feeding policy for TeleOp and every shooting auto, as on main:
 * - G1 A (stop) always wins; a reverse trigger is always allowed, to clear jams.
 * - While shooting, the intake only feeds once the shooter allows it (LQR at speed,
 *   or manual mode with the blocker held open), at [feedPower] or a stronger trigger.
 * - Forward intake waits 200 ms after the blocker opens, so balls never hit a closed blocker.
 * TeleOp and ShootCommand feed at 1.0; Basic, preload and MAIN autos feed at the shooter preset.
 */
class IntakeFeed(private val feedPower: Double, private val nanoTime: () -> Long = System::nanoTime) {
    private var blockerOpenedAt: Long? = null

    fun reset() { blockerOpenedAt = null }

    fun power(shooter: ShooterSubsystem, shooterPower: Double, stop: Boolean = false,
              manualPower: Double = 0.0, manualBlockerOpen: Boolean = false,
              minimumPower: Double = 0.0): Double {
        val requested = requestedPower(shooter, shooterPower, stop, manualPower,
            manualBlockerOpen, minimumPower)
        if (!shooter.control.blockerOpen) { blockerOpenedAt = null; return requested }
        val now = nanoTime()
        val opened = blockerOpenedAt ?: now.also { blockerOpenedAt = it }
        return if (requested > 0.0 && now - opened < BLOCKER_OPEN_DELAY_NANOS) 0.0 else requested
    }

    private fun requestedPower(shooter: ShooterSubsystem, shooterPower: Double, stop: Boolean,
                               manualPower: Double, manualBlockerOpen: Boolean,
                               minimumPower: Double): Double {
        if (stop) return 0.0
        val manual = if (manualPower.isFinite()) manualPower.coerceIn(-1.0, 1.0) else 0.0
        if (manual < 0.0) return manual
        val minimum = if (minimumPower.isFinite()) minimumPower.coerceIn(0.0, 1.0) else 0.0
        if (shooterPower <= 0.0) return maxOf(manual, minimum)
        val canFeed = shooter.control.feedReady ||
            (shooter.control.controlMode == ControlMode.MANUAL_POWER && manualBlockerOpen)
        if (!canFeed) return 0.0
        return maxOf(minimum, if (manual > 0.0) manual else feedPower)
    }

    companion object { const val BLOCKER_OPEN_DELAY_NANOS = 200_000_000L }
}

/** Default-on collection assist: driving above half power runs the intake at 0.45. */
internal class DriveIntakeControls {
    var enabled = true
        private set

    /** NextFTC binding: each press toggles the assist. */
    fun bind(pressed: () -> Boolean) { button { pressed() } whenBecomesTrue { enabled = !enabled } }

    fun minimumPower(drivePower: Double): Double =
        if (enabled && abs(drivePower) > 0.5) 0.45 else 0.0
}
