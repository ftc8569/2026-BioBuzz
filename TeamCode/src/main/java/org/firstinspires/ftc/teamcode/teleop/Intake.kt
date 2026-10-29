package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.subsystems.ShooterController.ControlMode

// Intake controls and feed timing; motors live in subsystems/IntakeSubsystem.kt.

/** Resolve feeding every loop; a trigger cannot bypass RPM readiness in LQR shooting. */
internal object FeedControls {
    fun intakePower(stop: Boolean, manualPower: Double, shooterPower: Double,
                    preset: Double, mode: ControlMode, feedReady: Boolean,
                    manualBlockerOpen: Boolean, minimumPower: Double = 0.0): Double {
        if (stop) return 0.0
        val manual = if (manualPower.isFinite()) manualPower.coerceIn(-1.0, 1.0) else 0.0
        // Reverse remains available to clear a jam.
        if (manual < 0.0) return manual
        val minimum = if (minimumPower.isFinite()) minimumPower.coerceIn(0.0, 1.0) else 0.0
        if (shooterPower > 0.0) {
            val canFeed = feedReady ||
                (mode == ControlMode.MANUAL_POWER && manualBlockerOpen)
            if (!canFeed) return 0.0
            return maxOf(minimum, if (manual > 0.0) manual else preset.coerceIn(0.0, 1.0))
        }
        // Ordinary collection is available with the shooter off.
        return maxOf(manual, minimum)
    }
}

/** Allow the blocker 100 ms to open before applying forward intake power. */
internal class BlockerFeedDelay(private val nanoTime: () -> Long = System::nanoTime) {
    private var openedAt: Long? = null

    fun apply(power: Double, blockerOpen: Boolean): Double {
        if (!blockerOpen) {
            openedAt = null
            return power
        }
        val now = nanoTime()
        val start = openedAt ?: now.also { openedAt = it }
        return if (power > 0.0 && now - start < 100_000_000L) 0.0 else power
    }
}

/** One feeding policy for TeleOp and every shooting auto. */
class IntakeFeed @JvmOverloads constructor(private val nanoTime: () -> Long = System::nanoTime) {
    private var delay = BlockerFeedDelay(nanoTime)

    fun reset() { delay = BlockerFeedDelay(nanoTime) }

    @JvmOverloads
    fun power(shooter: ShooterSubsystem, shooterPower: Double, stop: Boolean = false,
              manualPower: Double = 0.0, manualBlockerOpen: Boolean = false,
              minimumPower: Double = 0.0): Double {
        val requested = FeedControls.intakePower(stop, manualPower, shooterPower,
            FEED_POWER, shooter.control.controlMode, shooter.control.feedReady,
            manualBlockerOpen, minimumPower)
        return delay.apply(requested, shooter.control.blockerOpen)
    }

    companion object { const val FEED_POWER = 0.6 }
}

/** One event per press; releasing arms the next event. */
internal class PressEdge {
    private var previous = false
    fun pressed(down: Boolean): Boolean {
        val rising = down && !previous
        previous = down
        return rising
    }
}

/** Default-on collection assist, toggled once per left-stick button press. */
internal class DriveIntakeControls {
    var enabled = true
        private set
    private val button = PressEdge()

    fun update(button: Boolean) {
        if (this.button.pressed(button)) enabled = !enabled
    }

    fun minimumPower(drivePower: Double): Double =
        if (enabled && abs(drivePower) > 0.5) 0.45 else 0.0
}
