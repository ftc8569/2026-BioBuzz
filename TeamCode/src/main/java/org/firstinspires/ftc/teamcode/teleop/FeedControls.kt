package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem.ControlMode

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
