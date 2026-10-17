package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem.ControlMode

/** Hold Y for manual shooting; preset adjustments happen once per D-pad press. */
internal class ShooterControls {
    private var presetSteps = 32
    val preset: Double get() = presetSteps * 0.02
    var mode = ControlMode.VELOCITY_PID
        private set
    private var previousUp = false
    private var previousDown = false

    fun update(y: Boolean, up: Boolean, down: Boolean) {
        mode = if (y) ControlMode.MANUAL_POWER else ControlMode.VELOCITY_PID
        if (up && !previousUp && !down) presetSteps = (presetSteps + 1).coerceAtMost(50)
        if (down && !previousDown && !up) presetSteps = (presetSteps - 1).coerceAtLeast(0)
        previousUp = up
        previousDown = down
    }

    fun requestedPower(reverse: Boolean, pidShoot: Boolean): Double = when {
        reverse -> -0.8
        mode == ControlMode.MANUAL_POWER || pidShoot -> preset
        else -> 0.0
    }
}
