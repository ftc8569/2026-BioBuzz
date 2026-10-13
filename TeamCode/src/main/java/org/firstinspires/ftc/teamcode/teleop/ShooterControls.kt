package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem.ControlMode

/** One button press per edge, even when a button is held through many robot loops. */
internal class ShooterControls {
    private var presetSteps = 33
    val preset: Double get() = presetSteps * 0.02
    var mode = ControlMode.VELOCITY_PID
        private set
    private var previousY = false
    private var previousUp = false
    private var previousDown = false

    fun update(y: Boolean, up: Boolean, down: Boolean) {
        if (y && !previousY) {
            mode = if (mode == ControlMode.MANUAL_POWER) ControlMode.VELOCITY_PID
                else ControlMode.MANUAL_POWER
        }
        if (up && !previousUp && !down) presetSteps = (presetSteps + 1).coerceAtMost(50)
        if (down && !previousDown && !up) presetSteps = (presetSteps - 1).coerceAtLeast(0)
        previousY = y
        previousUp = up
        previousDown = down
    }
}
