package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem.ControlMode
import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem
import kotlin.math.roundToInt

/** Hold Y for manual shooting; preset adjustments happen once per D-pad press. */
internal class ShooterControls {
    private var presetSteps = (ShooterSubsystem.DEFAULT_SHOOTER_PRESET / 0.005).roundToInt()
    val preset: Double get() = presetSteps * 0.005
    var mode = ControlMode.VELOCITY_PID
        private set
    val pidTargetRpm: Double?
        get() = if (mode == ControlMode.VELOCITY_PID)
            preset * ShooterSubsystem.MAX_MOTOR_RPM else null
    private var previousUp = false
    private var previousDown = false

    fun update(y: Boolean, up: Boolean, down: Boolean) {
        mode = if (y) ControlMode.MANUAL_POWER else ControlMode.VELOCITY_PID
        if (up && !previousUp && !down) presetSteps = (presetSteps + 1).coerceAtMost(200)
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
