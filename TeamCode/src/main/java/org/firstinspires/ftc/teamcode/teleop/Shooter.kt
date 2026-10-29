package org.firstinspires.ftc.teamcode.teleop

import kotlin.math.roundToInt
import org.firstinspires.ftc.teamcode.subsystems.ShooterController
import org.firstinspires.ftc.teamcode.subsystems.ShooterController.ControlMode

/** Shared match/tuner controls from manual-shooter: hold Y, release to return to LQR.
 * One D-pad press changes 0.005 (22.5 RPM in LQR); reverse always wins.
 */
internal class ShooterControls {
    private var presetSteps = (ShooterController.DEFAULT_SHOOTER_PRESET / 0.005).roundToInt()
    val preset: Double get() = presetSteps * 0.005
    var mode = ControlMode.VELOCITY_LQR
        private set
    val velocityTargetRpm: Double?
        get() = if (mode == ControlMode.VELOCITY_LQR)
            preset * ShooterController.MAX_MOTOR_RPM else null
    private val upButton = PressEdge()
    private val downButton = PressEdge()

    fun update(y: Boolean, up: Boolean, down: Boolean) {
        mode = if (y) ControlMode.MANUAL_POWER else ControlMode.VELOCITY_LQR
        val increase = upButton.pressed(up)
        val decrease = downButton.pressed(down)
        val step = if (increase && !down) 1 else if (decrease && !up) -1 else 0
        presetSteps = (presetSteps + step).coerceIn(0, 200)
    }

    fun requestedPower(reverse: Boolean, velocityShoot: Boolean): Double = when {
        reverse -> -0.8
        mode == ControlMode.MANUAL_POWER || velocityShoot -> preset
        else -> 0.0
    }
}
