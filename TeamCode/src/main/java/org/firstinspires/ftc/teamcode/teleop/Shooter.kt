package org.firstinspires.ftc.teamcode.teleop

import dev.nextftc.bindings.button
import kotlin.math.roundToInt
import org.firstinspires.ftc.teamcode.subsystems.ShooterController
import org.firstinspires.ftc.teamcode.subsystems.ShooterController.ControlMode

/** Shared match/tuner controls: hold Y for manual power, release to return to LQR.
 * One D-pad press changes 0.005 (22.5 RPM in LQR); reverse always wins.
 */
internal class ShooterControls {
    private var presetSteps = (ShooterController.DEFAULT_SHOOTER_PRESET / 0.005).roundToInt()
    private var manualHeld: () -> Boolean = { false }
    val preset: Double get() = presetSteps * 0.005
    val mode: ControlMode get() = if (manualHeld()) ControlMode.MANUAL_POWER else ControlMode.VELOCITY_LQR
    val velocityTargetRpm: Double?
        get() = if (mode == ControlMode.VELOCITY_LQR) preset * ShooterController.MAX_MOTOR_RPM else null

    /** NextFTC bindings: one preset step per press; pressing both D-pad directions does nothing. */
    fun bind(manual: () -> Boolean, up: () -> Boolean, down: () -> Boolean) {
        manualHeld = manual
        button { up() } whenBecomesTrue { if (!down()) step(1) }
        button { down() } whenBecomesTrue { if (!up()) step(-1) }
    }

    private fun step(by: Int) { presetSteps = (presetSteps + by).coerceIn(0, 200) }

    fun requestedPower(reverse: Boolean, velocityShoot: Boolean): Double = when {
        reverse -> -0.8
        mode == ControlMode.MANUAL_POWER || velocityShoot -> preset
        else -> 0.0
    }
}
