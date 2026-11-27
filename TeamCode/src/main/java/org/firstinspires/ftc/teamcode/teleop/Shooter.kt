package org.firstinspires.ftc.teamcode.teleop

import dev.nextftc.bindings.button
import kotlin.math.roundToInt
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.Mode

/** Shooter buttons: hold manual for manual power; D-pad steps the preset by 0.005 (22.5 RPM). */
internal class ShooterControls {
    private var steps = (ShooterSubsystem.DEFAULT_SHOOTER_PRESET / 0.005).roundToInt()
    private var manualHeld: () -> Boolean = { false }
    val preset: Double get() = steps * 0.005
    val mode: Mode get() = if (manualHeld()) Mode.MANUAL else Mode.LQR

    /** NextFTC bindings: one step per press; pressing both directions does nothing. */
    fun bind(manual: () -> Boolean, up: () -> Boolean, down: () -> Boolean) {
        manualHeld = manual
        button { up() } whenBecomesTrue { if (!down()) steps = (steps + 1).coerceAtMost(200) }
        button { down() } whenBecomesTrue { if (!up()) steps = (steps - 1).coerceAtLeast(0) }
    }

    /** Reverse wins; then manual or LQR shooting at the preset; otherwise off. */
    fun requestedPower(reverse: Boolean, lqrShoot: Boolean): Double = when {
        reverse -> -0.8
        mode == Mode.MANUAL || lqrShoot -> preset
        else -> 0.0
    }
}
