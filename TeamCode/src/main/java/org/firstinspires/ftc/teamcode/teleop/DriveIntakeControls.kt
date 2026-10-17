package org.firstinspires.ftc.teamcode.teleop

import kotlin.math.abs

/** Default-on collection assist, toggled once per left-stick button press. */
internal class DriveIntakeControls {
    var enabled = true
        private set
    private var previousButton = false

    fun update(button: Boolean) {
        if (button && !previousButton) enabled = !enabled
        previousButton = button
    }

    fun minimumPower(drivePower: Double): Double =
        if (enabled && abs(drivePower) > 0.5) 0.45 else 0.0
}
