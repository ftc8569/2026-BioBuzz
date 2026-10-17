package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.hardware.Gamepad

/** Only controls used by this OpMode; copied once before controllers execute. */
internal data class ControlSnapshot(
    val x: Boolean = false, val y: Boolean = false, val a: Boolean = false, val b: Boolean = false,
    val dpad_up: Boolean = false, val dpad_down: Boolean = false,
    val left_bumper: Boolean = false, val right_bumper: Boolean = false,
    val left_stick_button: Boolean = false, val right_stick_button: Boolean = false,
    val left_stick_x: Float = 0f, val left_stick_y: Float = 0f, val right_stick_x: Float = 0f,
    val left_trigger: Float = 0f, val right_trigger: Float = 0f
) {
    companion object {
        fun capture(g: Gamepad) = ControlSnapshot(
            g.x, g.y, g.a, g.b, g.dpad_up, g.dpad_down, g.left_bumper, g.right_bumper,
            g.left_stick_button, g.right_stick_button, g.left_stick_x, g.left_stick_y,
            g.right_stick_x, g.left_trigger, g.right_trigger)
    }
}
