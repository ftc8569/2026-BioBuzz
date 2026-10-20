package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

/** Tests the blocker servo without running the shooter or intake. Remove game pieces first. */
@TeleOp(name = "Blocker Diagnostic", group = "Test")
class BlockerDiagnosticTeleOp : OpMode() {
    private lateinit var blocker: Servo
    private var started = false

    override fun init() {
        blocker = hardwareMap.get(Servo::class.java, "blocker")
        started = false
    }

    override fun start() {
        started = true
        blocker.position = CLOSED
    }

    override fun loop() {
        if (!started) return
        when {
            gamepad1.a -> blocker.position = CLOSED
            gamepad1.b -> blocker.position = OPEN
        }
        telemetry.addData("A: close / B: open", "%.2f", blocker.position)
        telemetry.update()
    }

    override fun stop() {
        if (started) blocker.position = CLOSED
        started = false
    }

    companion object {
        private const val CLOSED = 0.24
        private const val OPEN = 0.45
    }
}
