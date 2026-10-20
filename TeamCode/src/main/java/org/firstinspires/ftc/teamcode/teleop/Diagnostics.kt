package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.hardware.SafeHardware

/** Tests the blocker servo without running the shooter or intake. Remove game pieces first. */
@TeleOp(name = "Blocker Diagnostic", group = "Test")
class BlockerDiagnosticTeleOp : OpMode() {
    private lateinit var blocker: Servo

    override fun init() {
        blocker = hardwareMap.get(Servo::class.java, "blocker")
        blocker.position = CLOSED
    }

    override fun loop() {
        when {
            gamepad1.a -> blocker.position = CLOSED
            gamepad1.b -> blocker.position = OPEN
        }
        telemetry.addData("A: close / B: open", "%.2f", blocker.position)
        telemetry.update()
    }

    override fun stop() {
        if (::blocker.isInitialized) blocker.position = CLOSED
    }

    companion object {
        private const val CLOSED = 0.24
        private const val OPEN = 0.45
    }
}

/** Direct motor test: no PID, intake, or automatic blocker movement. Clear the wheels first. */
@TeleOp(name = "Shooter Motor Diagnostic", group = "Test")
class ShooterMotorDiagnosticTeleOp : OpMode() {
    private lateinit var shooter1: DcMotorEx
    private lateinit var shooter2: DcMotorEx
    private lateinit var blocker: Servo
    private val safety = SafeHardware()
    private var initialPosition1 = 0
    private var initialPosition2 = 0

    override fun init() = guarded {
        val hubs = hardwareMap.getAll(LynxModule::class.java)
        safety.setCaching(hubs, LynxModule.BulkCachingMode.OFF)
        hubs.forEach { it.clearBulkCache() }
        blocker = safety.closeBlocker(hardwareMap, 0.24)
        shooter1 = safety.motor(hardwareMap, "shooter1")
        shooter2 = safety.motor(hardwareMap, "shooter2")
        // Reverse both motor polarities while preserving logical forward/reverse controls.
        shooter1.direction = DcMotorSimple.Direction.FORWARD
        shooter2.direction = DcMotorSimple.Direction.REVERSE
        shooter1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        shooter2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        shooter1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        shooter2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        shooter1.power = 0.0
        shooter2.power = 0.0
        initialPosition1 = shooter1.currentPosition
        initialPosition2 = shooter2.currentPosition
        blocker.position = 0.24
    }

    override fun loop() = guarded {
        // Hold X or Y for one motor, B for both. Right trigger raises power to 0.80.
        val power = 0.25 + 0.55 * gamepad1.right_trigger.toDouble().coerceIn(0.0, 1.0)
        shooter1.power = if (gamepad1.x || gamepad1.b) power else 0.0
        shooter2.power = if (gamepad1.y || gamepad1.b) power else 0.0
        blocker.position = 0.24

        telemetry.addLine("Hold X: motor 1 | Y: motor 2 | B: both | RT: raise power")
        telemetry.addData("Power 1 / 2", "%.2f / %.2f", shooter1.power, shooter2.power)
        telemetry.addData("Raw encoder ticks/s 1 / 2", "%.0f / %.0f",
            shooter1.velocity, shooter2.velocity)
        val position1 = shooter1.currentPosition
        val position2 = shooter2.currentPosition
        telemetry.addData("Encoder position ticks 1 / 2", "%d / %d", position1, position2)
        telemetry.addData("Ticks moved since init 1 / 2", "%d / %d",
            position1.toLong() - initialPosition1, position2.toLong() - initialPosition2)
        telemetry.addData("Motor 1 connection", shooter1.connectionInfo)
        telemetry.addData("Motor 2 connection", shooter2.connectionInfo)
        telemetry.addData("Motor 1 profile", shooter1.motorType.name)
        telemetry.addData("Motor 2 profile", shooter2.motorType.name)
        telemetry.addLine("Bulk cache OFF: fresh reads. " +
            "Spinning + zero counts: check encoder cables/ports.")
        telemetry.addData("Configured ticks/rev 1 / 2", "%.0f / %.0f",
            shooter1.motorType.ticksPerRev, shooter2.motorType.ticksPerRev)
        telemetry.addData("Configured max ticks/s 1 / 2", "%.0f / %.0f",
            shooter1.motorType.achieveableMaxTicksPerSecond,
            shooter2.motorType.achieveableMaxTicksPerSecond)
        telemetry.update()
    }

    private inline fun guarded(action: () -> Unit) {
        try { action() } catch (failure: Throwable) {
            throw safety.stop(failure)!!
        }
    }

    override fun stop() { safety.stop(null)?.let { throw it } }
}
