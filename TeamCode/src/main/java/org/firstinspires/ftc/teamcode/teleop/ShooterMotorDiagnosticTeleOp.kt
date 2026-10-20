package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo

/** Direct motor test: no PID, intake, or automatic blocker movement. Clear the wheels first. */
@TeleOp(name = "Shooter Motor Diagnostic", group = "Test")
class ShooterMotorDiagnosticTeleOp : OpMode() {
    private lateinit var shooter1: DcMotorEx
    private lateinit var shooter2: DcMotorEx
    private lateinit var blocker: Servo
    private var started = false
    private var previousCacheModes = emptyMap<LynxModule, LynxModule.BulkCachingMode>()
    private var initialPosition1 = 0
    private var initialPosition2 = 0

    override fun init() {
        // A preceding NextFTC OpMode can leave MANUAL caching enabled. Bypass that cache
        // for this diagnostic so zero counts cannot be explained by a stale bulk read.
        previousCacheModes = hardwareMap.getAll(LynxModule::class.java)
            .associateWith { it.bulkCachingMode }
        previousCacheModes.keys.forEach {
            it.bulkCachingMode = LynxModule.BulkCachingMode.OFF
            it.clearBulkCache()
        }
        shooter1 = hardwareMap.get(DcMotorEx::class.java, "shooter1")
        shooter2 = hardwareMap.get(DcMotorEx::class.java, "shooter2")
        blocker = hardwareMap.get(Servo::class.java, "blocker")
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
        started = false
    }

    override fun start() {
        started = true
        blocker.position = 0.24
    }

    override fun loop() {
        if (!started) return
        // Hold X or Y for one motor, B for both. Right trigger raises power to 0.80.
        val power = 0.25 + 0.55 * gamepad1.right_trigger.toDouble().coerceIn(0.0, 1.0)
        shooter1.power = if (gamepad1.x || gamepad1.b) power else 0.0
        shooter2.power = if (gamepad1.y || gamepad1.b) power else 0.0
        blocker.position = 0.24

        telemetry.addLine("Hold X: motor 1 | Y: motor 2 | B: both | RT: raise power")
        telemetry.addData("Power 1 / 2", "%.2f / %.2f", shooter1.power, shooter2.power)
        telemetry.addData("Raw encoder ticks/s 1 / 2", "%.0f / %.0f", shooter1.velocity, shooter2.velocity)
        val position1 = shooter1.currentPosition
        val position2 = shooter2.currentPosition
        telemetry.addData("Encoder position ticks 1 / 2", "%d / %d", position1, position2)
        telemetry.addData("Ticks moved since init 1 / 2", "%d / %d",
            position1.toLong() - initialPosition1, position2.toLong() - initialPosition2)
        telemetry.addData("Motor 1 connection", shooter1.connectionInfo)
        telemetry.addData("Motor 2 connection", shooter2.connectionInfo)
        telemetry.addData("Motor 1 profile", shooter1.motorType.name)
        telemetry.addData("Motor 2 profile", shooter2.motorType.name)
        telemetry.addLine("Bulk cache OFF: fresh reads. Spinning + zero counts: check encoder cables/ports.")
        telemetry.addData("Configured ticks/rev 1 / 2", "%.0f / %.0f",
            shooter1.motorType.ticksPerRev, shooter2.motorType.ticksPerRev)
        telemetry.addData("Configured max ticks/s 1 / 2", "%.0f / %.0f",
            shooter1.motorType.achieveableMaxTicksPerSecond,
            shooter2.motorType.achieveableMaxTicksPerSecond)
        telemetry.update()
    }

    override fun stop() {
        shooter1.power = 0.0
        shooter2.power = 0.0
        if (started) blocker.position = 0.24
        started = false
        previousCacheModes.forEach { (hub, mode) ->
            hub.clearBulkCache()
            hub.bulkCachingMode = mode
        }
    }
}
