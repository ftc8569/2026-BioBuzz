package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.Subsytem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem

@TeleOp(name = "BioBuzz NextFTC Kotlin TeleOp")
class BioBuzzTeleOp : NextFTCOpMode() {
    private lateinit var driveFL: DcMotor
    private lateinit var driveFR: DcMotor
    private lateinit var driveBL: DcMotor
    private lateinit var driveBR: DcMotor
    private lateinit var intake: IntakeSubsystem
    private lateinit var shooter: ShooterSubsystem

    override fun onInit() {
        driveFL = hardwareMap.get(DcMotor::class.java, "driveFL")
        driveFR = hardwareMap.get(DcMotor::class.java, "driveFR")
        driveBL = hardwareMap.get(DcMotor::class.java, "driveBL")
        driveBR = hardwareMap.get(DcMotor::class.java, "driveBR")

        driveFL.direction = DcMotorSimple.Direction.REVERSE
        driveBL.direction = DcMotorSimple.Direction.REVERSE

        intake = IntakeSubsystem(hardwareMap)
        shooter = ShooterSubsystem(hardwareMap)
        addComponents(SubsystemComponent(intake, shooter), BulkReadComponent)
    }

    override fun onUpdate() {
        val drive = -gamepad1.left_stick_y.toDouble()
        val strafe = gamepad1.left_stick_x.toDouble()
        val turn = gamepad1.right_stick_x.toDouble()
        val denominator = maxOf(kotlin.math.abs(drive) + kotlin.math.abs(strafe) + kotlin.math.abs(turn), 1.0)

        driveFL.power = (drive + strafe + turn) / denominator
        driveFR.power = (drive - strafe - turn) / denominator
        driveBL.power = (drive - strafe + turn) / denominator
        driveBR.power = (drive + strafe - turn) / denominator

        val intakePower = when {
            gamepad1.a -> 0.0
            gamepad1.right_trigger > 0.05 -> gamepad1.right_trigger.toDouble()
            gamepad1.left_trigger > 0.05 -> -gamepad1.left_trigger.toDouble()
            else -> 0.0
        }
        intake.setPower(intakePower)

        val shooterPower = when {
            gamepad1.right_bumper -> 1.0
            gamepad1.left_bumper -> -1.0
            else -> 0.0
        }
        shooter.setFlywheelPower(shooterPower)

        if (gamepad1.b) shooter.setBlockerPosition(true)
        if (gamepad1.x) shooter.setBlockerPosition(false)

        telemetry.addData("Intake Power", intakePower)
        telemetry.addData("Shooter Power", shooterPower)
        telemetry.update()
    }
}
