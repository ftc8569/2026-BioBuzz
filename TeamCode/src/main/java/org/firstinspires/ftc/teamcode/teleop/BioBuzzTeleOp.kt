package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.Subsytem.DriveSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem

@TeleOp(name = "BioBuzz NextFTC Kotlin TeleOp")
class BioBuzzTeleOp : NextFTCOpMode() {
    private lateinit var drive: DriveSubsystem
    private lateinit var intake: IntakeSubsystem
    private lateinit var shooter: ShooterSubsystem
    private var shooterPresetSteps = 40
    private val shooterPreset: Double get() = shooterPresetSteps * 0.02
    private var currentShooterPower = 0.0
    private var currentIntakePower = 0.0
    private var previousDpadUp = false
    private var previousDpadDown = false

    override fun onInit() {
        drive = DriveSubsystem(hardwareMap)
        intake = IntakeSubsystem(hardwareMap)
        shooter = ShooterSubsystem(hardwareMap)
        addComponents(
            BulkReadComponent,
            SubsystemComponent(drive, intake, shooter)
        )
    }

    override fun onStartButtonPressed() {
        LambdaCommand("Driver control")
            .setIsDone { false }
            .setUpdate { updateDrive() }
            .setStop { drive.stop() }
            .requires(drive)
            .schedule()

        LambdaCommand("Shooter and intake control")
            .setIsDone { false }
            .setUpdate { updateMechanisms() }
            .setStop {
                shooter.setFlywheelPower(0.0)
                intake.setPower(0.0)
            }
            .requires(shooter, intake)
            .schedule()
    }

    private fun updateDrive() {
        drive.drive(
            -gamepad1.left_stick_y.toDouble(),
            gamepad1.left_stick_x.toDouble(),
            gamepad1.right_stick_x.toDouble()
        )
    }

    private fun updateMechanisms() {
        val dpadUp = gamepad1.dpad_up || gamepad2.dpad_up
        val dpadDown = gamepad1.dpad_down || gamepad2.dpad_down
        if (dpadUp && !previousDpadUp && !dpadDown) {
            shooterPresetSteps = (shooterPresetSteps + 1).coerceAtMost(50)
        }
        if (dpadDown && !previousDpadDown && !dpadUp) {
            shooterPresetSteps = (shooterPresetSteps - 1).coerceAtLeast(0)
        }
        previousDpadUp = dpadUp
        previousDpadDown = dpadDown

        currentShooterPower = when {
            gamepad1.left_bumper -> -0.8
            gamepad1.right_bumper -> shooterPreset
            else -> 0.0
        }

        if (currentShooterPower > 0.0) {
            shooter.updateShot(currentShooterPower)
        } else {
            shooter.setFlywheelPower(currentShooterPower)
        }

        val manualIntakePower = when {
            gamepad1.right_trigger > 0.05 -> gamepad1.right_trigger.toDouble()
            gamepad1.left_trigger > 0.05 -> -gamepad1.left_trigger.toDouble()
            else -> 0.0
        }

        currentIntakePower = when {
            gamepad1.a -> 0.0
            currentShooterPower != 0.0 -> currentShooterPower
            manualIntakePower != 0.0 -> manualIntakePower
            else -> 0.0
        }

        intake.setPower(currentIntakePower)
    }

    override fun onUpdate() {
        telemetry.addData("Intake Power", currentIntakePower)
        telemetry.addData("Shooter preset (G1/G2 up/down)", "%.2f", shooterPreset)
        telemetry.addData("D-pad seen (up/down)", "%s / %s", gamepad1.dpad_up || gamepad2.dpad_up, gamepad1.dpad_down || gamepad2.dpad_down)
        telemetry.addData("Shooter requested power", "%.2f", currentShooterPower)
        telemetry.addData("Shooter motor power 1 / 2", "%.2f / %.2f", shooter.flywheelPower1, shooter.flywheelPower2)
        telemetry.addData("Shooter at speed / blocker open", shooter.atSpeed)
        telemetry.addData("Blocker commanded position", "%.2f", shooter.blockerCommandedPosition)
        telemetry.addData("Flywheel 1 ticks/s", "%.0f / %.0f", shooter.measuredSpeed1, shooter.targetSpeed1)
        telemetry.addData("Flywheel 2 ticks/s", "%.0f / %.0f", shooter.measuredSpeed2, shooter.targetSpeed2)
        telemetry.update()
    }

    override fun onStop() {
        drive.stop()
        intake.setPower(0.0)
        shooter.setFlywheelPower(0.0)
    }
}
