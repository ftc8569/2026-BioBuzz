package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.bindings.BindingManager
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.components.Component
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
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
    private var shooterPreset = 0.80
    private var currentShooterPower = 0.0
    private var currentIntakePower = 0.0

    // NextFTC 1.0.0 runs component preUpdate before scheduled commands.
    // Update gamepad bindings here so selections apply in the same loop.
    private val bindingComponent = object : Component {
        override fun preUpdate() = BindingManager.update()
        override fun postStop() = BindingManager.reset()
    }

    override fun onInit() {
        drive = DriveSubsystem(hardwareMap)
        intake = IntakeSubsystem(hardwareMap)
        shooter = ShooterSubsystem(hardwareMap)
        addComponents(
            BulkReadComponent,
            bindingComponent,
            SubsystemComponent(drive, intake, shooter)
        )
    }

    override fun onStartButtonPressed() {
        Gamepads.gamepad2.dpadUp.whenBecomesTrue {
            shooterPreset = (shooterPreset + 0.02).coerceAtMost(1.0)
        }
        Gamepads.gamepad2.dpadDown.whenBecomesTrue {
            shooterPreset = (shooterPreset - 0.02).coerceAtLeast(0.0)
        }
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
        currentShooterPower = when {
            gamepad1.left_bumper -> -0.8
            gamepad1.right_bumper -> shooterPreset
            else -> 0.0
        }

        val shooterReady = if (currentShooterPower > 0.0) {
            shooter.updateShot(currentShooterPower)
        } else {
            shooter.setFlywheelPower(currentShooterPower)
            false
        }

        val manualIntakePower = when {
            gamepad1.right_trigger > 0.05 -> gamepad1.right_trigger.toDouble()
            gamepad1.left_trigger > 0.05 -> -gamepad1.left_trigger.toDouble()
            else -> 0.0
        }

        currentIntakePower = when {
            gamepad1.a -> 0.0
            currentShooterPower > 0.0 && !shooterReady -> 0.0
            manualIntakePower != 0.0 -> manualIntakePower
            currentShooterPower > 0.0 && shooterReady -> currentShooterPower
            currentShooterPower < 0.0 -> currentShooterPower
            else -> 0.0
        }

        intake.setPower(currentIntakePower)
    }

    override fun onUpdate() {
        telemetry.addData("Intake Power", currentIntakePower)
        telemetry.addData("Shooter preset (G2 up/down)", "%.2f", shooterPreset)
        telemetry.addData("Shooter requested power", "%.2f", currentShooterPower)
        telemetry.addData("Shooter motor power 1 / 2", "%.2f / %.2f", shooter.flywheelPower1, shooter.flywheelPower2)
        telemetry.addData("Shooter at speed / blocker open", shooter.atSpeed)
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
