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
    private val shooterControls = ShooterControls()
    private val shooterPreset: Double get() = shooterControls.preset
    private val shooterMode: ShooterSubsystem.ControlMode get() = shooterControls.mode
    private var currentShooterPower = 0.0
    private var currentIntakePower = 0.0

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
                shooter.setManualBlockerOpen(false)
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
        shooterControls.update(gamepad1.y, dpadUp, dpadDown)

        currentShooterPower = when {
            gamepad1.left_bumper -> -0.8
            gamepad1.right_bumper -> shooterPreset
            else -> 0.0
        }

        shooter.setControlMode(shooterMode)
        if (currentShooterPower > 0.0) {
            shooter.updateShot(currentShooterPower, shooterMode)
        } else {
            shooter.setFlywheelPower(currentShooterPower)
        }
        val manualBlockerOpen = gamepad1.b || gamepad2.b
        shooter.setManualBlockerOpen(manualBlockerOpen)

        val manualIntakePower = when {
            gamepad1.right_trigger > 0.05 -> gamepad1.right_trigger.toDouble()
            gamepad1.left_trigger > 0.05 -> -gamepad1.left_trigger.toDouble()
            else -> 0.0
        }

        currentIntakePower = FeedControls.intakePower(
            stop = gamepad1.a,
            manualPower = manualIntakePower,
            shooterPower = currentShooterPower,
            preset = shooterPreset,
            mode = shooterMode,
            feedReady = shooter.feedReady,
            manualBlockerOpen = manualBlockerOpen
        )

        intake.setPower(currentIntakePower)
    }

    override fun onUpdate() {
        telemetry.addData("Intake Power", currentIntakePower)
        telemetry.addData("Shooter preset (G1/G2 up/down)", "%.2f", shooterPreset)
        telemetry.addData("Shooter mode (G1 Y)", shooterMode)
        telemetry.addData("Shooter speed state", shooter.speedStatus)
        telemetry.addData("D-pad seen (up/down)", "%s / %s", gamepad1.dpad_up || gamepad2.dpad_up, gamepad1.dpad_down || gamepad2.dpad_down)
        telemetry.addData("Shooter requested fraction", "%.2f", currentShooterPower)
        telemetry.addData("Shooter motor power 1 / 2", "%.2f / %.2f", shooter.flywheelPower1, shooter.flywheelPower2)
        telemetry.addData("Manual blocker (G1/G2 B)", gamepad1.b || gamepad2.b)
        telemetry.addData("Blocker open", shooter.blockerOpen)
        telemetry.addData("Both flywheels at speed", shooter.atSpeed)
        telemetry.addData("Automatic feed ready", shooter.feedReady)
        telemetry.addData("Feed readiness", shooter.feedReadiness)
        telemetry.addData("Encoder feedback unavailable", shooter.encoderFault)
        telemetry.addData("Blocker status", shooter.blockerStatus)
        telemetry.addData("Blocker commanded position", "%.2f", shooter.blockerCommandedPosition)
        telemetry.addData("Full motor speed RPM", "%.0f", ShooterSubsystem.MAX_MOTOR_RPM)
        telemetry.addData("PID target RPM", "%.0f", shooter.targetRpm)
        telemetry.addData("Measured motor RPM 1 / 2", "%.0f / %.0f", shooter.measuredRpm1, shooter.measuredRpm2)
        telemetry.addData("RPM configuration valid", shooter.rpmConfigurationValid)
        telemetry.addData("Raw ticks/s 1 / 2", "%.0f / %.0f",
            shooter.measuredTicksPerSecond1, shooter.measuredTicksPerSecond2)
        telemetry.addData("Encoder position ticks 1 / 2", "%d / %d",
            shooter.encoderPosition1, shooter.encoderPosition2)
        telemetry.addData("PID target ticks/s 1 / 2", "%.0f / %.0f",
            shooter.targetTicksPerSecond1, shooter.targetTicksPerSecond2)
        telemetry.addData("Motor config ticks/rev 1 / 2", "%.0f / %.0f",
            shooter.configuredTicksPerRev1, shooter.configuredTicksPerRev2)
        telemetry.addData("Motor config max ticks/s 1 / 2", "%.0f / %.0f",
            shooter.configuredMaxTicksPerSecond1, shooter.configuredMaxTicksPerSecond2)
        telemetry.update()
    }

    override fun onStop() {
        drive.stop()
        intake.setPower(0.0)
        shooter.setManualBlockerOpen(false)
        shooter.setFlywheelPower(0.0)
    }
}
