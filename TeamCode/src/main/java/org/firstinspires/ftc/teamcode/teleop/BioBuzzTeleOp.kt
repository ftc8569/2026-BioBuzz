package org.firstinspires.ftc.teamcode.teleop

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.Subsytem.DriveSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.FieldLocation
import org.firstinspires.ftc.teamcode.Subsytem.RobotLocationSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem

@TeleOp(name = "BioBuzz NextFTC Kotlin TeleOp")
class BioBuzzTeleOp : NextFTCOpMode() {
    private lateinit var drive: DriveSubsystem
    private lateinit var intake: IntakeSubsystem
    private lateinit var shooter: ShooterSubsystem
    private lateinit var robotLocation: RobotLocationSubsystem
    private lateinit var driveMotors: List<DcMotor>
    private var previousDriveEncoderPositions: IntArray? = null
    private var locationInitialized = false
    private var previousXButtonPressed = false
    private var previousAimButtonPressed = false
    private var blueHiveHeading: Double? = null
    private var rotatingToBlueHive = false
    private val shooterControls = ShooterControls()
    private val shooterPreset: Double get() = shooterControls.preset
    private val shooterMode: ShooterSubsystem.ControlMode get() = shooterControls.mode
    private var currentShooterPower = 0.0
    private var currentIntakePower = 0.0

    override fun onInit() {
        telemetry = JoinedTelemetry(telemetry, PanelsTelemetry.ftcTelemetry)
        telemetry.setMsTransmissionInterval(50)
        drive = DriveSubsystem(hardwareMap)
        driveMotors = listOf("driveFL", "driveFR", "driveBL", "driveBR")
            .map { hardwareMap.get(DcMotor::class.java, it) }
        intake = IntakeSubsystem(hardwareMap)
        shooter = ShooterSubsystem(hardwareMap)
        robotLocation = RobotLocationSubsystem(hardwareMap)
        addComponents(
            BulkReadComponent,
            SubsystemComponent(drive, intake, shooter, robotLocation)
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
        if (!locationInitialized) {
            robotLocation.setStartingLocation(FieldLocation(0.0, 0.0, 0.0))
            locationInitialized = true
        }
        val positions = driveMotors.map { it.currentPosition }.toIntArray()
        val previousPositions = previousDriveEncoderPositions ?: positions
        telemetry.addData("Drive encoder delta (ticks)", "FL %d, FR %d, BL %d, BR %d",
            positions[0] - previousPositions[0], positions[1] - previousPositions[1],
            positions[2] - previousPositions[2], positions[3] - previousPositions[3])
        previousDriveEncoderPositions = positions
        val location = robotLocation.getLocation()
        if (gamepad1.x && !previousXButtonPressed) {
            robotLocation.updateHiveLocation()
            blueHiveHeading = null
            rotatingToBlueHive = false
        }
        previousXButtonPressed = gamepad1.x
        // Gamepad 1 Y is reserved for hold-to-run manual shooting.
        if (gamepad2.y && !previousAimButtonPressed) {
            blueHiveHeading = robotLocation.calculateRotationToBlueHive(location)
            rotatingToBlueHive = blueHiveHeading != null
        }
        previousAimButtonPressed = gamepad2.y
        val manualTurn = gamepad1.right_stick_x.toDouble()
        var turn = manualTurn
        var headingError: Double? = null
        if (kotlin.math.abs(manualTurn) > 0.05) {
            rotatingToBlueHive = false
        } else if (rotatingToBlueHive) {
            val targetHeading = blueHiveHeading
            if (targetHeading == null) {
                rotatingToBlueHive = false
            } else {
                headingError = wrapAngle(targetHeading - location.headingRadians)
                if (kotlin.math.abs(headingError) <= HEADING_TOLERANCE_RADIANS) {
                    rotatingToBlueHive = false
                } else {
                    val correction = (-TURN_GAIN * headingError)
                        .coerceIn(-MAX_AUTO_TURN, MAX_AUTO_TURN)
                    turn = if (kotlin.math.abs(correction) < MIN_AUTO_TURN)
                        kotlin.math.sign(correction) * MIN_AUTO_TURN else correction
                }
            }
        }
        drive.drive(
            -gamepad1.left_stick_y.toDouble(),
            gamepad1.left_stick_x.toDouble(),
            turn
        )
        telemetry.addData("Robot X (in)", location.xInches)
        telemetry.addData("Robot Y (in)", location.yInches)
        telemetry.addData("Robot Heading (rad)", location.headingRadians)
        telemetry.addData("Blue Hive Heading (rad)",
            blueHiveHeading ?: "Press G1 X to save hive, then G2 Y to aim")
        telemetry.addData("Turning to Blue Hive",
            if (rotatingToBlueHive) "Yes (right stick cancels)" else "No")
        headingError?.let { telemetry.addData("Heading Error (rad)", it) }
    }

    private fun wrapAngle(angle: Double): Double {
        var wrapped = angle
        while (wrapped > Math.PI) wrapped -= 2.0 * Math.PI
        while (wrapped < -Math.PI) wrapped += 2.0 * Math.PI
        return wrapped
    }

    private companion object {
        const val HEADING_TOLERANCE_RADIANS = 0.035
        const val TURN_GAIN = 0.8
        const val MIN_AUTO_TURN = 0.12
        const val MAX_AUTO_TURN = 0.35
    }

    private fun updateMechanisms() {
        val dpadUp = gamepad1.dpad_up || gamepad2.dpad_up
        val dpadDown = gamepad1.dpad_down || gamepad2.dpad_down
        shooterControls.update(gamepad1.y, dpadUp, dpadDown)

        currentShooterPower = shooterControls.requestedPower(
            reverse = gamepad1.left_bumper,
            pidShoot = gamepad1.right_bumper
        )

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
        telemetry.addData("Shooter flywheel 1 RPM", "%.0f", shooter.measuredRpm1)
        telemetry.addData("Shooter flywheel 2 RPM", "%.0f", shooter.measuredRpm2)
        val selectedPidTarget = shooterControls.pidTargetRpm
        if (selectedPidTarget != null) {
            telemetry.addData("PID target RPM", "%.0f", selectedPidTarget)
        } else {
            telemetry.addData("PID target RPM", "Manual mode: no RPM target")
        }
        telemetry.addData("PID shooting active", selectedPidTarget != null && currentShooterPower > 0.0)
        telemetry.addData("PID commanded RPM", "%.0f", shooter.targetRpm)
        telemetry.addData("PIDF gains (P/I/D/F)", shooter.pidGainsRead)
        telemetry.addData("PID feedback", "Flywheel 1 RPM (shared motor power)")
        telemetry.addData("Shooter preset (G1/G2 up/down)", "%.2f", shooterPreset)
        telemetry.addData("Shooter mode (hold G1 Y for manual)", shooterMode)
        telemetry.addData("Shooter speed state", shooter.speedStatus)
        telemetry.addData("D-pad seen (up/down)", "%s / %s", gamepad1.dpad_up || gamepad2.dpad_up, gamepad1.dpad_down || gamepad2.dpad_down)
        telemetry.addData("Shooter requested fraction", "%.2f", currentShooterPower)
        telemetry.addData("Shooter motor power 1 / 2", "%.2f / %.2f", shooter.flywheelPower1, shooter.flywheelPower2)
        telemetry.addData("Manual blocker (G1/G2 B)", gamepad1.b || gamepad2.b)
        telemetry.addData("Blocker open", shooter.blockerOpen)
        telemetry.addData("Flywheel 1 within 5 percent of target", shooter.atSpeed)
        telemetry.addData("Automatic feed ready", shooter.feedReady)
        telemetry.addData("Feed readiness", shooter.feedReadiness)
        telemetry.addData("Blocker status", shooter.blockerStatus)
        telemetry.addData("Blocker commanded position", "%.2f", shooter.blockerCommandedPosition)
        telemetry.addData("Shooter preset reference RPM", "%.0f", ShooterSubsystem.MAX_MOTOR_RPM)
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
