package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.Subsytem.DriveSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.FieldLocation
import org.firstinspires.ftc.teamcode.Subsytem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.RobotLocationSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem

@TeleOp(name = "BioBuzz NextFTC Kotlin TeleOp")
class BioBuzzTeleOpBlueHive : NextFTCOpMode() {
    private lateinit var drive: DriveSubsystem
    private lateinit var intake: IntakeSubsystem
    private lateinit var shooter: ShooterSubsystem
    private lateinit var robotLocation: RobotLocationSubsystem
    private lateinit var driveMotors: List<DcMotor>
    private lateinit var pollenMacro: PollenMacro
    private var previousDriveEncoderPositions: IntArray? = null
    private var locationInitialized = false
    private var previousXButtonPressed = false
    private var blueHiveHeading: Double? = null
    private var rotatingToBlueHive = false
    private var movingToBlueHive = false
    private val shooterControls = ShooterControls()
    private val shooterPreset: Double get() = shooterControls.preset
    private val shooterMode: ShooterSubsystem.ControlMode get() = shooterControls.mode
    private var currentShooterPower = 0.0
    private var currentIntakePower = 0.0

    private var isBlue = false

    private var isRed = false

    override fun onInit() {
        telemetry.setMsTransmissionInterval(50)
        drive = DriveSubsystem(hardwareMap)
        driveMotors = listOf("driveFL", "driveFR", "driveBL", "driveBR")
            .map { hardwareMap.get(DcMotor::class.java, it) }
        intake = IntakeSubsystem(hardwareMap)
        shooter = ShooterSubsystem(hardwareMap)
        robotLocation = RobotLocationSubsystem(hardwareMap)
        pollenMacro = PollenMacro(r = 10.0, loopAmount = 8)
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
            robotLocation.updateBlueHiveLocation()
            locationInitialized = true
        }
        val positions = driveMotors.map { it.currentPosition }.toIntArray()
        val previousPositions = previousDriveEncoderPositions ?: positions
        telemetry.addData(
            "Drive encoder delta (ticks)",
            "FL %d, FR %d, BL %d, BR %d",
            positions[0] - previousPositions[0],
            positions[1] - previousPositions[1],
            positions[2] - previousPositions[2],
            positions[3] - previousPositions[3]
        )
        previousDriveEncoderPositions = positions
        val location = robotLocation.getLocation()
        if (gamepad1.x && !previousXButtonPressed) {
            blueHiveHeading = robotLocation.calculateRotationToBlueHive(location)
            rotatingToBlueHive = blueHiveHeading != null
            movingToBlueHive = robotLocation.blueHiveTargetLocation != null
        }
        previousXButtonPressed = gamepad1.x
        var forward = -gamepad1.left_stick_y.toDouble()
        var strafe = gamepad1.left_stick_x.toDouble()
        if (movingToBlueHive &&
            (kotlin.math.abs(forward) > MANUAL_INPUT_DEADBAND ||
                kotlin.math.abs(strafe) > MANUAL_INPUT_DEADBAND ||
                kotlin.math.abs(gamepad1.right_stick_x) > MANUAL_INPUT_DEADBAND)
        ) {
            movingToBlueHive = false
        }
        if (movingToBlueHive) {
            val target = robotLocation.blueHiveTargetLocation
            if (target == null) {
                movingToBlueHive = false
            } else {
                val deltaX = target.xInches - location.xInches
                val deltaY = target.yInches - location.yInches
                val distance = kotlin.math.hypot(deltaX, deltaY)
                if (distance <= POSITION_TOLERANCE_INCHES) {
                    movingToBlueHive = false
                } else {
                    val speed = (distance * POSITION_GAIN)
                        .coerceIn(MIN_AUTO_DRIVE, MAX_AUTO_DRIVE)
                    val headingCos = kotlin.math.cos(location.headingRadians)
                    val headingSin = kotlin.math.sin(location.headingRadians)
                    forward = speed * (headingCos * deltaX + headingSin * deltaY) / distance
                    strafe = speed * (headingSin * deltaX - headingCos * deltaY) / distance
                }
            }
        }
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
            forward,
            strafe,
            turn
        )
        telemetry.addData("Robot X (in)", location.xInches)
        telemetry.addData("Robot Y (in)", location.yInches)
        telemetry.addData("Robot Heading (rad)", location.headingRadians)
        telemetry.addData("Blue Hive Target", robotLocation.blueHiveTargetLocation)
        telemetry.addData("Moving to Blue Hive Target", movingToBlueHive)
        telemetry.addData(
            "Blue Hive Heading (rad)",
            blueHiveHeading ?: "Press G1 X to auto-aim and move to the saved target"
        )
        telemetry.addData(
            "Turning to Blue Hive",
            if (rotatingToBlueHive) "Yes (right stick cancels)" else "No"
        )
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
        const val POSITION_TOLERANCE_INCHES = 1.0
        const val POSITION_GAIN = 0.06
        const val MIN_AUTO_DRIVE = 0.12
        const val MAX_AUTO_DRIVE = 0.45
        const val MANUAL_INPUT_DEADBAND = 0.05
        const val TURN_GAIN = 0.8
        const val MIN_AUTO_TURN = 0.12
        const val MAX_AUTO_TURN = 0.35
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
        telemetry.addData("Shooter flywheel 1 RPM", "%.0f", shooter.measuredRpm1)
        telemetry.addData("Shooter flywheel 2 RPM", "%.0f", shooter.measuredRpm2)
        telemetry.addData("Shooter preset (G1/G2 up/down)", "%.2f", shooterPreset)
        telemetry.addData("Shooter mode (G1 Y)", shooterMode)
        telemetry.addData("Shooter speed state", shooter.speedStatus)
        telemetry.addData(
            "D-pad seen (up/down)",
            "%s / %s",
            gamepad1.dpad_up || gamepad2.dpad_up,
            gamepad1.dpad_down || gamepad2.dpad_down
        )
        telemetry.addData("Shooter requested fraction", "%.2f", currentShooterPower)
        telemetry.addData(
            "Shooter motor power 1 / 2",
            "%.2f / %.2f",
            shooter.flywheelPower1,
            shooter.flywheelPower2
        )
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
        telemetry.addData("RPM configuration valid", shooter.rpmConfigurationValid)
        telemetry.addData(
            "Raw ticks/s 1 / 2",
            "%.0f / %.0f",
            shooter.measuredTicksPerSecond1,
            shooter.measuredTicksPerSecond2
        )
        telemetry.addData(
            "Encoder position ticks 1 / 2",
            "%d / %d",
            shooter.encoderPosition1,
            shooter.encoderPosition2
        )
        telemetry.addData(
            "PID target ticks/s 1 / 2",
            "%.0f / %.0f",
            shooter.targetTicksPerSecond1,
            shooter.targetTicksPerSecond2
        )
        telemetry.addData(
            "Motor config ticks/rev 1 / 2",
            "%.0f / %.0f",
            shooter.configuredTicksPerRev1,
            shooter.configuredTicksPerRev2
        )
        telemetry.addData(
            "Motor config max ticks/s 1 / 2",
            "%.0f / %.0f",
            shooter.configuredMaxTicksPerSecond1,
            shooter.configuredMaxTicksPerSecond2
        )
        telemetry.update()
    }

    override fun onStop() {
        drive.stop()
        intake.setPower(0.0)
        shooter.setManualBlockerOpen(false)
        shooter.setFlywheelPower(0.0)
    }
}
