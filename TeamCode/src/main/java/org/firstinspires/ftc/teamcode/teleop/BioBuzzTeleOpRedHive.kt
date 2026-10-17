package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.Subsytem.FieldLocation
import org.firstinspires.ftc.teamcode.Subsytem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.RobotLocationSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.VisionSubsystem
import kotlin.math.abs
import kotlin.math.sign

@TeleOp(name = "BioBuzz NextFTC Kotlin TeleOp")
class BioBuzzTeleOpRedHive : NextFTCOpMode() {
    // The four mecanum drive motors let the robot drive forward, sideways,
    // backward, and turn.
    private lateinit var driveFL: DcMotor
    private lateinit var driveFR: DcMotor
    private lateinit var driveBL: DcMotor
    private lateinit var driveBR: DcMotor

    // Each subsystem owns one job so this main file can coordinate the robot
    // without needing to know every motor and sensor detail.
    private lateinit var intake: IntakeSubsystem
    private lateinit var shooter: ShooterSubsystem
    private lateinit var robotLocation: RobotLocationSubsystem
    private lateinit var vision: VisionSubsystem
    private lateinit var pollenMacro: PollenMacro

    // The starting position should be set only once, not every update cycle.
    private var locationInitialized = false
    private var previousXButtonPressed = false
    private var redHiveHeading: Double? = null
    private var rotatingToRedHive = false
    private var movingToRedHive = false
    private var blockerOpen = false

    // Used only to show how much each drive encoder changed since the last
    // update. Pinpoint, rather than these drive encoders, provides the location.
    private var previousDriveEncoderPositions: IntArray? = null

    override fun onInit() {
        // Send telemetry to the Driver Station about every 50 milliseconds.
        telemetry.setMsTransmissionInterval(50)

        // HardwareMap is the robot's address book. These names must exactly
        // match the names in the Robot Controller hardware configuration.
        driveFL = hardwareMap.get(DcMotor::class.java, "driveFL")
        driveFR = hardwareMap.get(DcMotor::class.java, "driveFR")
        driveBL = hardwareMap.get(DcMotor::class.java, "driveBL")
        driveBR = hardwareMap.get(DcMotor::class.java, "driveBR")

        // The left motors are mounted opposite the right motors, so reverse
        // them to make all four wheels drive forward together.
        driveFL.direction = DcMotorSimple.Direction.REVERSE
        driveBL.direction = DcMotorSimple.Direction.REVERSE

        // Build the robot's helper objects and register them with NextFTC.
        intake = IntakeSubsystem(hardwareMap)
        shooter = ShooterSubsystem(hardwareMap)
        shooter.setControlMode(ShooterSubsystem.ControlMode.MANUAL_POWER)
        robotLocation = RobotLocationSubsystem(hardwareMap)
        vision = VisionSubsystem(hardwareMap)
        pollenMacro = PollenMacro(r = 10.0, loopAmount = 8)
        addComponents(
            SubsystemComponent(intake, shooter, robotLocation, vision),
            BulkReadComponent
        )
    }

    override fun onUpdate() {
        // Set Pinpoint's zero position during the first update only.
        if (!locationInitialized) {
            robotLocation.setStartingLocation(
                // !-----Set the Starting Position based on Teleop Position (Parked Position)-----!
                FieldLocation(
                    xInches = 0.0,
                    yInches = 0.0,
                    headingRadians = 0.0
                )
            )
            // Set a fixed hive/approach target from the configured TeleOp start.
            robotLocation.updateRedHiveLocation()
            locationInitialized = true
        }

        // The left stick controls forward/backward and sideways movement.
        // The right stick controls rotation.
        var drive = -gamepad1.left_stick_y.toDouble()
        var strafe = gamepad1.left_stick_x.toDouble()

        // Encoders count how far motor shafts have rotated. These readings are
        // displayed for debugging and are not used as the main field location.
        val driveEncoderPositions = intArrayOf(
            driveFL.currentPosition,
            driveFR.currentPosition,
            driveBL.currentPosition,
            driveBR.currentPosition
        )
        val previousPositions = previousDriveEncoderPositions ?: driveEncoderPositions
        telemetry.addData(
            "Drive encoder delta (ticks)",
            "FL %d, FR %d, BL %d, BR %d",
            driveEncoderPositions[0] - previousPositions[0],
            driveEncoderPositions[1] - previousPositions[1],
            driveEncoderPositions[2] - previousPositions[2],
            driveEncoderPositions[3] - previousPositions[3]
        )

        // Save this cycle's values so the next cycle can calculate the change.
        previousDriveEncoderPositions = driveEncoderPositions

        // Right trigger runs the intake forward, left trigger reverses it,
        // and A stops it.
        val intakePower = when {
            gamepad1.a -> 0.0
            gamepad1.right_trigger > 0.05 -> gamepad1.right_trigger.toDouble()
            gamepad1.left_trigger > 0.05 -> -gamepad1.left_trigger.toDouble()
            else -> 0.0
        }
        intake.setPower(intakePower)

        // Right bumper runs the shooter forward, left bumper reverses it,
        // and releasing both bumpers stops it.
        val shooterPower = when {
            gamepad1.right_bumper -> 1.0
            gamepad1.left_bumper -> -1.0
            else -> 0.0
        }
        if (shooterPower > 0.0) {
            shooter.updateShot(shooterPower, ShooterSubsystem.ControlMode.MANUAL_POWER)
        } else {
            shooter.setFlywheelPower(shooterPower)
        }
        // Keep the existing B-open/X-close blocker behavior.
        if (gamepad1.x) blockerOpen = false
        else if (gamepad1.b) blockerOpen = true
        shooter.setManualBlockerOpen(blockerOpen)
        val location = robotLocation.getLocation()

        // X starts auto-aim and travel without changing the saved hive coordinates.
        if (gamepad1.x && !previousXButtonPressed) {
            redHiveHeading = robotLocation.calculateRotationToRedHive(location)
            rotatingToRedHive = redHiveHeading != null
            movingToRedHive = robotLocation.redHiveTargetLocation != null
        }
        previousXButtonPressed = gamepad1.x

        // Let the driver take over immediately by moving either stick.
        if (movingToRedHive &&
            (abs(drive) > MANUAL_INPUT_DEADBAND ||
                abs(strafe) > MANUAL_INPUT_DEADBAND ||
                abs(gamepad1.right_stick_x) > MANUAL_INPUT_DEADBAND)
        ) {
            movingToRedHive = false
        }
        if (movingToRedHive) {
            val target = robotLocation.redHiveTargetLocation
            if (target == null) {
                movingToRedHive = false
            } else {
                val deltaX = target.xInches - location.xInches
                val deltaY = target.yInches - location.yInches
                val distance = kotlin.math.hypot(deltaX, deltaY)
                if (distance <= POSITION_TOLERANCE_INCHES) {
                    movingToRedHive = false
                } else {
                    val speed = (distance * POSITION_GAIN)
                        .coerceIn(MIN_AUTO_DRIVE, MAX_AUTO_DRIVE)
                    val headingCos = kotlin.math.cos(location.headingRadians)
                    val headingSin = kotlin.math.sin(location.headingRadians)
                    drive = speed * (headingCos * deltaX + headingSin * deltaY) / distance
                    strafe = speed * (headingSin * deltaX - headingCos * deltaY) / distance
                }
            }
        }

        // Heading feedback turns toward the saved hive; manual turning cancels it.
        val manualTurn = gamepad1.right_stick_x.toDouble()
        var turn = manualTurn
        var headingError: Double? = null
        if (abs(manualTurn) > 0.05) {
            rotatingToRedHive = false
        } else if (rotatingToRedHive) {
            val targetHeading = redHiveHeading
            if (targetHeading == null) {
                rotatingToRedHive = false
            } else {
                headingError = wrapAngle(targetHeading - location.headingRadians)
                if (abs(headingError) <= HEADING_TOLERANCE_RADIANS) {
                    rotatingToRedHive = false
                } else {
                    val correction = (-TURN_GAIN * headingError)
                        .coerceIn(-MAX_AUTO_TURN, MAX_AUTO_TURN)
                    turn = if (abs(correction) < MIN_AUTO_TURN) {
                        sign(correction) * MIN_AUTO_TURN
                    } else {
                        correction
                    }
                }
            }
        }

        // Mecanum math combines the requested movements. The denominator keeps
        // every motor power between -1.0 and 1.0.
        val denominator = maxOf(
            abs(drive) + abs(strafe) + abs(turn),
            1.0
        )
        driveFL.power = (drive + strafe + turn) / denominator
        driveFR.power = (drive - strafe - turn) / denominator
        driveBL.power = (drive - strafe + turn) / denominator
        driveBR.power = (drive + strafe - turn) / denominator

        // Telemetry is the robot's report card for the driver. It shows useful
        // numbers without changing how the robot moves.
        telemetry.addData("Intake Power", intakePower)
        telemetry.addData("Shooter Power", shooterPower)
        telemetry.addData("Robot X (in)", location.xInches)
        telemetry.addData("Robot Y (in)", location.yInches)
        telemetry.addData("Robot Heading (rad)", location.headingRadians)
        telemetry.addData("Red Hive Target", robotLocation.redHiveTargetLocation)
        telemetry.addData("Moving to Red Hive Target", movingToRedHive)
        telemetry.addData(
            "Red Hive Heading (rad)",
            redHiveHeading ?: "Press X to auto-aim and move to the saved target"
        )
        telemetry.addData(
            "Turning to Red Hive",
            if (rotatingToRedHive) "Yes (right stick cancels)" else "No"
        )
        headingError?.let { telemetry.addData("Heading Error (rad)", it) }
        //telemetry.addData("Visible AprilTags", vision.getVisibleTagIds())
        //telemetry.addData("Robot Pose", vision.getRobotPose() ?: "No valid pose")
        telemetry.update()
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
}
