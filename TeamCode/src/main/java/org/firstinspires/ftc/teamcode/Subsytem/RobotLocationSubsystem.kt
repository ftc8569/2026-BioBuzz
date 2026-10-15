package org.firstinspires.ftc.teamcode.Subsytem

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D

import dev.nextftc.core.subsystems.Subsystem

data class FieldLocation(
    // X and Y describe where the robot is on the field. Heading describes the
    // direction it faces. Distances are stored in inches and angles in radians.
    val xInches: Double,
    val yInches: Double,
    val headingRadians: Double
)

/**
 * Pinpoint mounting values are measured from the robot's tracking point. The X pod is the forward
 * pod and the Y pod is the strafe pod.
 */
data class PinpointCalibration(
    // These offsets describe where the Pinpoint tracking pods are mounted
    // relative to the robot's tracking point. Measure them on the real robot.
    val xPodOffsetMillimeters: Double = -80.0,
    val yPodOffsetMillimeters: Double = -157.475,

    // This tells Pinpoint which kind of odometry pod is installed. Different
    // pod types produce different encoder counts for the same distance.
    val podType: GoBildaPinpointDriver.GoBildaOdometryPods =
        GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD,

    // Positive encoder directions should be checked on the real robot:
    // forward motion should increase the X pod, and left motion should
    // increase the Y pod.
    val xDirection: GoBildaPinpointDriver.EncoderDirection =
        GoBildaPinpointDriver.EncoderDirection.FORWARD,
    val yDirection: GoBildaPinpointDriver.EncoderDirection =
        GoBildaPinpointDriver.EncoderDirection.FORWARD
) {
    init {
        // A calibration value must be a real number, not infinity or NaN.
        require(xPodOffsetMillimeters.isFinite())
        require(yPodOffsetMillimeters.isFinite())
    }
}

class RobotLocationSubsystem(
    hardwareMap: HardwareMap,
    calibration: PinpointCalibration = PinpointCalibration()
) : Subsystem {
    // "pinpoint" must exactly match the device name in the Robot Controller
    // hardware configuration.
    private val pinpoint = hardwareMap.get(GoBildaPinpointDriver::class.java, PINPOINT_NAME)
    private var redHiveLocation: FieldLocation? = null
    var redHiveTargetLocation: FieldLocation? = null
        private set
    private var blueHiveLocation: FieldLocation? = null
    var blueHiveTargetLocation: FieldLocation? = null
        private set

    init {
        // Tell Pinpoint where its two tracking pods are mounted.
        pinpoint.setOffsets(
            calibration.xPodOffsetMillimeters,
            calibration.yPodOffsetMillimeters,
            DistanceUnit.MM
        )

        // Tell Pinpoint how to convert pod encoder movement into distance and
        // which direction should count as positive movement.
        pinpoint.setEncoderResolution(calibration.podType)
        pinpoint.setEncoderDirections(calibration.xDirection, calibration.yDirection)

        // The robot must be still while this resets the starting pose and
        // recalibrates Pinpoint's internal motion sensor.
        pinpoint.resetPosAndIMU()
    }

    fun getLocation(): FieldLocation {
        // update() asks the sensor for fresh readings and calculates a new pose.
        pinpoint.update()
        val pose = pinpoint.position
        return FieldLocation(
            // Convert Pinpoint's values to the units used by the team code.
            xInches = pose.getX(DistanceUnit.INCH),
            yInches = pose.getY(DistanceUnit.INCH),
            headingRadians = pose.getHeading(AngleUnit.RADIANS)
        )

    }

    fun setStartingLocation(location: FieldLocation) {
        // This tells Pinpoint, "You are here now." It is useful when an
        // autonomous or TeleOp program starts from a known field position.
        pinpoint.setPosition(
            Pose2D(
                DistanceUnit.INCH,
                location.xInches,
                location.yInches,
                AngleUnit.RADIANS,
                location.headingRadians
            )
        )
    }

    fun updateRedHiveLocation() {
        val hive = getLocation()
        redHiveLocation = hive
        redHiveTargetLocation = getApproachTarget(hive)
    }

    fun updateBlueHiveLocation() {
        val hive = getLocation()
        blueHiveLocation = hive
        blueHiveTargetLocation = getApproachTarget(hive)
    }

    private fun getApproachTarget(hive: FieldLocation): FieldLocation {
        val rightX = kotlin.math.sin(hive.headingRadians)
        val rightY = -kotlin.math.cos(hive.headingRadians)
        val forwardX = kotlin.math.cos(hive.headingRadians)
        val forwardY = kotlin.math.sin(hive.headingRadians)
        return FieldLocation(
            xInches = hive.xInches + RED_HIVE_RIGHT_OFFSET_INCHES * rightX +
                RED_HIVE_FORWARD_OFFSET_INCHES * forwardX,
            yInches = hive.yInches + RED_HIVE_RIGHT_OFFSET_INCHES * rightY +
                RED_HIVE_FORWARD_OFFSET_INCHES * forwardY,
            headingRadians = hive.headingRadians
        )
    }

    fun calculateRotationToBlueHive(location: FieldLocation): Double? {
        return calculateRotationToHive(blueHiveLocation, location)
    }

    fun calculateRotationToRedHive(location: FieldLocation): Double? {
        return calculateRotationToHive(redHiveLocation, location)
    }

    private fun calculateRotationToHive(hive: FieldLocation?, location: FieldLocation): Double? {
        hive ?: return null
        val deltaX = hive.xInches - location.xInches
        val deltaY = hive.yInches - location.yInches
        return kotlin.math.atan2(deltaY, deltaX)
    }

    companion object {
        // This is the hardware configuration name used to find the Pinpoint.
        private const val PINPOINT_NAME = "pinpoint"
        private const val RED_HIVE_RIGHT_OFFSET_INCHES = 3.0
        private const val RED_HIVE_FORWARD_OFFSET_INCHES = 53.369

    }
}
