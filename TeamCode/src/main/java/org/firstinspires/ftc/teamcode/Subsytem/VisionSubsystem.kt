package org.firstinspires.ftc.teamcode.Subsytem

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.LLResultTypes
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.nextftc.core.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D

/**
 * Limelight 3A AprilTag subsystem.
 *
 * The Limelight pipeline must be configured for the FTC field and contain the
 * 2027-2028 AprilTag map. The pipeline index is selected by the caller so the
 * robot code does not hard-code tag IDs or field geometry.
 */
class VisionSubsystem(
    hardwareMap: HardwareMap,
    // The device and pipeline can be changed by the caller without editing
    // the rest of the vision code.
    limelightName: String = DEFAULT_LIMELIGHT_NAME,
    pipelineIndex: Int = DEFAULT_PIPELINE_INDEX
) : Subsystem {
    // HardwareMap finds the Limelight by its configured name.
    private val limelight = hardwareMap.get(Limelight3A::class.java, limelightName)

    init {
        // A pipeline is a saved set of camera instructions, such as "look for
        // AprilTags." Start the selected pipeline and begin receiving results.
        limelight.pipelineSwitch(pipelineIndex)
        limelight.start()
    }

    // The result can be null because the camera may not have produced a frame yet.
    fun getLatestResult(): LLResult? = limelight.latestResult

    fun getRobotPose(): Pose3D? =
        // Only use a pose when the Limelight says the result is valid.
        getLatestResult()
            ?.takeIf { it.isValid }
            ?.botpose

    fun getFiducialResults(): List<LLResultTypes.FiducialResult> =
        // Fiducials include recognizable markers such as AprilTags. If there
        // is no valid camera result, return an empty list instead of crashing.
        getLatestResult()
            ?.takeIf { it.isValid }
            ?.fiducialResults
            ?: emptyList()

    fun getVisibleTagIds(): List<Int> =
        // Turn the full detection objects into just their tag numbers.
        getFiducialResults().map { it.fiducialId }

    fun stop() {
        // Stop the camera when vision is no longer needed.
        limelight.stop()
    }

    companion object {
        const val DEFAULT_LIMELIGHT_NAME = "limelight"
        const val DEFAULT_PIPELINE_INDEX = 0
    }
}
