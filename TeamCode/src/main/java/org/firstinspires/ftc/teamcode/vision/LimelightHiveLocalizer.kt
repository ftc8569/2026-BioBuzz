package org.firstinspires.ftc.teamcode.vision

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.bylazar.configurables.annotations.Configurable
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

data class VisionFrame(
    val estimate: HivePoseEstimate?,
    val seenIds: List<Int>,
    val ageMillis: Long,
    val status: String
)

/** Camera position in a robot frame with +X forward, +Y left, +Z up. */
@Configurable
object HiveCameraMount {
    @JvmField var configured = false
    @JvmField var forwardInches = 0.0
    @JvmField var leftInches = 0.0
    @JvmField var heightInches = 12.0
    @JvmField var yawLeftDegrees = 0.0
    @JvmField var pitchUpDegrees = 0.0
    @JvmField var rollDegrees = 0.0

    private val levelForwardCamera = Mat3.rows(
        0.0, 0.0, 1.0,
        -1.0, 0.0, 0.0,
        0.0, -1.0, 0.0
    )

    fun robotToCamera(): Transform3 {
        val rotation = Mat3.rotZ(Math.toRadians(yawLeftDegrees)) *
            Mat3.rotY(-Math.toRadians(pitchUpDegrees)) *
            levelForwardCamera * Mat3.rotZ(Math.toRadians(rollDegrees))
        return Transform3(Vec3(forwardInches, leftInches, heightInches) * 0.0254, rotation)
    }
}

/** Reads compact camera-to-tag poses; no static Limelight field map is involved. */
class LimelightHiveLocalizer(private val camera: Limelight3A) {
    private val solver = HivePoseSolver()
    private var previousCaptureNanos = Long.MIN_VALUE
    private var previousFrame = VisionFrame(null, emptyList(), 0, "Waiting for camera")

    fun start(pipeline: Int = 0) {
        camera.pipelineSwitch(pipeline)
        camera.setPollRateHz(50)
        camera.start()
    }

    fun stop() = camera.stop()

    fun read(): VisionFrame {
        if (!HiveCameraMount.configured) return VisionFrame(null, emptyList(), 0, "Set HIVE camera mount in Panels")
        val result = camera.latestResult ?: return VisionFrame(null, emptyList(), 0, "No camera result")
        val age = result.staleness
        if (age > 250) return VisionFrame(null, emptyList(), age, "Stale camera frame")
        if (!result.isValid) return VisionFrame(null, emptyList(), age, "No valid targets")
        val capture = result.controlHubTimeStampNanos
        if (capture == previousCaptureNanos) return previousFrame.copy(ageMillis = age)
        previousCaptureNanos = capture

        val detections = result.fiducialResults.filter { HiveFieldModel.isRed(it.fiducialId) || HiveFieldModel.isBlue(it.fiducialId) }
        val robotToCamera = HiveCameraMount.robotToCamera()
        val observations = detections.mapNotNull { detection ->
            val pose = detection.cameraPoseTargetSpace ?: return@mapNotNull null
            val p = pose.position.toUnit(DistanceUnit.METER)
            val o = pose.orientation
  val values = listOf(p.x, p.y, p.z, o.getYaw(AngleUnit.RADIANS),
   o.getPitch(AngleUnit.RADIANS), o.getRoll(AngleUnit.RADIANS))
  if (!values.all(Double::isFinite)) return@mapNotNull null
  // SDK 12 substitutes an all-zero pose when the pipeline omits c6c_ts.
  // A tag cannot physically coincide with the camera, so discard it.
  if (Vec3(p.x, p.y, p.z).norm() < 0.05) return@mapNotNull null
            val tagToCamera = Transform3(Vec3(p.x, p.y, p.z),
                Mat3.yawPitchRoll(values[3], values[4], values[5]))
            TagObservation(detection.fiducialId, robotToCamera * tagToCamera.inverse())
        }
        val estimate = solver.solve(observations)
        previousFrame = VisionFrame(estimate, observations.map { it.id }, age,
            if (estimate != null) "OK" else if (observations.isEmpty()) "No usable HIVE tags" else "Geometry did not fit")
        return previousFrame
    }
}
