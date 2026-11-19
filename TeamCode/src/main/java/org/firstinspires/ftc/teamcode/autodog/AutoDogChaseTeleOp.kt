package org.firstinspires.ftc.teamcode.autodog

import android.util.Log
import android.util.Size
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.Subsytem.DriveSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.FieldLocation
import org.firstinspires.ftc.teamcode.Subsytem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.RobotLocationSubsystem
import org.firstinspires.ftc.vision.VisionPortal

/**
 * Auto dog chase, running entirely on the robot. The global shutter webcam on the
 * Control Hub finds POLLEN and NECTAR (BallVisionProcessor), BallTracker follows each
 * ball in field coordinates using Pinpoint odometry, and ChaseDriver drives onto the
 * nearest one while a driver holds A. See docs/autodog.md.
 *
 * The robot must be still during init: RobotLocationSubsystem resets the Pinpoint IMU.
 */
@TeleOp(name = "Auto Dog Chase", group = "Test")
class AutoDogChaseTeleOp : LinearOpMode() {

    override fun runOpMode() {
        val out = JoinedTelemetry(telemetry, PanelsTelemetry.ftcTelemetry)
        val drive = DriveSubsystem(hardwareMap)
        val intake = IntakeSubsystem(hardwareMap)
        val location = RobotLocationSubsystem(hardwareMap)
        val driver = ChaseDriver(drive, intake)
        val tune = AutoDogTuning
        location.setStartingLocation(
            FieldLocation(tune.startXInches, tune.startYInches, Math.toRadians(tune.startHeadingDeg))
        )

        val processor = BallVisionProcessor()
        var cameraError: String? = null
        val portal: VisionPortal? = try {
            VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName::class.java, CAMERA_NAME))
                .setCameraResolution(Size(tune.camWidthPx, tune.camHeightPx))
                // MJPEG: uncompressed YUY2 cannot carry 1280x720 at a useful frame rate over USB 2.
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false)
                .addProcessor(processor)
                .build()
        } catch (e: Exception) {
            cameraError = e.message ?: e.javaClass.simpleName
            Log.e(TAG, "camera '$CAMERA_NAME' failed to open", e)
            null
        }

        // Show the camera and what it detects during init, so the mount and the color
        // thresholds can be checked on the Driver Station camera stream before driving.
        while (opModeInInit()) {
            out.addLine("Auto Dog (on-robot tracker)")
            if (portal == null) {
                out.addData("Camera", "'$CAMERA_NAME' NOT OPENED: $cameraError")
            } else {
                out.addData("Camera", "%s %.0f fps".format(portal.cameraState, portal.fps))
                processor.latest?.let { out.addData("Sees", describe(it)) }
            }
            out.update()
            sleep(50)
        }
        if (isStopRequested) {
            portal?.close()
            return
        }

        val cfg = TrackerConfig.biobuzz().apply { gateField = tune.gateField }
        val tracker = BallTracker(tune.intrinsics(), tune.mount(), cfg)
        var lastSeq = processor.latest?.seq ?: 0L
        var lastTickAt = Double.NEGATIVE_INFINITY
        var lastTick: TickResult? = null
        var lastFrame: FrameBalls? = null
        var lastLog = 0.0
        try {
            while (opModeIsActive()) {
                val loc = location.getLocation()
                val now = System.nanoTime() / 1e9
                val (rx, ry) = FieldFrame.inchesToTracker(loc.xInches, loc.yInches)
                val theta = loc.headingRadians
                tracker.poses.add(now, rx, ry, theta)

                val frame = processor.latest
                if (frame != null && frame.seq != lastSeq) {
                    lastSeq = frame.seq
                    lastFrame = frame
                    lastTick = tracker.tick(frame.t, frame.detections)
                    // A frame the pose buffer cannot place is not evidence of where balls are.
                    if (lastTick.pose != null) lastTickAt = now
                }
                // Camera stalled or unplugged: stop rather than chase a stale track.
                val visionFresh = now - lastTickAt < tune.maxTargetAgeS
                val target = if (visionFresh) tracker.target(now, TrackerPose(rx, ry, theta)) else null

                driver.step(gamepad1, rx, ry, theta, target?.let { it.x to it.y })

                val targetText = when {
                    portal == null -> "no camera"
                    !visionFresh -> "no fresh camera frame"
                    target == null -> "no ball"
                    else -> "#%d %s at %.2f, %.2f (%.2f m away)".format(
                        target.trackId, target.color ?: '?', target.x, target.y, target.distance)
                }
                out.addData("Mode", driver.mode)
                out.addData("Target", targetText)
                out.addData("Tracks", "%d confirmed of %d".format(tracker.confirmedTracks().size, tracker.tracks.size))
                if (portal == null) {
                    out.addData("Camera", "'$CAMERA_NAME' NOT OPENED: $cameraError")
                } else {
                    out.addData("Camera", "%s %.0f fps".format(portal.cameraState, portal.fps))
                }
                lastFrame?.let { out.addData("Frame", describe(it)) }
                out.addData("Pose (tracker m)", "%.2f, %.2f, %.0f deg".format(rx, ry, Math.toDegrees(theta)))
                val p = driver.power
                out.addData("Cmd fwd/left/turn", "%.2f %.2f %.2f".format(p.forward, p.left, p.turn))
                out.update()

                // Readable over Wi-Fi with: adb logcat -s AutoDog
                if (now - lastLog > 0.2) {
                    lastLog = now
                    Log.i(TAG, "mode=%s pose=%.3f,%.3f,%.3f fwd=%.2f left=%.2f turn=%.2f tracks=%d target=%s frame=%s".format(
                        driver.mode, rx, ry, theta, p.forward, p.left, p.turn, tracker.tracks.size,
                        targetText, lastFrame?.let { describe(it) } ?: "none"))
                }
            }
        } finally {
            driver.stop()
            portal?.close()
        }
    }

    private fun describe(f: FrameBalls): String {
        val counts = f.blobs.groupingBy { it.color }.eachCount()
        return "Y%d R%d B%d, detect %.0f ms, latency %.0f ms%s".format(
            counts[TrackerConfig.POLLEN] ?: 0, counts[TrackerConfig.RED_NECTAR] ?: 0,
            counts[TrackerConfig.BLUE_NECTAR] ?: 0, f.processMs, 1000 * f.latencyS,
            if (f.stampUsed) "" else " (no camera time stamp)")
    }

    companion object {
        private const val TAG = "AutoDog"
        /** The webcam's name in the Robot Controller hardware configuration. */
        const val CAMERA_NAME = "globalShutter"
    }
}
