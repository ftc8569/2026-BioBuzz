package org.firstinspires.ftc.teamcode.autodog

import android.util.Log
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Subsytem.DriveSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.FieldLocation
import org.firstinspires.ftc.teamcode.Subsytem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.RobotLocationSubsystem
import org.firstinspires.ftc.teamcode.vision.HiveCameraMount

/**
 * Onboard vision settings, live-tunable in Panels. The camera position comes
 * from HiveCameraMount, the same Limelight the HIVE localizer uses, so measure
 * it once there.
 */
@Configurable
object AutoDogVisionTuning {
    /** Limelight slot running limelight/autodog_balls.py (slot 0 is the HIVE AprilTag pipeline). */
    @JvmField var pipeline = 1
    @JvmField var hfovDeg = 54.5
    @JvmField var vfovDeg = 42.0
    @JvmField var ballDiameterInches = 3.0
    /** Size sanity check; set to 0 to disable. */
    @JvmField var radiusTolerance = 1.8
    @JvmField var maxRangeM = 3.0
    /** Drop detections outside the field. Turn off when the start pose isn't set. */
    @JvmField var fieldGate = true
    /** 0 any color, 1 POLLEN only, 2 NECTAR only. */
    @JvmField var colorFilter = 0
    @JvmField var trackGateM = 0.35
    @JvmField var trackAlpha = 0.6
    @JvmField var trackBeta = 0.3
    @JvmField var dropAfterS = 0.6
    @JvmField var confirmHits = 2
    @JvmField var switchMarginM = 0.3
    /** Ignore Limelight results older than this when they reach the hub (ms). */
    @JvmField var maxResultStalenessMs = 250L
}

/**
 * Auto Dog with no laptop: the Limelight runs limelight/autodog_balls.py, and
 * this OpMode projects, tracks and chases the balls itself. It's legal at
 * events, unlike "Auto Dog Chase". Controls are the same: hold A to chase,
 * A + B to search, release A for manual drive.
 */
@TeleOp(name = "Auto Dog Chase (Onboard)", group = "Test")
class AutoDogOnboardTeleOp : LinearOpMode() {

    override fun runOpMode() {
        val out = JoinedTelemetry(telemetry, PanelsTelemetry.ftcTelemetry)
        val drive = DriveSubsystem(hardwareMap)
        val intake = IntakeSubsystem(hardwareMap)
        val location = RobotLocationSubsystem(hardwareMap)
        val limelight = try {
            hardwareMap.get(Limelight3A::class.java, "limelight")
        } catch (_: Exception) {
            null
        }
        val tune = AutoDogTuning
        val vt = AutoDogVisionTuning
        location.setStartingLocation(
            FieldLocation(tune.startXInches, tune.startYInches, Math.toRadians(tune.startHeadingDeg))
        )
        val chase = ChaseController()
        val vision = OnboardBallTracker(BallProjector(robotToCamera = HiveCameraMount.robotToCamera()))
        val clock = { System.nanoTime() / 1e9 }

        out.addData("Limelight", if (limelight == null) "NOT FOUND (name it 'limelight')" else "ok")
        out.addData("Camera mount", if (HiveCameraMount.configured) "ok" else "NOT SET: fill HiveCameraMount in Panels")
        out.addLine("Upload limelight/autodog_balls.py to Limelight pipeline ${vt.pipeline}")
        out.update()

        waitForStart()
        if (isStopRequested) return
        limelight?.pipelineSwitch(vt.pipeline)
        limelight?.setPollRateHz(50)
        limelight?.start()

        var lastLog = 0.0
        try {
            while (opModeIsActive()) {
                chase.applyTuning()
                applyVisionTuning(vision)

                val loc = location.getLocation()
                val now = clock()
                val (rx, ry) = FieldFrame.inchesToTracker(loc.xInches, loc.yInches)
                val pose = PlanarPose(rx, ry, loc.headingRadians)
                vision.addPose(now, pose)

                var visionStatus = "no Limelight"
                val result = limelight?.latestResult
                if (result != null) {
                    val stalenessMs = result.staleness
                    val frame = LimelightBallFrame.parse(result.pythonOutput)
                    visionStatus = when {
                        !HiveCameraMount.configured -> "camera mount not set"
                        stalenessMs > vt.maxResultStalenessMs -> "stale result ($stalenessMs ms)"
                        frame == null -> "no ball script output (pipeline ${vt.pipeline}?)"
                        else -> {
                            // Staleness counts from arrival at the hub; add the Limelight's own latency.
                            val ageS = (stalenessMs + result.captureLatency + result.targetingLatency) / 1000.0
                            vision.onFrame(result.timestamp, now - ageS, frame)
                            "${frame.balls.size} seen, ${vision.lastDetections} on field, " +
                                "${vision.tracker.confirmed().size} tracked"
                        }
                    }
                }

                val target = vision.target(now, pose, tune.intakeOffset, OnboardBallTracker.colorFilter(vt.colorFilter))
                val fresh = target != null && target.age(now) < tune.maxTargetAgeS
                val mode = chooseChaseMode(gamepad1.a, gamepad1.b, fresh)
                val cmd = chase.command(mode, target, now, pose, tune.intakePower, tune.searchTurn)
                intake.setPower(cmd.intakePower)
                if (mode == ChaseMode.MANUAL) {
                    drive.drive(
                        -gamepad1.left_stick_y.toDouble(),
                        gamepad1.left_stick_x.toDouble(),
                        gamepad1.right_stick_x.toDouble(),
                    )
                } else {
                    drive.driveChase(cmd.power)
                }

                val targetText = if (target == null) "none" else {
                    val (gx, gy) = target.predictedTo(now)
                    "#%d %c at %.2f, %.2f (age %.0f ms)".format(
                        target.trackId, target.color, gx, gy, 1000 * target.age(now))
                }
                val p = cmd.power
                out.addData("Mode", mode.label)
                out.addData("Vision", visionStatus)
                out.addData("Pose (tracker m)", "%.2f, %.2f, %.0f deg".format(rx, ry, Math.toDegrees(pose.theta)))
                out.addData("Target", targetText)
                out.addData("Cmd fwd/left/turn", "%.2f %.2f %.2f".format(p.forward, p.left, p.turn))
                out.update()

                if (now - lastLog > 0.2) {
                    lastLog = now
                    Log.i(TAG, "mode=%s pose=%.3f,%.3f,%.3f fwd=%.2f left=%.2f turn=%.2f vision=%s target=%s".format(
                        mode.label, rx, ry, pose.theta, p.forward, p.left, p.turn, visionStatus, targetText))
                }
            }
        } finally {
            drive.stop()
            intake.setPower(0.0)
            limelight?.stop()
        }
    }

    private fun applyVisionTuning(vision: OnboardBallTracker) {
        val vt = AutoDogVisionTuning
        vision.projector.apply {
            hfovDeg = vt.hfovDeg
            vfovDeg = vt.vfovDeg
            ballRadiusM = vt.ballDiameterInches * 0.0254 / 2
            radiusTolerance = vt.radiusTolerance
            maxRangeM = vt.maxRangeM
            robotToCamera = HiveCameraMount.robotToCamera()
        }
        vision.tracker.apply {
            gateM = vt.trackGateM
            alpha = vt.trackAlpha
            beta = vt.trackBeta
            dropAfterS = vt.dropAfterS
            confirmHits = vt.confirmHits
        }
        vision.selector.switchMarginM = vt.switchMarginM
        vision.fieldGate = vt.fieldGate
    }

    companion object {
        private const val TAG = "AutoDog"
    }
}
