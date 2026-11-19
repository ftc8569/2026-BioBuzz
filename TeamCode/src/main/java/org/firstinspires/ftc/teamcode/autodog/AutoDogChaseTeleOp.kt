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

/** ChaseController uses +left and +CCW; DriveSubsystem.drive uses +strafe right and +turn CW. */
fun DriveSubsystem.driveChase(p: DrivePower) = drive(p.forward, -p.left, -p.turn)

/** Copies the live Panels values into the controller; call every loop. */
fun ChaseController.applyTuning(t: AutoDogTuning = AutoDogTuning) {
    kTurn = t.kTurn
    kForward = t.kForward
    kStrafe = t.kStrafe
    maxForward = t.maxForward
    maxStrafe = t.maxStrafe
    maxTurn = t.maxTurn
    intakeOffset = t.intakeOffset
}

/** Live-tunable from Panels Configurables. Distances in meters unless named. */
@Configurable
object AutoDogTuning {
    @JvmField var kTurn = 1.6
    @JvmField var kForward = 1.2
    @JvmField var kStrafe = 1.5
    // Low caps for first runs on a real robot; raise once the chase looks right.
    @JvmField var maxForward = 0.35
    @JvmField var maxStrafe = 0.25
    @JvmField var maxTurn = 0.40
    @JvmField var intakeOffset = 0.22
    @JvmField var intakePower = 1.0
    @JvmField var searchTurn = 0.25
    /** Ignore targets older than this (s): the tracker or the link has stalled. */
    @JvmField var maxTargetAgeS = 0.25

    /** Pose Pinpoint is told at init, in the team's center-origin frame (docs/hive-vision.md). */
    @JvmField var startXInches = 0.0
    @JvmField var startYInches = 0.0
    @JvmField var startHeadingDeg = 0.0

    /** Limelight pipeline to run while chasing; its stream is what the laptop reads. */
    @JvmField var limelightPipeline = 0
}

/**
 * Auto dog chase. python/autodog_run.py (on a laptop on the robot Wi-Fi, reading the
 * Limelight stream) tracks POLLEN and NECTAR in field coordinates. This OpMode streams
 * Pinpoint odometry to it and drives onto the target it sends back.
 *
 * Gamepad 1:
 *   hold A      chase the target with the intake on; with no fresh target the robot
 *               stops. Releasing A is the dead-man switch.
 *   hold A + B  same, but spin slowly in place while there is no target
 *   sticks      normal manual drive whenever A is not held
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
        val limelight = try {
            hardwareMap.get(Limelight3A::class.java, "limelight")
        } catch (_: Exception) {
            null
        }
        val tune = AutoDogTuning
        location.setStartingLocation(
            FieldLocation(tune.startXInches, tune.startYInches, Math.toRadians(tune.startHeadingDeg))
        )
        // null host: learn the laptop's address from its first clock-sync ping
        val link = AutoDogLink(piHost = null)
        val chase = ChaseController()

        out.addLine("Auto Dog ready on UDP ${link.localPort}")
        out.addLine("Laptop: python autodog_run.py robot --rc 192.168.43.1 --source limelight")
        out.addData("Limelight", if (limelight == null) "NOT FOUND (name it 'limelight')" else "ok")
        out.update()

        waitForStart()
        if (isStopRequested) {
            link.close()
            return
        }
        limelight?.pipelineSwitch(tune.limelightPipeline)
        limelight?.start()

        var lastLog = 0.0
        try {
            while (opModeIsActive()) {
                chase.applyTuning()

                val loc = location.getLocation()
                val now = link.clock()
                val (rx, ry) = FieldFrame.inchesToTracker(loc.xInches, loc.yInches)
                val theta = loc.headingRadians
                link.sendPose(rx, ry, theta, now)

                val target = link.latestTarget
                val fresh = target != null && target.hasTarget && target.age(now) < tune.maxTargetAgeS
                val chaseMode = chooseChaseMode(gamepad1.a, gamepad1.b, fresh)
                val cmd = chase.command(chaseMode, target, now, PlanarPose(rx, ry, theta), tune.intakePower, tune.searchTurn)
                val power = cmd.power
                val mode = chaseMode.label
                intake.setPower(cmd.intakePower)
                if (chaseMode == ChaseMode.MANUAL) {
                    drive.drive(
                        -gamepad1.left_stick_y.toDouble(),
                        gamepad1.left_stick_x.toDouble(),
                        gamepad1.right_stick_x.toDouble(),
                    )
                } else {
                    drive.driveChase(power)
                }

                out.addData("Mode", mode)
                out.addData("Laptop", if (link.pingsAnswered > 0) "synced (${link.pingsAnswered} pings)" else "waiting for ping")
                out.addData("Pose (tracker m)", "%.2f, %.2f, %.0f deg".format(rx, ry, Math.toDegrees(theta)))
                val targetText = when {
                    target == null -> "none received"
                    !target.hasTarget -> "laptop sees no ball"
                    else -> {
                        val (gx, gy) = target.predictedTo(now)
                        "#%d %c at %.2f, %.2f (age %.0f ms)".format(
                            target.trackId, target.color, gx, gy, 1000 * target.age(now))
                    }
                }
                out.addData("Target", targetText)
                out.addData("Cmd fwd/left/turn", "%.2f %.2f %.2f".format(power.forward, power.left, power.turn))
                out.addData("Bad packets", link.badPackets)
                out.update()

                // Readable over Wi-Fi with: adb logcat -s AutoDog
                if (now - lastLog > 0.2) {
                    lastLog = now
                    Log.i(TAG, "mode=%s pose=%.3f,%.3f,%.3f fwd=%.2f left=%.2f turn=%.2f target=%s".format(
                        mode, rx, ry, theta, power.forward, power.left, power.turn, targetText))
                }
            }
        } finally {
            drive.stop()
            intake.setPower(0.0)
            limelight?.stop()
            link.close()
        }
    }

    companion object {
        private const val TAG = "AutoDog"
    }
}
