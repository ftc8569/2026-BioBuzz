package org.firstinspires.ftc.teamcode.autodog

import android.util.Log
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Subsytem.DriveSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.FieldLocation
import org.firstinspires.ftc.teamcode.Subsytem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.RobotLocationSubsystem

/**
 * Auto dog chase with the tracker on a laptop: a debugging setup, not for matches.
 * python/autodog_run.py (BioBuzzAutoDog, on a laptop on the robot Wi-Fi, reading the
 * Limelight stream) tracks the balls. This OpMode streams Pinpoint odometry to it and
 * drives onto the target it sends back. "Auto Dog Chase" is the on-robot version.
 *
 * Controls are ChaseDriver's. The robot must be still during init:
 * RobotLocationSubsystem resets the Pinpoint IMU.
 */
@TeleOp(name = "Auto Dog Chase (laptop tracker)", group = "Test")
class AutoDogLaptopChaseTeleOp : LinearOpMode() {

    override fun runOpMode() {
        val out = JoinedTelemetry(telemetry, PanelsTelemetry.ftcTelemetry)
        val drive = DriveSubsystem(hardwareMap)
        val intake = IntakeSubsystem(hardwareMap)
        val location = RobotLocationSubsystem(hardwareMap)
        val driver = ChaseDriver(drive, intake)
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

        out.addLine("Auto Dog (laptop tracker) ready on UDP ${link.localPort}")
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
                val loc = location.getLocation()
                val now = link.clock()
                val (rx, ry) = FieldFrame.inchesToTracker(loc.xInches, loc.yInches)
                val theta = loc.headingRadians
                link.sendPose(rx, ry, theta, now)

                val target = link.latestTarget
                val fresh = target != null && target.hasTarget && target.age(now) < tune.maxTargetAgeS
                driver.step(gamepad1, rx, ry, theta, if (fresh) target!!.predictedTo(now) else null)

                out.addData("Mode", driver.mode)
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
                val p = driver.power
                out.addData("Cmd fwd/left/turn", "%.2f %.2f %.2f".format(p.forward, p.left, p.turn))
                out.addData("Bad packets", link.badPackets)
                out.update()

                // Readable over Wi-Fi with: adb logcat -s AutoDog
                if (now - lastLog > 0.2) {
                    lastLog = now
                    Log.i(TAG, "mode=%s pose=%.3f,%.3f,%.3f fwd=%.2f left=%.2f turn=%.2f target=%s".format(
                        driver.mode, rx, ry, theta, p.forward, p.left, p.turn, targetText))
                }
            }
        } finally {
            driver.stop()
            limelight?.stop()
            link.close()
        }
    }

    companion object {
        private const val TAG = "AutoDog"
    }
}
