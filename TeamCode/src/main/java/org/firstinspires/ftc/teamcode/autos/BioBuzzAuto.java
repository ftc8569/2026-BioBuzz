package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsytem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Four segments from pedro paths/redleftpickup.pp, then shoot until STOP. */
@Autonomous(name = "BioBuzz Auto")
public class BioBuzzAuto extends OpMode {
    private static final double SHOOTER_PRESET = 0.70;
    private static final double PICKUP_POWER = 1.0;
    private Follower follower;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private Path[] paths;
    // Zero-based: 0..3 follow paths 1..4; 4 shoots at the final pose.
    private int pathIndex = -1;

    @Override
    public void init() {
        PoseFactory p = PoseFactory.degrees();
        Pose start = p.of(55.99999999999999, 11.461151079136723, 90);
        Pose end1 = p.of(43.580575539568336, 9.1251798561151, 180);
        Pose end2 = p.of(13.01654676258992, 7.334532374100731, 180);
        Pose end3 = p.of(12.999280575539576, 14.346762589928069, 270);
        Pose end4 = p.of(57.44892086330934, 9.968345323740998, 270);
        paths = new Path[] {
                // The export starts localization at 90 degrees but path 1 heading at 270.
                Paths.line(start, end1).linear(Math.toRadians(270), Math.toRadians(180)),
                Paths.line(end1, end2).linear(end1, end2),
                Paths.line(end2, end3).linear(end2, end3),
                Paths.line(end3, end4).linear(end3, end4)
        };
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, System::nanoTime);
        pathIndex = -1;
        shooter.startVoltagePolling();
    }

    @Override
    public void start() {
        pathIndex = 0;
        follower.follow(paths[pathIndex]);
    }

    @Override
    public void loop() {
        if (pathIndex < 0) return;
        follower.update();
        if (pathIndex < paths.length && !follower.isBusy()) {
            pathIndex++;
            if (pathIndex < paths.length) {
                follower.follow(paths[pathIndex]);
            } else {
                follower.hold(paths[paths.length - 1].endPose());
            }
        }

        if (pathIndex == paths.length) {
            shooter.updateShot(SHOOTER_PRESET, ShooterSubsystem.ControlMode.VELOCITY_PID);
            intake.setPower(shooter.getFeedReady() ? SHOOTER_PRESET : 0.0);
        } else {
            shooter.setFlywheelPower(0.0);
            intake.setPower(pathIndex == 1 ? PICKUP_POWER : 0.0);
        }

        telemetry.addData("Auto stage", pathIndex < paths.length
                ? "Path " + (pathIndex + 1) : "Shooting");
        telemetry.addData("Shooter target RPM", shooter.getTargetRpm());
        telemetry.addData("Flywheel 1 RPM", shooter.getMeasuredRpm1());
        telemetry.addData("Flywheel 2 RPM", shooter.getMeasuredRpm2());
        telemetry.addData("Feed latched", shooter.getFeedReady());
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooter != null) shooter.stopVoltagePolling();
        pathIndex = -1;
        if (follower != null) {
            follower.stop();
            follower.update();
        }
        if (intake != null) intake.setPower(0.0);
        if (shooter != null) {
            shooter.setManualBlockerOpen(false);
            shooter.setFlywheelPower(0.0);
        }
    }
}
