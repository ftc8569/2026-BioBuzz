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

import java.util.function.LongSupplier;

/** Shoot, follow pedro paths/redleftpickup.pp, then shoot again. */
@Autonomous(name = "BioBuzz Auto")
public class BioBuzzAuto extends OpMode {
    private static final double SHOOTER_PRESET = 0.70;
    private static final double PICKUP_POWER = 1.0;
    private static final long FEED_DURATION_NANOS = 2_000_000_000L;
    private enum Stage { INITIAL_SHOT, DRIVING, FINAL_SHOT, DONE }
    private Stage stage = Stage.DONE;
    private final LongSupplier nanoTime;
    private long feedingStartedNanos = -1L;

    public BioBuzzAuto() {
        this(System::nanoTime);
    }

    BioBuzzAuto(LongSupplier nanoTime) {
        this.nanoTime = nanoTime;
    }
    private Follower follower;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private Path[] paths;
    // Zero-based index into paths 1..4.
    private int pathIndex = 0;

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
        shooter = new ShooterSubsystem(hardwareMap, nanoTime::getAsLong);
        stage = Stage.DONE;
    }

    @Override
    public void start() {
        pathIndex = 0;
        feedingStartedNanos = -1L;
        stage = Stage.INITIAL_SHOT;
        // Remain stationary while the preload shot spins up and feeds.
        follower.stop();
    }

    @Override
    public void loop() {
        if (stage == Stage.DONE) return;
        follower.update();
        if (stage == Stage.DRIVING) {
            if (!follower.isBusy()) {
                pathIndex++;
                if (pathIndex < paths.length) {
                    follower.follow(paths[pathIndex]);
                } else {
                    follower.hold(paths[paths.length - 1].endPose());
                    feedingStartedNanos = -1L;
                    stage = Stage.FINAL_SHOT;
                }
            }
            if (stage == Stage.DRIVING) {
                shooter.setFlywheelPower(0.0);
                intake.setPower(pathIndex == 1 ? PICKUP_POWER : 0.0);
            }
        }

        if (stage == Stage.INITIAL_SHOT || stage == Stage.FINAL_SHOT) {
            if (updateShooting()) {
                if (stage == Stage.INITIAL_SHOT) {
                    shooter.setFlywheelPower(0.0);
                    intake.setPower(0.0);
                    stage = Stage.DRIVING;
                    follower.follow(paths[pathIndex]);
                } else {
                    stop();
                }
            }
        }

        telemetry.addData("Auto stage", stage == Stage.DRIVING
                ? "Path " + (pathIndex + 1) : stage);
        telemetry.addData("Shooter target RPM", shooter.getTargetRpm());
        telemetry.addData("Flywheel 1 RPM", shooter.getMeasuredRpm1());
        telemetry.addData("Flywheel 2 RPM", shooter.getMeasuredRpm2());
        telemetry.addData("Feed latched", shooter.getFeedReady());
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    /** Spin-up time is excluded: the two-second timer starts at measured readiness. */
    private boolean updateShooting() {
        shooter.updateShot(SHOOTER_PRESET, ShooterSubsystem.ControlMode.VELOCITY_PID);
        intake.setPower(shooter.getFeedReady() ? SHOOTER_PRESET : 0.0);
        if (!shooter.getFeedReady()) {
            feedingStartedNanos = -1L;
            return false;
        }
        long now = nanoTime.getAsLong();
        if (feedingStartedNanos == -1L) feedingStartedNanos = now;
        return now - feedingStartedNanos >= FEED_DURATION_NANOS;
    }

    @Override
    public void stop() {
        stage = Stage.DONE;
        feedingStartedNanos = -1L;
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
