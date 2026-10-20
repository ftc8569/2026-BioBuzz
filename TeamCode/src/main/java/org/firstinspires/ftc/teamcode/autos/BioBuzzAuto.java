package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsytem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.function.LongSupplier;
import java.util.List;
import org.firstinspires.ftc.teamcode.teleop.TelemetryRefresh;
import org.firstinspires.ftc.teamcode.teleop.BlockerFeedDelay;

/** Shoot while stationary, follow pedro paths/basicred.pp, then stop. */
@Autonomous(name = "BasicRed Auto")
public class BioBuzzAuto extends OpMode {
    private static final double SHOOTER_PRESET = ShooterSubsystem.DEFAULT_SHOOTER_PRESET;
    private static final long FEED_DURATION_NANOS = 2_000_000_000L;
    private enum Stage { INITIAL_SHOT, DRIVING, DONE }
    private Stage stage = Stage.DONE;
    private final LongSupplier nanoTime;
    private long feedingStartedNanos = -1L;
    private BlockerFeedDelay blockerFeedDelay;

    public BioBuzzAuto() {
        this(System::nanoTime);
    }

    BioBuzzAuto(LongSupplier nanoTime) {
        this.nanoTime = nanoTime;
    }
    private Follower follower;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private Path path;
    private List<LynxModule> hubs;
    private final TelemetryRefresh telemetryRefresh = new TelemetryRefresh();

    @Override
    public void init() {
        Pose start = startingPose();
        path = buildPath(start);
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, nanoTime::getAsLong);
        stage = Stage.DONE;
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        telemetry.setMsTransmissionInterval(50);
        shooter.startVoltagePolling();
    }

    protected Pose startingPose() {
        return PoseFactory.degrees().of(57.01798561151079, 9.221582733812928, 90);
    }

    protected Path buildPath(Pose start) {
        Pose end = PoseFactory.degrees().of(8.96906474820144, 97.8935251798561, 270);
        // Preserve the export's 90-degree starting pose and 270-degree path heading.
        return Paths.line(start, end).constant(Math.toRadians(270));
    }

    @Override
    public void start() {
        shooter.startBlockerControl();
        blockerFeedDelay = new BlockerFeedDelay(nanoTime::getAsLong);
        feedingStartedNanos = -1L;
        stage = Stage.INITIAL_SHOT;
        telemetryRefresh.reset();
        // Remain stationary while the preload shot spins up and feeds.
        follower.stop();
    }

    @Override
    public void loop() {
        if (stage == Stage.DONE) return;
        for (LynxModule hub : hubs) hub.clearBulkCache();
        follower.update();
        if (stage == Stage.INITIAL_SHOT) {
            if (updateShooting()) {
                shooter.setFlywheelPower(0.0);
                intake.setPower(0.0);
                stage = Stage.DRIVING;
                follower.follow(path);
            }
        } else if (stage == Stage.DRIVING && !follower.isBusy()) {
            stop();
        }

        if (!telemetryRefresh.isDue()) return;
        telemetry.addData("Auto stage", stage);
        telemetry.addData("LQR interval (ms)", shooter.getLqrIntervalMs());
        telemetry.addData("LQR frequency (Hz)", shooter.getLqrFrequencyHz());
        telemetry.addData("Shooter target RPM", shooter.getTargetRpm());
        telemetry.addData("Flywheel 1 RPM", shooter.getMeasuredRpm1());
        telemetry.addData("Flywheel 2 RPM", shooter.getMeasuredRpm2());
        telemetry.addData("Feed latched", shooter.getFeedReady());
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    /** Count two seconds of transfer after spin-up and the blocker-opening delay. */
    private boolean updateShooting() {
        shooter.updateShot(SHOOTER_PRESET, ShooterSubsystem.ControlMode.VELOCITY_LQR);
        double requestedPower = shooter.getFeedReady() ? SHOOTER_PRESET : 0.0;
        double transferPower = blockerFeedDelay.apply(requestedPower, shooter.getBlockerOpen());
        intake.setPower(transferPower);
        if (transferPower <= 0.0) {
            feedingStartedNanos = -1L;
            return false;
        }
        long now = nanoTime.getAsLong();
        if (feedingStartedNanos == -1L) feedingStartedNanos = now;
        return now - feedingStartedNanos >= FEED_DURATION_NANOS;
    }

    @Override
    public void stop() {
        if (shooter != null) shooter.stopVoltagePolling();
        stage = Stage.DONE;
        feedingStartedNanos = -1L;
        if (follower != null) {
            follower.stop();
            follower.update();
        }
        if (hubs != null) {
            for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        if (intake != null) intake.setPower(0.0);
        if (shooter != null) {
            shooter.setManualBlockerOpen(false);
            shooter.setFlywheelPower(0.0);
        }
    }
}
