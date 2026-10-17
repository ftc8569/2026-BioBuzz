package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Subsytem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.function.LongSupplier;
import java.util.List;
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.teleop.TelemetryRefresh;

/** Shoot while stationary, follow pedro paths/basicred.pp, then stop. */
@Autonomous(name = "BasicRed Auto")
public class BioBuzzAuto extends OpMode {
    private static final double SHOOTER_PRESET = ShooterSubsystem.DEFAULT_SHOOTER_PRESET;
    private static final long FEED_DURATION_NANOS = 2_000_000_000L;
    private enum Stage { UNINITIALIZED, READY, INITIAL_SHOT, DRIVING, DONE, FAILED }
    private Stage stage = Stage.UNINITIALIZED;
    private String operation = "Not initialized";
    private String failureMessage;
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
    private Path path;
    private List<LynxModule> hubs;
    private final List<DcMotorEx> stopMotors = new ArrayList<>();
    private final TelemetryRefresh telemetryRefresh = new TelemetryRefresh();

    @Override
    public void init() {
        try {
            operation = "INIT motor setup";
            for (String name : new String[] {"driveFL", "driveFR", "driveBL", "driveBR",
                    "shooter1", "shooter2", "intake1", "intake2"}) {
                DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
                stopMotors.add(motor);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setPower(0.0);
            }
            operation = "INIT path construction";
            Pose start = startingPose();
            path = buildPath(start);
            operation = "INIT intake";
            intake = new IntakeSubsystem(hardwareMap);
            operation = "INIT shooter";
            shooter = new ShooterSubsystem(hardwareMap, nanoTime::getAsLong);
            operation = "INIT Pedro / Pinpoint";
            follower = Constants.create(hardwareMap);
            follower.setPose(start);
            operation = "INIT hub bulk cache";
            hubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            telemetry.setMsTransmissionInterval(50);
            operation = "INIT voltage polling";
            shooter.startVoltagePolling();
            stage = Stage.READY;
        } catch (RuntimeException | LinkageError error) {
            fail(error);
        }
    }

    @Override
    public void init_loop() {
        if (failureMessage != null) showFailure();
        else {
            telemetry.addData("Auto stage", stage);
            telemetry.addData("Shooter target RPM", SHOOTER_PRESET * ShooterSubsystem.MAX_MOTOR_RPM);
            telemetry.update();
        }
    }

    protected Pose startingPose() {
        return PoseFactory.degrees().of(57.01798561151079, 9.221582733812928, 270);
    }

    protected Path buildPath(Pose start) {
        Pose end = new Pose(8.96906474820144, 97.8935251798561, start.heading());
        // Drive the straight parking line without commanding a turn after shooting.
        return Paths.line(start, end).constant(start);
    }

    @Override
    public void start() {
        if (stage == Stage.FAILED) { showFailure(); return; }
        try {
            operation = "START";
            if (stage != Stage.READY) throw new IllegalStateException("Auto initialization is incomplete");
            feedingStartedNanos = -1L;
            telemetryRefresh.reset();
            // Stop outputs directly; do not require a successful sensor update to stop.
            follower.stop();
            follower.drivetrain.stop();
            stage = Stage.INITIAL_SHOT;
        } catch (RuntimeException | LinkageError error) {
            fail(error);
        }
    }

    @Override
    public void loop() {
        if (stage == Stage.FAILED) { if (telemetryRefresh.isDue()) showFailure(); return; }
        if (stage != Stage.INITIAL_SHOT && stage != Stage.DRIVING) return;
        try {
            operation = "LOOP hub bulk cache";
            for (LynxModule hub : hubs) hub.clearBulkCache();
            operation = "LOOP Pedro / Pinpoint";
            follower.update();
            if (stage == Stage.INITIAL_SHOT) {
                operation = "LOOP shooter / feed";
                if (updateShooting()) {
                    shooter.setFlywheelPower(0.0);
                    intake.setPower(0.0);
                    operation = "LOOP begin parking path";
                    follower.follow(path);
                    stage = Stage.DRIVING;
                }
            } else if (!follower.isBusy()) {
                stop();
            }

            if (!telemetryRefresh.isDue()) return;
            operation = "LOOP telemetry";
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
        } catch (RuntimeException | LinkageError error) {
            fail(error);
        }
    }

    /** Spin-up time is excluded: the two-second timer starts at measured readiness. */
    private boolean updateShooting() {
        shooter.updateShot(SHOOTER_PRESET, ShooterSubsystem.ControlMode.VELOCITY_LQR);
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
        stage = failureMessage == null ? Stage.DONE : Stage.FAILED;
        feedingStartedNanos = -1L;
        Throwable error = shutdown(null);
        if (error != null) {
            operation = "STOP";
            recordFailure(error);
        }
    }

    /** Attempt every output even if another device throws; stopping never reads odometry. */
    private Throwable shutdown(Throwable error) {
        if (shooter != null) error = attempt(error, () -> shooter.stopVoltagePolling());
        if (follower != null) {
            error = attempt(error, () -> follower.stop());
            error = attempt(error, () -> follower.drivetrain.stop());
        }
        for (DcMotorEx motor : stopMotors) error = attempt(error, () -> motor.setPower(0.0));
        if (intake != null) error = attempt(error, () -> intake.forceStop());
        if (shooter != null) error = attempt(error, () -> shooter.forceStop());
        if (hubs != null) {
            for (LynxModule hub : hubs) {
                error = attempt(error, () -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO));
            }
        }
        return error;
    }

    private Throwable attempt(Throwable first, Runnable action) {
        try { action.run(); }
        catch (RuntimeException | LinkageError next) {
            if (first == null) return next;
            if (first != next) first.addSuppressed(next);
        }
        return first;
    }

    private void fail(Throwable error) {
        stage = Stage.FAILED;
        recordFailure(shutdown(error));
    }

    private void recordFailure(Throwable error) {
        stage = Stage.FAILED;
        failureMessage = operation + ": " + error.getClass().getSimpleName() + ": " + error.getMessage();
        RobotLog.ee("BioBuzzAuto", error, failureMessage);
        showFailure();
    }

    private void showFailure() {
        try {
            telemetry.addData("Auto stage", Stage.FAILED);
            telemetry.addData("Auto error", failureMessage);
            telemetry.update();
        } catch (RuntimeException | LinkageError error) {
            // Preserve the original logged stack trace even if telemetry itself is unavailable.
            RobotLog.ee("BioBuzzAuto", error, "Unable to display auto failure");
        }
    }
}
