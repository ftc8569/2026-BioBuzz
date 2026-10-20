package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.Collections;
import java.util.List;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.teamcode.control.ShooterController;
import org.firstinspires.ftc.teamcode.diagnostics.TelemetryRefresh;
import org.firstinspires.ftc.teamcode.hardware.SafeHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/** Shared lifecycle: initialize fully, run only after START, stop every acquired output. */
public abstract class AutoOpMode extends OpMode {
    private enum State { UNINITIALIZED, READY, RUNNING, DONE, FAILED }
    private State state = State.UNINITIALIZED;
    private String failureMessage;
    private Throwable failure;
    protected String operation = "Not initialized";
    protected final LongSupplier nanoTime;
    protected Follower follower;
    protected IntakeSubsystem intake;
    protected ShooterSubsystem shooter;
    private final SafeHardware safety = new SafeHardware();
    private final TelemetryRefresh telemetryRefresh;
    private List<LynxModule> hubs = Collections.emptyList();

    protected AutoOpMode() { this(System::nanoTime); }
    protected AutoOpMode(LongSupplier nanoTime) {
        this.nanoTime = nanoTime;
        telemetryRefresh = new TelemetryRefresh(nanoTime::getAsLong);
    }

    protected boolean usesShooter() { return true; }
    protected abstract Pose startingPose();
    protected abstract void buildRoutine(Pose start);
    protected abstract void startRoutine();
    protected abstract void updateRoutine();
    protected void stopRoutine() { }
    protected String runningStage() { return "RUNNING"; }

    @Override public final void init() {
        guarded(() -> {
            if (usesShooter()) {
                operation = "INIT blocker";
                safety.closeBlocker(hardwareMap, ShooterController.BLOCKER_CLOSED);
            }
            operation = "INIT motor setup";
            safety.prepareMotors(hardwareMap, usesShooter());
            if (usesShooter()) {
                operation = "INIT intake";
                intake = new IntakeSubsystem(hardwareMap);
                operation = "INIT shooter";
                shooter = new ShooterSubsystem(hardwareMap, nanoTime::getAsLong);
            }
            operation = "INIT Pedro / Pinpoint";
            follower = Constants.create(hardwareMap);
            Pose start = startingPose();
            follower.setPose(start);
            operation = "INIT path construction";
            buildRoutine(start);
            operation = "INIT hub bulk cache";
            hubs = hardwareMap.getAll(LynxModule.class);
            safety.setCaching(hubs, LynxModule.BulkCachingMode.MANUAL);
            telemetry.setMsTransmissionInterval(50);
            operation = "INIT voltage polling";
            if (shooter != null) shooter.startVoltagePolling();
            state = State.READY;
        });
    }

    @Override public final void init_loop() {
        if (failure != null) { showFailure(); return; }
        guarded(() -> {
            operation = "INIT telemetry";
            telemetry.addData("Auto stage", state);
            if (shooter != null) telemetry.addData("Shooter target RPM",
                    ShooterController.DEFAULT_SHOOTER_PRESET * ShooterController.MAX_MOTOR_RPM);
            telemetry.update();
        });
    }

    @Override public final void start() {
        if (failure != null) { showFailure(); return; }
        guarded(() -> {
            operation = "START";
            if (state != State.READY) {
                throw new IllegalStateException("Auto initialization is incomplete");
            }
            telemetryRefresh.reset();
            follower.stop();
            follower.drivetrain.stop();
            startRoutine();
            state = State.RUNNING;
        });
    }

    @Override public final void loop() {
        if (failure != null) { if (telemetryRefresh.isDue()) showFailure(); return; }
        if (state != State.RUNNING) return;
        guarded(() -> {
            operation = "LOOP hub bulk cache";
            for (LynxModule hub : hubs) hub.clearBulkCache();
            updateRoutine();
            if (failure == null && telemetryRefresh.isDue()) {
                operation = "LOOP telemetry";
                telemetry.addData("Auto stage", state == State.RUNNING ? runningStage() : state);
                reportTelemetry();
                telemetry.update();
            }
        });
    }

    protected void reportTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("x", pose.x());
        telemetry.addData("y", pose.y());
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading()));
    }

    @Override public final void stop() {
        state = failure == null ? State.DONE : State.FAILED;
        Throwable error = shutdown(failure);
        if (error != null) recordFailure(error, "STOP");
    }

    private Throwable shutdown(Throwable error) {
        error = SafeHardware.attempt(error, this::stopRoutine);
        if (shooter != null) error = SafeHardware.attempt(error, shooter::stopVoltagePolling);
        if (follower != null) {
            error = SafeHardware.attempt(error, follower::stop);
            error = SafeHardware.attempt(error, follower.drivetrain::stop);
        }
        if (intake != null) error = SafeHardware.attempt(error, intake::forceStop);
        if (shooter != null) error = SafeHardware.attempt(error, shooter::forceStop);
        return safety.stop(error);
    }

    private void guarded(Runnable action) {
        try { action.run(); }
        catch (RuntimeException | LinkageError error) {
            recordFailure(shutdown(error), operation);
        }
    }

    private void recordFailure(Throwable error, String phase) {
        state = State.FAILED;
        if (failure == null) {
            failure = error;
            failureMessage = phase + ": " + error.getClass().getSimpleName()
                    + ": " + error.getMessage();
        }
        RobotLog.ee("BioBuzzAuto", failure, failureMessage);
        showFailure();
    }

    private void showFailure() {
        try {
            telemetry.addData("Auto stage", State.FAILED);
            telemetry.addData("Auto error", failureMessage);
            telemetry.update();
        } catch (RuntimeException | LinkageError error) {
            RobotLog.ee("BioBuzzAuto", error, "Unable to display auto failure");
        }
    }
}
