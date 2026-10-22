package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.teamcode.subsystems.ShooterController;
import org.firstinspires.ftc.teamcode.subsystems.IntakeFeed;

/** Shoot while stationary, follow pedro paths/basicred.pp, then stop. */
@Autonomous(name = "BasicRed Auto")
public class BioBuzzAuto extends AutoOpMode {
    private static final double SHOOTER_PRESET = ShooterController.DEFAULT_SHOOTER_PRESET;
    private final IntakeFeed feed;
    private Long feedingStarted;
    private boolean driving;
    private Path path;

    public BioBuzzAuto() { this(System::nanoTime); }
    BioBuzzAuto(LongSupplier nanoTime) {
        super(nanoTime);
        feed = new IntakeFeed(nanoTime::getAsLong);
    }

    protected Pose startingPose() {
        return PoseFactory.degrees().of(57.01798561151079, 9.221582733812928, 270);
    }

    protected Path buildPath(Pose start) {
        Pose end = new Pose(8.96906474820144, 97.8935251798561, start.heading());
        // Drive the straight parking line without commanding a turn after shooting.
        return Paths.line(start, end).constant(start);
    }

    @Override protected void buildRoutine(Pose start) { path = buildPath(start); }

    @Override protected void startRoutine() {
        driving = false;
        feedingStarted = null;
        feed.reset();
    }

    @Override protected String runningStage() { return driving ? "DRIVING" : "INITIAL_SHOT"; }

    @Override protected void updateRoutine() {
        operation = "LOOP Pedro / Pinpoint";
        follower.update();
        if (driving) {
            if (!follower.isBusy()) stop();
            return;
        }
        operation = "LOOP shooter / feed";
        shooter.updateShot(SHOOTER_PRESET, ShooterController.ControlMode.VELOCITY_LQR);
        double power = feed.power(shooter, SHOOTER_PRESET);
        intake.setPower(power);
        if (power <= 0.0) { feedingStarted = null; return; }
        long now = nanoTime.getAsLong();
        if (feedingStarted == null) feedingStarted = now;
        if (now - feedingStarted < 2_000_000_000L) return;
        shooter.setFlywheelPower(0.0);
        intake.setPower(0.0);
        operation = "LOOP begin parking path";
        follower.follow(path);
        driving = true;
    }

    @Override protected void reportTelemetry() {
        telemetry.addData("LQR interval (ms)", shooter.getControl().getLqrIntervalMs());
        telemetry.addData("LQR frequency (Hz)", shooter.getControl().getLqrFrequencyHz());
        telemetry.addData("Shooter target RPM", shooter.getControl().getTargetRpm());
        telemetry.addData("Flywheel 1 RPM", shooter.getWheels().get(0).getRpm());
        telemetry.addData("Flywheel 2 RPM", shooter.getWheels().get(1).getRpm());
        telemetry.addData("Feed latched", shooter.getControl().getFeedReady());
        super.reportTelemetry();
    }
}
