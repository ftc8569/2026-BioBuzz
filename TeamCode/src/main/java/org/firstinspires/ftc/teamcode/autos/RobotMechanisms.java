package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsytem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem;

/**
 * The autonomous's intake and shooter, built on the TeleOp's subsystems (same motors, same PID shooter).
 *
 * How it works, copied from BioBuzzTeleOp / FeedControls:
 *  - Shooting: the flywheels run at SHOOTER_FRACTION in VELOCITY_PID mode; when ShooterSubsystem.feedReady is true the
 *    blocker is open and the intake motors feed balls at the shooter fraction. There is no ball sensor, so a shot
 *    of N balls feeds for N * FEED_SEC_PER_BALL seconds (counted from when feeding starts).
 *  - Collecting: flywheels off, intake on. (The blocker opens by itself when the flywheels are at speed, so the
 *    flywheels are stopped while the intake collects.)
 *  - Otherwise: flywheels pre-spin so a shot is ready on arrival; the intake stays off.
 * Not tested on a robot. TODO tune the three numbers below.
 */
public class RobotMechanisms implements Mechanisms {
    /** Flywheel speed as a fraction of full motor speed. 0.66 is the TeleOp's starting preset (33 steps x 0.02). */
    private static final double SHOOTER_FRACTION = 0.66;
    private static final double INTAKE_POWER = 1.0;
    /** How long the feed runs per ball. A guess; time one real ball. The sim's launcher takes 0.3 s per ball. */
    private static final double FEED_SEC_PER_BALL = 0.5;

    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final ElapsedTime feedTimer = new ElapsedTime();
    private boolean intakeOn = false;
    private boolean spin = false;
    private boolean feeding = false;
    private boolean feedStarted = false;
    private double feedSeconds = 0;
    private String status = "idle";

    public RobotMechanisms(HardwareMap hardwareMap) {
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
    }

    @Override public void intake(boolean on) { intakeOn = on; }
    @Override public void spinUpShooter() { spin = true; }

    @Override
    public void startShooting(int balls) {
        feeding = true;
        feedStarted = false;
        feedSeconds = balls * FEED_SEC_PER_BALL;
    }

    @Override public void stopShooting() { feeding = false; feedStarted = false; }

    @Override
    public void stopAll() {
        feeding = false;
        intakeOn = false;
        spin = false;
        shooter.setManualBlockerOpen(false);
        shooter.setFlywheelPower(0.0);
        intake.setPower(0.0);
        status = "stopped";
    }

    @Override
    public boolean shootingDone() {
        return feeding && feedStarted && feedTimer.seconds() >= feedSeconds;
    }

    @Override
    public void update() {
        if (feeding) {
            shooter.updateShot(SHOOTER_FRACTION, ShooterSubsystem.ControlMode.VELOCITY_PID);
            if (shooter.getFeedReady()) {
                if (!feedStarted) { feedStarted = true; feedTimer.reset(); }
                intake.setPower(SHOOTER_FRACTION);
                status = "feeding";
            } else {
                intake.setPower(0.0);
                status = "waiting for flywheel speed";
            }
        } else if (intakeOn) {
            shooter.setFlywheelPower(0.0);
            intake.setPower(INTAKE_POWER);
            status = "collecting";
        } else if (spin) {
            shooter.updateShot(SHOOTER_FRACTION, ShooterSubsystem.ControlMode.VELOCITY_PID);
            intake.setPower(0.0);
            status = "flywheel spinning";
        } else {
            shooter.setFlywheelPower(0.0);
            intake.setPower(0.0);
            status = "idle";
        }
    }

    @Override public String status() {
        return status + "  rpm " + Math.round(shooter.getMeasuredRpm1()) + "/" + Math.round(shooter.getMeasuredRpm2());
    }
}
