package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.function.LongSupplier;

/** A single diagonal forward-left move at half wheel power for a quarter second. */
@Autonomous(name = "MoveForwardLeft")
public class MoveForwardLeft extends OpMode {
    private static final long RUN_DURATION_NANOS = 250_000_000L;
    private final LongSupplier nanoTime;
    // Front left, front right, back left, back right.
    private DcMotor[] motors = new DcMotor[0];
    private long startedNanos;
    private boolean running;

    public MoveForwardLeft() {
        this(System::nanoTime);
    }

    MoveForwardLeft(LongSupplier nanoTime) {
        this.nanoTime = nanoTime;
    }

    @Override
    public void init() {
        motors = new DcMotor[] {
                hardwareMap.get(DcMotor.class, "driveFL"),
                hardwareMap.get(DcMotor.class, "driveFR"),
                hardwareMap.get(DcMotor.class, "driveBL"),
                hardwareMap.get(DcMotor.class, "driveBR")
        };
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(0.0);
            motors[i].setDirection(i == 0 || i == 2
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        running = false;
    }

    @Override
    public void start() {
        motors[0].setPower(0.0);
        motors[1].setPower(0.5);
        motors[2].setPower(0.5);
        motors[3].setPower(0.0);
        startedNanos = nanoTime.getAsLong();
        running = true;
    }

    @Override
    public void loop() {
        if (running && nanoTime.getAsLong() - startedNanos >= RUN_DURATION_NANOS) stop();
    }

    @Override
    public void stop() {
        running = false;
        for (DcMotor motor : motors) motor.setPower(0.0);
    }
}
