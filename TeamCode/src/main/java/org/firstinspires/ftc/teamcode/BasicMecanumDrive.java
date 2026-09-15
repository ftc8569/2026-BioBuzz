package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Basic Mecanum Drive")
public class BasicMecanumDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "driveFL");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "driveFR");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "driveBL");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "driveBR");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double frontLeftPower = drive + strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double backLeftPower = drive - strafe + turn;
            double backRightPower = drive + strafe - turn;

            double denominator = Math.max(
                    Math.abs(drive) + Math.abs(strafe) + Math.abs(turn),
                    1.0);

            frontLeft.setPower(frontLeftPower / denominator);
            frontRight.setPower(frontRightPower / denominator);
            backLeft.setPower(backLeftPower / denominator);
            backRight.setPower(backRightPower / denominator);
        }
    }
}
