package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig));
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("driveFL");
        c.frontRightName.set("driveFR");
        c.backLeftName.set("driveBL");
        c.backRightName.set("driveBR");
        // FL and BL reversed; matches BioBuzzTeleOp.kt and Decode. Confirm with MecanumTuner.
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // Decode offsets (-80 mm, -157.475 mm) converted to inches.
        // TODO: confirm with PinpointTuner and paste its generated config over this one.
        c.xPodOffset.set(-3.1496);
        c.yPodOffset.set(-6.2000);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // TODO: every value below is a PLACEHOLDER. Run ForesightTuner and paste its generated config here.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.32431511367862054);
                Controller secondaryTranslationalForward = Controller.proportional(0.11982576864481138);
                Controller primaryTranslationalLateral = Controller.proportional(0.5072720435643365);
                Controller secondaryTranslationalLateral = Controller.proportional(0.18742346553838038);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013901114366289954));
                c.brake.set(Controller.proportionalFeedforward(0.011815947211346461));

                c.headingFeedback.set(Controller.proportional(5.6853899391811655));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04712896527669559, 0.005072253913791439));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10247947766923571, 0.055980780044920624));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001036673079825146, 0.0016948256712569288));

                c.maxAchievableForwardVelocity.set(71.56683990284671);
                c.maxAchievableStrafeVelocity.set(58.0385955628027);
                c.naturalForwardDeceleration.set(29.688873484671042);
                c.naturalStrafeDeceleration.set(68.8466311273792);
            }
    );
}