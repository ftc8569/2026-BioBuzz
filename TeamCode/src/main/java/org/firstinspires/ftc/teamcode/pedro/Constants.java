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
        // Tuned with PinpointTuner.
        c.xPodOffset.set(0.39301879762664554);
        c.yPodOffset.set(1.1028997165950265);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // Tuned with ForesightTuner.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2945314778514534);
                Controller secondaryTranslationalForward = Controller.proportional(0.10882151104007948);
                Controller primaryTranslationalLateral = Controller.proportional(0.4345129742529553);
                Controller secondaryTranslationalLateral = Controller.proportional(0.16054093358596314);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012359923590432979));
                c.brake.set(Controller.proportionalFeedforward(0.010505935051868032));

                c.headingFeedback.set(Controller.proportional(4.744505408571478));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04750431153429649, 0.005077158170629162));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06004411905705971, 0.05411067759136341));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001564179452025076, 0.0016074115423164556));

                c.maxAchievableForwardVelocity.set(78.81759823180964);
                c.maxAchievableStrafeVelocity.set(59.535486035266956);
                c.naturalForwardDeceleration.set(49.47675119801514);
                c.naturalStrafeDeceleration.set(71.56252916286806);
            }
    );
}