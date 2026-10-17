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

    /** TeleOp supplies snapshot-only localization and its shared buffered motor owner. */
    public static Follower create(com.pedropathing.localization.Localizer localizer,
                                  com.pedropathing.drivetrain.Drivetrain drivetrain) {
        return new Follower(localizer, drivetrain, new Foresight(foresightConfig));
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
        c.xPodOffset.set(1.8827563007985515);
        c.yPodOffset.set(-5.472809799074189);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // TODO: every value below is a PLACEHOLDER. Run ForesightTuner and paste its generated config here.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.28906605721852313);
                Controller secondaryTranslationalForward = Controller.proportional(0.10680218415494071);
                Controller primaryTranslationalLateral = Controller.proportional(0.4724357900333263);
                Controller secondaryTranslationalLateral = Controller.proportional(0.17455240070051004);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011556465715348165));
                c.brake.set(Controller.proportionalFeedforward(0.00982299585804594));

                c.headingFeedback.set(Controller.proportional(4.230913749290777));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05623561406019874, 0.0044157663150905255));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10013157211093868, 0.06569808318998005));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0010291978739261542, 0.0016436628154132236));

                c.maxAchievableForwardVelocity.set(82.54744990234515);
                c.maxAchievableStrafeVelocity.set(69.03537164271943);
                c.naturalForwardDeceleration.set(43.902269745087004);
                c.naturalStrafeDeceleration.set(74.03666599198603);
            }
    );
}