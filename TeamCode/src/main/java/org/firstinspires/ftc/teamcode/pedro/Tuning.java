package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.utils.Utils.linearFit;
import static com.pedropathing.utils.Utils.quadraticFit;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.*;
import com.pedropathing.tuning.autotune.Display.FourWheelBot.Wheel;
import com.pedropathing.tuning.autotune.DisplayName;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.*;
import java.util.function.Function;
import java.util.function.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** Pedro 3 tuner registry. Factories create procedures without opening hardware. */
public class Tuning {
  @Tuner
  public static Procedure otosTuner() {
    return new org.firstinspires.ftc.teamcode.pedro.procedures.OTOSTuner();
  }

  @Tuner
  public static Procedure octoQuadTuner() {
    return new org.firstinspires.ftc.teamcode.pedro.procedures.OctoQuadTuner();
  }

  @Tuner
  public static Procedure twoWheelTuner() {
    return new org.firstinspires.ftc.teamcode.pedro.procedures.TwoWheelTuner();
  }

  @Tuner
  public static Procedure threeWheelTuner() {
    return new org.firstinspires.ftc.teamcode.pedro.procedures.ThreeWheelTuner();
  }

  @Tuner
  public static Procedure threeWheelIMUTuner() {
    return new org.firstinspires.ftc.teamcode.pedro.procedures.ThreeWheelIMUTuner();
  }

  @Tuner
  public static Procedure mecanumTuner() {
    return new MecanumTuner();
  }

  @Tuner
  public static Procedure pinpointTuner() {
    return new PinpointTuner();
  }

  @Tuner
  public static Procedure foresightTuner() {
    return new ForesightTuner(
        (hardwareMap) -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
        (hardwareMap) -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
  }

  @Tuner
  public static Procedure tests() {
    return new Tests(
        hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
        hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
        () -> new Foresight(Constants.foresightConfig));
  }
}

enum Direction {
  @DisplayName("Forward")
  FORWARD,
  @DisplayName("Reversed")
  REVERSE
}

class MecanumTuner extends Procedure {
  public MecanumTuner() {
    super("Mecanum Tuner", "A procedure to find the directions of mecanum wheels.");
  }

  @Override
  public void run() throws InterruptedException {
    Inputs motorNames =
        inputs("Mecanum Motor Names", "Enter the names in HardwareMap of your drivetrain motors.");
    Inputs.Field<String> frontLeftName = motorNames.s("Front Left Name");
    Inputs.Field<String> frontRightName = motorNames.s("Front Right Name");
    Inputs.Field<String> backLeftName = motorNames.s("Back Left Name");
    Inputs.Field<String> backRightName = motorNames.s("Back Right Name");
    awaitInputs(motorNames);

    confirmation(
        "Motor Directions",
        "Each drivetrain motor will spin, one at a time. After each one, you will enter whether it"
            + " spun forward or reversed. You may use the interactive diagram to see which wheel"
            + " should be spinning and which direction is forward.");

    Direction frontLeftDirection = testMotor(Wheel.FRONT_LEFT, "Front Left", frontLeftName.get());
    Direction frontRightDirection =
        testMotor(Wheel.FRONT_RIGHT, "Front Right", frontRightName.get());
    Direction backLeftDirection = testMotor(Wheel.BACK_LEFT, "Back Left", backLeftName.get());
    Direction backRightDirection = testMotor(Wheel.BACK_RIGHT, "Back Right", backRightName.get());

    result("frontLeftName", frontLeftName.get());
    result("frontRightName", frontRightName.get());
    result("backLeftName", backLeftName.get());
    result("backRightName", backRightName.get());
    result("frontLeftDirection", frontLeftDirection);
    result("frontRightDirection", frontRightDirection);
    result("backLeftDirection", backLeftDirection);
    result("backRightDirection", backRightDirection);

    code(
        Language.JAVA,
        "public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {\n"
            + "    c.frontLeftName.set(\""
            + frontLeftName.get()
            + "\");\n"
            + "    c.frontRightName.set(\""
            + frontRightName.get()
            + "\");\n"
            + "    c.backLeftName.set(\""
            + backLeftName.get()
            + "\");\n"
            + "    c.backRightName.set(\""
            + backRightName.get()
            + "\");\n"
            + "    c.frontLeftDirection.set(DcMotorSimple.Direction."
            + frontLeftDirection
            + ");\n"
            + "    c.frontRightDirection.set(DcMotorSimple.Direction."
            + frontRightDirection
            + ");\n"
            + "    c.backLeftDirection.set(DcMotorSimple.Direction."
            + backLeftDirection
            + ");\n"
            + "    c.backRightDirection.set(DcMotorSimple.Direction."
            + backRightDirection
            + ");\n"
            + "});");
  }

  private Direction testMotor(Wheel wheel, String displayName, String hardwareName)
      throws InterruptedException {
    final boolean[] correctMotor = new boolean[1];
    final Direction[] direction = new Direction[1];

    withDisplay(
        Display.fourWheelBot(wheel, false),
        () -> {
          runOpMode(new SpinMotor(displayName, hardwareName));

          Inputs inputs =
              inputs(
                  displayName, "Determine the " + displayName.toLowerCase(Locale.ROOT) + " motor direction.");
          Inputs.Field<Boolean> correctMotorField =
              inputs.b("Did the " + displayName.toLowerCase(Locale.ROOT) + " motor spin?").withDefault(true);
          Inputs.Field<Direction> directionField =
              inputs.e("Which way did the motor spin?", Direction.class);
          awaitInputs(inputs);

          correctMotor[0] = correctMotorField.get();
          direction[0] = directionField.get();
        });

    if (!correctMotor[0])
      abort(
          "The wrong motor spun. Check that your motors are plugged into the correct ports, and"
              + " that they are configured correctly. Then, try again.");

    return direction[0];
  }
}

class SpinMotor extends TuningOpMode<Void> {
  private final String name;

  public SpinMotor(String displayName, String hardwareName) {
    super(
        displayName,
        "The "
            + displayName.toLowerCase(Locale.ROOT)
            + " motor will spin. The interactive diagram shows which way is forward. Click stop"
            + " when you know if it is spinning forward or reversed.",
        true);
    this.name = hardwareName;
  }

  @SuppressWarnings("StatementWithEmptyBody")
  @Override
  protected Void runTuningOpMode() {
    DcMotor motor = hardwareMap.dcMotor.get(name);
    waitForStart();
    motor.setPower(0.5);
    while (opModeIsActive()) {}
    motor.setPower(0);
    return null;
  }
}

class PinpointTuner extends Procedure {
  enum PodType {
    SWING_ARM,
    FOUR_BAR,
    CUSTOM
  }

  public PinpointTuner() {
    super("Pinpoint Tuner", "A procedure for tuning the Pinpoint localizer.");
  }

  @Override
  public void run() throws InterruptedException {
    Inputs inputs = inputs("Setup", "Set Pinpoint HardwareMap Name and Odometry Pod Type");
    Inputs.Field<String> pinpointName = inputs.s("HardwareMap Name").withDefault("pinpoint");
    Inputs.Field<PodType> podType =
        inputs.e("Odometry Pod Type", PodType.class).withDefault(PodType.FOUR_BAR);
    awaitInputs(inputs);

    OptionalDouble customPodScalar = OptionalDouble.empty();

    if (podType.get() == PodType.CUSTOM) {
      Inputs inputsCustom =
          inputs(
              "Custom Scalar Identification Push Distance",
              "Set the distance you will push your robot forward in inches");
      Inputs.Field<Double> distance = inputsCustom.d("Distance").withDefault(48.0);
      awaitInputs(inputsCustom);
      customPodScalar =
          OptionalDouble.of(
              runOpMode(new PinpointCustomPodScalar(distance.get(), pinpointName.get())));
    }

    boolean forwardPodReversed =
        runOpMode(new PinpointForwardDirection(pinpointName.get(), podType.get(), customPodScalar));
    boolean strafePodReversed =
        runOpMode(new PinpointStrafeDirection(pinpointName.get(), podType.get(), customPodScalar));

    List<Double> offsets =
        runOpMode(
            new PinpointOffsets(
                pinpointName.get(),
                podType.get(),
                customPodScalar,
                forwardPodReversed,
                strafePodReversed));

    result("name", pinpointName.get());

    if (customPodScalar.isPresent()) {
      result("podType", "Custom");
      result("ticksPerUnit", customPodScalar.getAsDouble());
    } else {
      result(
          "podType",
          podType.get() == PodType.SWING_ARM
              ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
              : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    }

    result(
        "xPodDirection",
        forwardPodReversed
            ? GoBildaPinpointDriver.EncoderDirection.REVERSED
            : GoBildaPinpointDriver.EncoderDirection.FORWARD);
    result(
        "yPodDirection",
        strafePodReversed
            ? GoBildaPinpointDriver.EncoderDirection.REVERSED
            : GoBildaPinpointDriver.EncoderDirection.FORWARD);
    result("xPodOffset", offsets.get(0));
    result("yPodOffset", offsets.get(1));

    code(
        Language.JAVA,
        "public static PinpointConfig localizerConfig = new PinpointConfig(c -> {\n"
            + "    c.name.set(\""
            + pinpointName.get()
            + "\");\n"
            + (customPodScalar.isPresent()
                ? "    c.ticksPerUnit.set(OptionalDouble.of("
                    + customPodScalar.getAsDouble()
                    + "));\n"
                : "    c.podType.set("
                    + (podType.get() == PodType.SWING_ARM
                        ? "GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD"
                        : "GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD")
                    + ");\n")
            + "    c.xPodOffset.set("
            + offsets.get(0)
            + ");\n"
            + "    c.yPodOffset.set("
            + offsets.get(1)
            + ");\n"
            + "    c.xPodDirection.set("
            + (forwardPodReversed
                ? "GoBildaPinpointDriver.EncoderDirection.REVERSED"
                : "GoBildaPinpointDriver.EncoderDirection.FORWARD")
            + ");\n"
            + "    c.yPodDirection.set("
            + (strafePodReversed
                ? "GoBildaPinpointDriver.EncoderDirection.REVERSED"
                : "GoBildaPinpointDriver.EncoderDirection.FORWARD")
            + ");\n"
            + "    c.globalDistanceUnit.set(DistanceUnit.INCH);\n"
            + "    c.offsetUnits.set(DistanceUnit.INCH);\n"
            + "});");
  }
}

class PinpointCustomPodScalar extends TuningOpMode<Double> {

  String name;
  double distance;

  public PinpointCustomPodScalar(Double distance, String name) {
    super(
        "Custom Scalar Identification",
        "Determines the scalar for the custom pods of the Pinpoint localizer. \n"
            + "Push your robot forward "
            + distance
            + " inches exactly and then stop the Opmode",
        true);
    this.name = name;
    this.distance = distance;
  }

  @Override
  protected Double runTuningOpMode() throws InterruptedException {
    PinpointConfig config =
        new PinpointConfig(
            c -> {
              c.name.set(name);
              c.ticksPerUnit.set(OptionalDouble.of(1.0));
              c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
              c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
              c.xPodOffset.set(0.0);
              c.yPodOffset.set(0.0);
            });
    PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, config);
    localizer.setPose(new Pose(0, 0));
    Thread.sleep(1000);
    waitForStart();
    localizer.setPose(Pose.zero());
    localizer.update();
    while (!isStopRequested()) {
      localizer.update();
    }
    return Math.abs((localizer.pose().x() / distance));
  }
}

class PinpointForwardDirection extends TuningOpMode<Boolean> {
  String name;
  PinpointTuner.PodType podType;
  OptionalDouble customPodScalar;

  public PinpointForwardDirection(
      String name, PinpointTuner.PodType podType, OptionalDouble customPodScalar) {
    super(
        "Forward Direction Identification",
        "Determines if your forward pod needs to be reversed. \n"
            + "Push your robot forward and then stop the Opmode",
        true);
    this.name = name;
    this.podType = podType;
    this.customPodScalar = customPodScalar;
  }

  @Override
  protected Boolean runTuningOpMode() throws InterruptedException {
    PinpointConfig config =
        new PinpointConfig(
            c -> {
              c.name.set(name);
              c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
              c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
              c.xPodOffset.set(0.0);
              c.yPodOffset.set(0.0);
              if (customPodScalar.isPresent()) {
                c.encoderResolutionUnit.set(DistanceUnit.INCH);
                c.ticksPerUnit.set(OptionalDouble.of(customPodScalar.getAsDouble()));
              } else {
                c.podType.set(
                    podType == PinpointTuner.PodType.SWING_ARM
                        ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
                        : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
              }
            });
    PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, config);
    localizer.setPose(new Pose(0, 0));
    Thread.sleep(1000);
    waitForStart();
    localizer.setPose(Pose.zero());
    localizer.update();

    // Keep the last reading taken before Stop. After Stop the Pinpoint reports a reset
    // pose (x = 0), so reading it afterwards always returns false (not reversed).
    double x = 0;
    while (!isStopRequested()) {
      localizer.update();
      if (!isStopRequested()) {
        x = localizer.pose().x();
      }
    }

    return x < 0;
  }
}

class PinpointStrafeDirection extends TuningOpMode<Boolean> {
  String name;
  PinpointTuner.PodType podType;
  OptionalDouble customPodScalar;

  public PinpointStrafeDirection(
      String name, PinpointTuner.PodType podType, OptionalDouble customPodScalar) {
    super(
        "Strafe Direction Identification",
        "Determines if your strafe pod needs to be reversed. \n"
            + "Push your robot to the left and then stop the Opmode",
        true);
    this.name = name;
    this.podType = podType;
    this.customPodScalar = customPodScalar;
  }

  @Override
  protected Boolean runTuningOpMode() throws InterruptedException {
    PinpointConfig config =
        new PinpointConfig(
            c -> {
              c.name.set(name);
              c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
              c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
              c.xPodOffset.set(0.0);
              c.yPodOffset.set(0.0);
              if (customPodScalar.isPresent()) {
                c.encoderResolutionUnit.set(DistanceUnit.INCH);
                c.ticksPerUnit.set(OptionalDouble.of(customPodScalar.getAsDouble()));
              } else {
                c.podType.set(
                    podType == PinpointTuner.PodType.SWING_ARM
                        ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
                        : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
              }
            });
    PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, config);
    localizer.setPose(new Pose(0, 0));
    Thread.sleep(1000);
    waitForStart();
    localizer.setPose(Pose.zero());
    localizer.update();
    // Keep the last reading taken before Stop. After Stop the Pinpoint reports a reset
    // pose (y = 0), so reading it afterwards always returns false (not reversed).
    double y = 0;
    while (!isStopRequested()) {
      localizer.update();
      if (!isStopRequested()) {
        y = localizer.pose().y();
      }
    }

    return y < 0;
  }
}

class PinpointOffsets extends TuningOpMode<List<Double>> {
  String name;
  PinpointTuner.PodType podType;
  OptionalDouble customPodScalar = OptionalDouble.empty();
  boolean forwardPodReversed, strafePodReversed;
  private Pose previous = Pose.zero();

  public PinpointOffsets(
      String name,
      PinpointTuner.PodType podType,
      OptionalDouble customPodScalar,
      Boolean forwardPodReversed,
      Boolean strafePodReversed) {
    super(
        "Offsets Identification",
        "Automatically identifies the offsets for your Pinpoint localizer. \n"
            + "Spin your robot in place 180 degrees counterclockwise and then stop the Opmode",
        true);
    this.name = name;
    this.podType = podType;
    if (customPodScalar.isPresent()) {
      this.customPodScalar = customPodScalar;
    }
    this.forwardPodReversed = forwardPodReversed;
    this.strafePodReversed = strafePodReversed;
  }

  @Override
  protected List<Double> runTuningOpMode() throws InterruptedException {
    PinpointConfig config =
        new PinpointConfig(
            c -> {
              c.name.set(name);
              c.xPodDirection.set(
                  forwardPodReversed
                      ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                      : GoBildaPinpointDriver.EncoderDirection.FORWARD);
              c.yPodDirection.set(
                  strafePodReversed
                      ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                      : GoBildaPinpointDriver.EncoderDirection.FORWARD);
              if (customPodScalar.isPresent()) {
                c.encoderResolutionUnit.set(DistanceUnit.INCH);
                c.ticksPerUnit.set(OptionalDouble.of(customPodScalar.getAsDouble()));
                c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
              } else {
                c.podType.set(
                    podType == PinpointTuner.PodType.SWING_ARM
                        ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
                        : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
              }
              c.xPodOffset.set(0.0);
              c.yPodOffset.set(0.0);
              c.globalDistanceUnit.set(DistanceUnit.INCH);
              c.offsetUnits.set(DistanceUnit.INCH);
            });
    PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, config);
    if (customPodScalar.isPresent()) {
      localizer.reset();
    }
    localizer.setPose(Pose.zero());
    localizer.update();

    Thread.sleep(1000);
    waitForStart();
    localizer.setPose(Pose.zero());
    localizer.update();

    localizer.setPose(Pose.zero());

    while (!isStopRequested()) {
      previous = localizer.pose();
      localizer.update();

      telemetry.addData("heading", localizer.pose().heading());
      telemetry.addData("pose", localizer.pose());
      telemetry.addData("previous", previous);
      telemetry.update();
    }

    if (localizer.pose().x() != Pose.zero().x() || localizer.pose().y() != Pose.zero().y()) {
      previous = localizer.pose();
    }

    return List.of(((-previous.y()) / 2.0), ((-previous.x()) / 2.0));
  }
}

class ForesightTuner extends Procedure {
  Function<HardwareMap, Localizer> localizerFunction;
  Function<HardwareMap, Drivetrain> drivetrainFunction;

  public ForesightTuner(
      Function<HardwareMap, Localizer> localizerFunction,
      Function<HardwareMap, Drivetrain> drivetrainFunction) {
    super("Foresight Tuner", "A procedure for tuning the Foresight Algorithm.");
    this.localizerFunction = localizerFunction;
    this.drivetrainFunction = drivetrainFunction;
  }

  @Override
  public void run() throws InterruptedException {
    Inputs distanceInput =
        inputs(
            "Distance",
            "The distance to drive in inches for the Max Achievable Forward and Strafe"
                + " Identifiers");
    Inputs.Field<Double> distance = distanceInput.d("Distance").withDefault(48.0);
    awaitInputs(distanceInput);

    double forwardVelocity =
        runOpMode(new ForwardVelocity(localizerFunction, drivetrainFunction, distance.get()));
    double strafeVelocity =
        runOpMode(new StrafeVelocity(localizerFunction, drivetrainFunction, distance.get()));

    Inputs velocityInput =
        inputs(
            "Velocity",
            "The velocity to drive to in inches for the Max Achievable Forward and Strafe"
                + " Deceleration Identifiers");
    Inputs.Field<Double> velocity = velocityInput.d("Velocity").withDefault(30.0);
    awaitInputs(velocityInput);

    double forwardDeceleration =
        runOpMode(new ForwardDeceleration(localizerFunction, drivetrainFunction, velocity.get()));
    double strafeDeceleration =
        runOpMode(new StrafeDeceleration(localizerFunction, drivetrainFunction, velocity.get()));

    List<Double> headingBraking =
        runOpMode(new HeadingBraking(localizerFunction, drivetrainFunction));
    double heading = runOpMode(new HeadingTuner(localizerFunction, drivetrainFunction));

    double headingLinear = headingBraking.get(0);
    double headingQuadratic = headingBraking.get(1);

    Inputs distanceBrakingInput =
        inputs(
            "Distance",
            "The distance to drive in inches for the Forward and Strafe Braking Identifiers."
                + " Distance must be at least 15 inches for accurate results.");
    Inputs.Field<Double> distanceBraking = distanceBrakingInput.d("Distance").withDefault(36.0);
    awaitInputs(distanceBrakingInput);
    double safeDistanceBraking = Math.max(distanceBraking.get(), 15.0);

    List<Double> forwardBraking =
        runOpMode(
            new ForwardBraking(
                localizerFunction,
                drivetrainFunction,
                headingLinear,
                headingQuadratic,
                heading,
                safeDistanceBraking));
    List<Double> strafeBraking =
        runOpMode(
            new StrafeBraking(
                localizerFunction,
                drivetrainFunction,
                headingLinear,
                headingQuadratic,
                heading,
                safeDistanceBraking));

    double forwardLinear = forwardBraking.get(0);
    double forwardQuadratic = forwardBraking.get(1);
    double strafeLinear = strafeBraking.get(0);
    double strafeQuadratic = strafeBraking.get(1);

    List<Double> forwardTranslational =
        runOpMode(new ForwardTranslational(localizerFunction, drivetrainFunction));
    List<Double> strafeTranslational =
        runOpMode(new StrafeTranslational(localizerFunction, drivetrainFunction));

    double forwardTranslationalPrimary = forwardTranslational.get(0);
    double forwardTranslationalSecondary = forwardTranslational.get(1);
    double coast = forwardTranslational.get(2);
    double brake = forwardTranslational.get(3);

    double strafeTranslationalPrimary = strafeTranslational.get(0);
    double strafeTranslationalSecondary = strafeTranslational.get(1);

    result("maxAchievableForwardVelocity", forwardVelocity);
    result("maxAchievableStrafeVelocity", strafeVelocity);
    result("naturalForwardDeceleration", forwardDeceleration);
    result("naturalStrafeDeceleration", strafeDeceleration);
    result("headingBrakingLinearCoefficient", headingLinear);
    result("headingBrakingQuadraticCoefficient", headingQuadratic);
    result("heading kP", heading);
    result("forwardBrakingLinearCoefficient", forwardLinear);
    result("forwardBrakingQuadraticCoefficient", forwardQuadratic);
    result("strafeBrakingLinearCoefficient", strafeLinear);
    result("strafeBrakingQuadraticCoefficient", strafeQuadratic);
    result("forwardTranslational Primary kP", forwardTranslationalPrimary);
    result("forwardTranslational Secondary kP", forwardTranslationalSecondary);
    result("strafeTranslational Primary kP", strafeTranslationalPrimary);
    result("strafeTranslational Secondary kP", strafeTranslationalSecondary);
    result("coast kV", coast);
    result("brake kV", brake);

    code(
        Language.JAVA,
        "public static ForesightConfig foresightConfig = new ForesightConfig(\n"
            + "            c -> {\n"
            + "                Controller primaryTranslationalForward = Controller.proportional("
            + forwardTranslationalPrimary
            + ");\n"
            + "                Controller secondaryTranslationalForward = Controller.proportional("
            + forwardTranslationalSecondary
            + ");\n"
            + "                Controller primaryTranslationalLateral = Controller.proportional("
            + strafeTranslationalPrimary
            + ");\n"
            + "                Controller secondaryTranslationalLateral = Controller.proportional("
            + strafeTranslationalSecondary
            + ");\n"
            + "\n"
            + "               "
            + " c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward)"
            + ".put(2.5,"
            + " primaryTranslationalForward));\n"
            + "               "
            + " c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral)"
            + ".put(2.5,"
            + " primaryTranslationalLateral));\n"
            + "\n"
            + "                c.coast.set(Controller.proportionalFeedforward("
            + coast
            + "));\n"
            + "                c.brake.set(Controller.proportionalFeedforward("
            + brake
            + "));\n"
            + "\n"
            + "                c.headingFeedback.set(Controller.proportional("
            + heading
            + "));\n"
            + "                c.headingBrakeCoefficients.set(Vector2D.cartesian("
            + headingLinear
            + ", "
            + headingQuadratic
            + "));\n"
            + "\n"
            + "                c.linearBrakeCoefficients.set(Matrix.diag("
            + forwardLinear
            + ", "
            + strafeLinear
            + "));\n"
            + "                c.quadraticBrakeCoefficients.set(Matrix.diag("
            + forwardQuadratic
            + ", "
            + strafeQuadratic
            + "));\n"
            + "\n"
            + "                c.maxAchievableForwardVelocity.set("
            + forwardVelocity
            + ");\n"
            + "                c.maxAchievableStrafeVelocity.set("
            + strafeVelocity
            + ");\n"
            + "                c.naturalForwardDeceleration.set("
            + forwardDeceleration
            + ");\n"
            + "                c.naturalStrafeDeceleration.set("
            + strafeDeceleration
            + ");\n"
            + "            }\n"
            + "    );");
  }
}

class ForwardVelocity extends TuningOpMode<Double> {
  Function<HardwareMap, Localizer> localizerFunction;
  Function<HardwareMap, Drivetrain> drivetrainFunction;
  double distance;
  private final ArrayDeque<Double> velocities = new ArrayDeque<>();
  public static double RECORD_NUMBER = 10;

  public ForwardVelocity(
      Function<HardwareMap, Localizer> localizerFunction,
      Function<HardwareMap, Drivetrain> drivetrainFunction,
      double distance) {
    super(
        "Max Forward Velocity",
        "A tuner for finding the maximum achievable forward velocity. This will drive forward for "
            + distance
            + " inches and then likely drift past that position.",
        false);
    this.localizerFunction = localizerFunction;
    this.drivetrainFunction = drivetrainFunction;
    this.distance = distance;
  }

  @Override
  protected Double runTuningOpMode() throws InterruptedException {
    Localizer localizer = localizerFunction.apply(hardwareMap);
    Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

    boolean end = false;

    localizer.setPose(Pose.zero());
    localizer.update();

    DrivePowers power = new DrivePowers(1, 0, 0);

    for (int i = 0; i < RECORD_NUMBER; i++) {
      velocities.add(0.0);
    }

    Thread.sleep(1000);
    waitForStart();
    localizer.setPose(Pose.zero());
    localizer.update();

    while (!end) {
      localizer.update();
      if (Math.abs(localizer.pose().x()) > distance) {
        end = true;
        drivetrain.stop();
      } else {
        drivetrain.drive(power, true);
        double currentVelocity = Math.abs(localizer.twist().toVector2D().x());
        velocities.addLast(currentVelocity);
        velocities.removeFirst();
      }
    }

    drivetrain.stop();
    double average = 0;
    for (double velocity : velocities) {
      average += velocity;
    }
    average /= velocities.size();
    return average;
  }
}

class StrafeVelocity extends TuningOpMode<Double> {
  Function<HardwareMap, Localizer> localizerFunction;
  Function<HardwareMap, Drivetrain> drivetrainFunction;
  double distance;
  private final ArrayDeque<Double> velocities = new ArrayDeque<>();
  public static double RECORD_NUMBER = 10;

  public StrafeVelocity(
      Function<HardwareMap, Localizer> localizerFunction,
      Function<HardwareMap, Drivetrain> drivetrainFunction,
      double distance) {
    super(
        "Max Strafe Velocity",
        "A tuner for finding the maximum achievable strafe velocity. This will drive left for "
            + distance
            + " inches and then likely drift past that position.",
        false);
    this.localizerFunction = localizerFunction;
    this.drivetrainFunction = drivetrainFunction;
    this.distance = distance;
  }

  @Override
  protected Double runTuningOpMode() throws InterruptedException {
    Localizer localizer = localizerFunction.apply(hardwareMap);
    Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

    boolean end = false;

    localizer.setPose(Pose.zero());
    localizer.update();

    DrivePowers power = new DrivePowers(0, 1, 0);

    for (int i = 0; i < RECORD_NUMBER; i++) {
      velocities.add(0.0);
    }

    Thread.sleep(1000);
    waitForStart();
    localizer.setPose(Pose.zero());
    localizer.update();

    while (!end) {
      localizer.update();
      if (Math.abs(localizer.pose().y()) > distance) {
        end = true;
        drivetrain.stop();
      } else {
        drivetrain.drive(power, false);
        double currentVelocity = Math.abs(localizer.twist().toVector2D().y());
        velocities.addLast(currentVelocity);
        velocities.removeFirst();
      }
    }

    drivetrain.stop();
    double average = 0;
    for (double velocity : velocities) {
      average += velocity;
    }
    average /= velocities.size();
    return average;
  }
}

class ForwardDeceleration extends TuningOpMode<Double> {
  Function<HardwareMap, Localizer> localizerFunction;
  Function<HardwareMap, Drivetrain> drivetrainFunction;
  double velocity;

  private final ArrayList<Double> accelerations = new ArrayList<>();

  private double previousVelocity;
  private long previousTimeNano;
  private boolean stopping;

  public ForwardDeceleration(
      Function<HardwareMap, Localizer> localizerFunction,
      Function<HardwareMap, Drivetrain> drivetrainFunction,
      double velocity) {
    super(
        "Forward Deceleration",
        "A tuner for finding the deceleration of the robot when moving forward. This will move"
            + " forward until it reaches "
            + velocity
            + " inches per second.",
        false);

    this.localizerFunction = localizerFunction;
    this.drivetrainFunction = drivetrainFunction;
    this.velocity = velocity;
  }

  @Override
  protected Double runTuningOpMode() throws InterruptedException {
    Localizer localizer = localizerFunction.apply(hardwareMap);
    Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

    accelerations.clear();
    previousVelocity = 0;
    previousTimeNano = 0;
    stopping = false;

    localizer.setPose(Pose.zero());
    localizer.update();

    DrivePowers power = new DrivePowers(1, 0, 0);
    Thread.sleep(1000);
    waitForStart();
    localizer.setPose(Pose.zero());
    localizer.update();

    drivetrain.drive(power, false);

    while (!stopping) {
      localizer.update();
      double currentVelocity = localizer.twist().toVector2D().x();
      if (Math.abs(currentVelocity) > velocity) {
        previousVelocity = currentVelocity;
        previousTimeNano = System.nanoTime();

        stopping = true;
        drivetrain.stop(false);
      } else {
        drivetrain.drive(power, false);
      }
    }

    boolean end = false;

    while (!end) {
      localizer.update();
      drivetrain.stop(false);
      double currentVelocity = localizer.twist().toVector2D().x();
      long currentTimeNano = System.nanoTime();
      double dt = (currentTimeNano - previousTimeNano) / 1e9;

      if (dt > 0) {
        double acceleration = (currentVelocity - previousVelocity) / dt;
        accelerations.add(acceleration);
      }

      previousVelocity = currentVelocity;
      previousTimeNano = currentTimeNano;

      if (Math.abs(currentVelocity) <= 1) {
        end = true;
      }
    }

    drivetrain.stop(false);

    double average = 0;

    for (double acceleration : accelerations) {
      average += acceleration;
    }

    if (accelerations.isEmpty()) {
      return 0.0;
    }

    average /= accelerations.size();

    return Math.abs(average);
  }
}

class StrafeDeceleration extends TuningOpMode<Double> {
  Function<HardwareMap, Localizer> localizerFunction;
  Function<HardwareMap, Drivetrain> drivetrainFunction;
  double velocity;

  private final ArrayList<Double> accelerations = new ArrayList<>();

  private double previousVelocity;
  private long previousTimeNano;
  private boolean stopping;

  public StrafeDeceleration(
      Function<HardwareMap, Localizer> localizerFunction,
      Function<HardwareMap, Drivetrain> drivetrainFunction,
      double velocity) {
    super(
        "Strafe Deceleration",
        "A tuner for finding the deceleration of the robot when moving laterally. This will drive"
            + " left until it reaches "
            + velocity
            + " inches per second.",
        false);

    this.localizerFunction = localizerFunction;
    this.drivetrainFunction = drivetrainFunction;
    this.velocity = velocity;
  }

  @Override
  protected Double runTuningOpMode() throws InterruptedException {
    Localizer localizer = localizerFunction.apply(hardwareMap);
    Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

    accelerations.clear();
    previousVelocity = 0;
    previousTimeNano = 0;
    stopping = false;

    localizer.setPose(Pose.zero());
    localizer.update();

    DrivePowers power = new DrivePowers(0, 1, 0);
    Thread.sleep(1000);
    waitForStart();
    localizer.setPose(Pose.zero());
    localizer.update();

    drivetrain.drive(power, false);

    while (!stopping) {
      localizer.update();
      double currentVelocity = localizer.twist().toVector2D().y();
      if (Math.abs(currentVelocity) > velocity) {
        previousVelocity = currentVelocity;
        previousTimeNano = System.nanoTime();

        stopping = true;
        drivetrain.stop(false);
      } else {
        drivetrain.drive(power, false);
      }
    }

    boolean end = false;

    while (!end) {
      drivetrain.stop(false);
      localizer.update();
      double currentVelocity = localizer.twist().toVector2D().y();
      long currentTimeNano = System.nanoTime();
      double dt = (currentTimeNano - previousTimeNano) / 1e9;

      if (dt > 0) {
        double acceleration = (currentVelocity - previousVelocity) / dt;
        accelerations.add(acceleration);
      }

      previousVelocity = currentVelocity;
      previousTimeNano = currentTimeNano;

      if (Math.abs(currentVelocity) <= 1) {
        end = true;
      }
    }

    drivetrain.stop(false);

    double average = 0;

    for (double acceleration : accelerations) {
      average += acceleration;
    }

    if (accelerations.isEmpty()) {
      return 0.0;
    }

    average /= accelerations.size();

    return Math.abs(average);
  }
}

class HeadingBraking extends TuningOpMode<List<Double>> {
  Function<HardwareMap, Localizer> localizerFunction;
  Function<HardwareMap, Drivetrain> drivetrainFunction;

  private static double[] POWERS;
  public static double MAX_BRAKE_TIME =
      3; // seconds, the robot shouldn't take longer than this to brake

  public static int trials = 12;
  public static double maxPower = 1;
  public static double minPower = 0.2;
  public static double bias = 1.5; // how much it favors doing trials with higher powers
  public static double brakingPower = 0.001;

  private final ElapsedTime timer = new ElapsedTime();

  private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
  private State state = State.DRIVE;
  private int iteration = 0;
  private int direction;
  private double power;

  private double startHeading;
  private double measuredVelocity;
  private double totalHeading;
  private double previousHeading;

  //    private VoltageSensor voltageSensor;

  public HeadingBraking(
      Function<HardwareMap, Localizer> localizerFunction,
      Function<HardwareMap, Drivetrain> drivetrainFunction) {
    super(
        "Heading Braking",
        "A tuner for finding the Heading Braking Coefficients. The robot will turn back at forth at"
            + " various speed levels.",
        false);

    this.localizerFunction = localizerFunction;
    this.drivetrainFunction = drivetrainFunction;
  }

  @Override
  protected List<Double> runTuningOpMode() throws InterruptedException {
    Localizer localizer = localizerFunction.apply(hardwareMap);
    Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

    localizer.setPose(Pose.zero());
    localizer.update();

    List<Double> coefficients = Collections.emptyList();

    POWERS = biasedGradient(trials, maxPower, minPower, bias);

    Thread.sleep(1000);
    waitForStart();
    localizer.setPose(Pose.zero());
    localizer.update();
    timer.reset();

    while (state != State.DONE && !isStopRequested()) {
      localizer.update();
      double currentHeading = localizer.pose().heading();
      totalHeading += Angle.normalizeSigned(currentHeading - previousHeading);
      previousHeading = currentHeading;

      direction = (iteration % 2 == 0) ? 1 : -1;
      if (iteration < POWERS.length) {
        power = POWERS[iteration];
      }

      //            if (state != State.DONE) {
      //                double voltage = voltageSensor.getVoltage();
      //                double duty = state == State.BRAKE ? -brakingPower * direction: power *
      // direction;
      //                double appliedVoltage = voltage * duty;
      //            }

      switch (state) {
        case DRIVE:
          {
            if (timer.seconds() > 2) {
              startHeading = totalHeading;
              measuredVelocity = Math.abs(localizer.velocity().omega);

              drivetrain.drive(new DrivePowers(0.0, 0.0, -brakingPower * direction), false);
              state = State.BRAKE;
              timer.reset();
              break;
            }
            drivetrain.drive(new DrivePowers(0.0, 0.0, power * direction), false);
            break;
          }
        case BRAKE:
          {
            if (Math.abs(localizer.velocity().omega) > 0.001 && timer.seconds() < MAX_BRAKE_TIME) {
              drivetrain.drive(new DrivePowers(0.0, 0.0, -brakingPower * direction), false);
              break;
            }

            double endHeading = totalHeading;
            double brakingDistance = Math.abs(endHeading - startHeading);

            velocityToBrakingDistance.add(new double[] {measuredVelocity, brakingDistance});

            iteration++;

            if (iteration >= POWERS.length) {
              drivetrain.stop();

              double[] c = quadraticFit(velocityToBrakingDistance);
              coefficients = List.of(c[0], c[1]);

              state = State.DONE;
            } else {
              timer.reset();
              state = State.DRIVE;
            }
            break;
          }
        case DONE:
          {
          }
      }
    }

    return coefficients;
  }

  private enum State {
    DRIVE,
    BRAKE,
    DONE
  }

  private static double[] biasedGradient(int count, double max, double min, double bias) {
    if (count < 2) return new double[] {max};

    double[] values = new double[count];

    for (int i = 0; i < count; i++) {
      double t = (double) i / (count - 1);

      double curved = 1 - Math.pow(t, bias);

      values[i] = min + curved * (max - min);
    }

    return values;
  }
}

class HeadingTuner extends TuningOpMode<Double> {
  Function<HardwareMap, Localizer> localizerFunction;
  Function<HardwareMap, Drivetrain> drivetrainFunction;

  private static final double POWER = 0.4;
  private static final double RUNTIME = 1.2;
  private static final int SAMPLES = 15;
  public static double ALPHA = 18.25;

  private double tau;
  private double K;
  private double kV;
  private double kA;
  private double vMax = 0;
  private final List<Double> times = new ArrayList<>();
  private final List<Double> velocities = new ArrayList<>();
  private final ElapsedTime timer = new ElapsedTime();
  private boolean done = false;
  private double lastTime = 0.0;

  public HeadingTuner(
      Function<HardwareMap, Localizer> localizerFunction,
      Function<HardwareMap, Drivetrain> drivetrainFunction) {
    super(
        "Heading Tuner",
        "A tuner for finding the Heading Tuning Coefficients using system identification. This will"
            + " spin the robot in place for a couple seconds.",
        false);
    this.localizerFunction = localizerFunction;
    this.drivetrainFunction = drivetrainFunction;
  }

  @Override
  protected Double runTuningOpMode() throws InterruptedException {
    Localizer localizer = localizerFunction.apply(hardwareMap);
    Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

    localizer.setPose(Pose.zero());
    localizer.update();

    times.clear();
    velocities.clear();
    done = false;
    vMax = 0;
    lastTime = 0.0;

    Thread.sleep(1000);
    waitForStart();
    localizer.setPose(Pose.zero());
    localizer.update();
    timer.reset();
    lastTime = timer.seconds();
    drivetrain.drive(new DrivePowers(0.0, 0.0, POWER), false);

    while (!done && !isStopRequested()) {
      double now = timer.seconds();
      double dt = now - lastTime;
      if (dt <= 0) dt = 1e-6;
      lastTime = now;

      localizer.update();

      if (!done) {
        times.add(timer.seconds());

        double turnVel = Math.abs(localizer.velocity().omega);
        vMax = Math.max(vMax, turnVel / POWER);

        velocities.add(turnVel);

        if (timer.seconds() >= RUNTIME) {
          done = true;
          systemIdentification();
          drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), false);
        } else {
          drivetrain.drive(new DrivePowers(0.0, 0.0, POWER), false);
        }
      }
    }

    drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), true);
    return calculatekP(ALPHA);
  }

  private double calculatekP(double alpha) {
    kV = 1 / K;
    kA = tau / K;
    return tau * alpha * alpha / K;
  }

  private void systemIdentification() {
    int N = times.size();
    if (N < 4) {
      throw new IllegalArgumentException("Failed calibration.");
    }

    int start = Math.max(0, N - SAMPLES);
    double samples = N - start;
    double sum = 0;
    for (int i = start; i < N; i++) sum += velocities.get(i);
    double A = sum / samples;
    this.K = A / POWER;

    List<Double> y = new ArrayList<>();
    List<Double> x = new ArrayList<>();
    for (int i = 0; i < N; i++) {
      double vel = velocities.get(i) / POWER;
      if (vel > 0.8 * K) continue;
      if (vel < 0.1 * K) continue;
      y.add(Math.log(K - vel));
      x.add(times.get(i));
    }
    double[] linReg = linearFit(x.toArray(new Double[0]), y.toArray(new Double[0]));
    if (linReg[1] == 0) throw new IllegalArgumentException("Failed calibration.");
    this.tau = -1.0 / linReg[1];
  }
}

class ForwardBraking extends TuningOpMode<List<Double>> {
  Function<HardwareMap, Localizer> localizerFunction;
  Function<HardwareMap, Drivetrain> drivetrainFunction;
  private final double headingLinear;
  private final double headingQuadratic;
  private final double headingKP;

  private double[] POWERS;
  public double MAX_BRAKE_TIME = 7.0;
  public int trials = 5;
  public double maxPower = 0.7;
  public double minPower = 0.3;
  public double bias = 1.5;
  public double brakingPower = 0.001;
  public double distance;
  public double IDLE_SECONDS = 1;

  private final ElapsedTime timer = new ElapsedTime();
  private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
  private State state = State.DRIVE;
  private int iteration = 0;
  private int direction;
  private double power;
  private Vector2D startPosition;
  private double measuredVelocity;

  public ForwardBraking(
      Function<HardwareMap, Localizer> localizerFunction,
      Function<HardwareMap, Drivetrain> drivetrainFunction,
      double headingLinear,
      double headingQuadratic,
      double headingKP,
      double distance) {
    super(
        "Forward Braking",
        "A tuner for finding the Forward Braking Coefficients by driving forward and backward at"
            + " various speeds. Please ensure that you have plenty of room at least "
            + distance
            + " inches ahead of the robot, but also tile space behind and laterally around the"
            + " robot.",
        false);
    this.localizerFunction = localizerFunction;
    this.drivetrainFunction = drivetrainFunction;
    this.headingLinear = headingLinear;
    this.headingQuadratic = headingQuadratic;
    this.headingKP = headingKP;
    this.distance = distance;
  }

  @Override
  protected List<Double> runTuningOpMode() throws InterruptedException {
    Localizer localizer = localizerFunction.apply(hardwareMap);
    Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

    localizer.setPose(Pose.zero());
    localizer.update();

    POWERS = biasedGradient(trials, maxPower, minPower, bias);

    List<Double> coefficients = Collections.emptyList();

    Thread.sleep(1000);
    waitForStart();
    localizer.setPose(Pose.zero());
    localizer.update();
    timer.reset();

    drivetrain.drive(new DrivePowers(maxPower, 0, 0), false);

    while (state != State.DONE && !isStopRequested()) {
      localizer.update();
      direction = (iteration % 2 == 0) ? 1 : -1;
      if (iteration < POWERS.length) {
        power = POWERS[iteration];
      }

      switch (state) {
        case DRIVE:
          {
            if ((direction > 0 && Math.abs(localizer.pose().x()) >= distance)
                || (direction < 0 && Math.abs(localizer.pose().x()) <= 12)) {
              startPosition = localizer.pose().toVector2D();
              measuredVelocity = localizer.velocity().toVector2D().magnitude();

              brake(drivetrain, localizer);
              state = State.BRAKE;
              timer.reset();
              break;
            }
            drive(drivetrain, localizer);
            break;
          }
        case BRAKE:
          {
            if (localizer.velocity().toVector2D().magnitude() > 0.25
                && timer.seconds() < MAX_BRAKE_TIME) {
              brake(drivetrain, localizer);
              break;
            }

            collectTrialData(localizer, drivetrain);
            break;
          }
        case WAIT:
          {
            drivetrain.stop();
            if (timer.seconds() > IDLE_SECONDS) state = State.DRIVE;
            break;
          }
        case DONE:
          {
          }
      }
    }

    if (state == State.DONE) {
      double[] c = quadraticFit(velocityToBrakingDistance);
      coefficients = List.of(c[0], c[1]);
    }

    return coefficients;
  }

  private double getHeadingPower(Localizer localizer) {
    double angularVel = localizer.velocity().omega;
    double brakeDist =
        headingLinear * angularVel
            + headingQuadratic * angularVel * angularVel * Math.signum(angularVel);
    double headingError = Angle.normalizeSigned(-localizer.pose().heading());
    double error = headingError - brakeDist;
    return Utils.clamp(headingKP * error, -0.3, 1.0) / 2;
  }

  private void drive(Drivetrain drivetrain, Localizer localizer) {
    drivetrain.drive(new DrivePowers(power * direction, 0.0, getHeadingPower(localizer)), false);
  }

  private void brake(Drivetrain drivetrain, Localizer localizer) {
    double headingPower = getHeadingPower(localizer);
    double brake = -brakingPower * direction;
    double minBrake = Math.abs(headingPower) + 0.001;

    if (direction > 0) {
      brake = Math.min(brake, -minBrake);
    } else {
      brake = Math.max(brake, minBrake);
    }

    drivetrain.drive(new DrivePowers(brake, 0, headingPower), false);
  }

  private void collectTrialData(Localizer localizer, Drivetrain drivetrain) {
    Vector2D endPosition = localizer.pose().toVector2D();
    double brakingDistance = endPosition.minus(startPosition).magnitude();

    velocityToBrakingDistance.add(new double[] {measuredVelocity, brakingDistance});

    iteration++;

    if (iteration >= POWERS.length) {
      drivetrain.stop();
      state = State.DONE;
    } else {
      state = State.WAIT;
      timer.reset();
    }
  }

  private enum State {
    DRIVE,
    BRAKE,
    WAIT,
    DONE
  }

  private static double[] biasedGradient(int count, double max, double min, double bias) {
    if (count < 2) return new double[] {max};
    double[] values = new double[count];
    for (int i = 0; i < count; i++) {
      double t = (double) i / (count - 1);
      double curved = 1 - Math.pow(t, bias);
      values[i] = min + curved * (max - min);
    }
    return values;
  }
}

class StrafeBraking extends TuningOpMode<List<Double>> {
  Function<HardwareMap, Localizer> localizerFunction;
  Function<HardwareMap, Drivetrain> drivetrainFunction;
  private final double headingLinear;
  private final double headingQuadratic;
  private final double headingKP;

  private double[] POWERS;
  public double MAX_BRAKE_TIME = 7.0;
  public int trials = 5;
  public double maxPower = 1;
  public double minPower = 0.3;
  public double bias = 1.5;
  public double brakingPower = 0.001;
  public double distance;
  public double IDLE_SECONDS = 1;

  private final ElapsedTime timer = new ElapsedTime();
  private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
  private State state = State.DRIVE;
  private int iteration = 0;
  private int direction;
  private double power;
  private Vector2D startPosition;
  private double measuredVelocity;

  public StrafeBraking(
      Function<HardwareMap, Localizer> localizerFunction,
      Function<HardwareMap, Drivetrain> drivetrainFunction,
      double headingLinear,
      double headingQuadratic,
      double headingKP,
      double distance) {
    super(
        "Strafe Braking",
        "A tuner for finding the Strafe Braking Coefficients by strafing left and right at various"
            + " speeds. Please ensure that you have plenty of room at least "
            + distance
            + " inches to the left and right of the robot and space in front and behind the robot.",
        false);
    this.localizerFunction = localizerFunction;
    this.drivetrainFunction = drivetrainFunction;
    this.headingLinear = headingLinear;
    this.headingQuadratic = headingQuadratic;
    this.headingKP = headingKP;
    this.distance = distance;
  }

  @Override
  protected List<Double> runTuningOpMode() throws InterruptedException {
    Localizer localizer = localizerFunction.apply(hardwareMap);
    Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

    localizer.setPose(Pose.zero());
    localizer.update();

    POWERS = biasedGradient(trials, maxPower, minPower, bias);

    List<Double> coefficients = Collections.emptyList();

    Thread.sleep(1000);
    waitForStart();
    localizer.setPose(Pose.zero());
    localizer.update();
    timer.reset();

    drivetrain.drive(new DrivePowers(0, maxPower, 0), false);

    while (state != State.DONE && !isStopRequested()) {
      localizer.update();
      direction = (iteration % 2 == 0) ? 1 : -1;
      if (iteration < POWERS.length) {
        power = POWERS[iteration];
      }

      switch (state) {
        case DRIVE:
          {
            if ((direction > 0 && Math.abs(localizer.pose().y()) > distance)
                || (direction < 0 && Math.abs(localizer.pose().y()) <= 6)) {
              startPosition = localizer.pose().toVector2D();
              measuredVelocity = localizer.velocity().toVector2D().magnitude();

              brake(drivetrain, localizer);
              state = State.BRAKE;
              timer.reset();
              break;
            }
            drive(drivetrain, localizer);
            break;
          }
        case BRAKE:
          {
            if (localizer.velocity().toVector2D().magnitude() > 0.25
                && timer.seconds() < MAX_BRAKE_TIME) {
              brake(drivetrain, localizer);
              break;
            }

            collectTrialData(localizer, drivetrain);
            break;
          }
        case WAIT:
          {
            drivetrain.stop();
            if (timer.seconds() > IDLE_SECONDS) state = State.DRIVE;
            break;
          }
        case DONE:
          {
          }
      }
    }

    if (state == State.DONE) {
      double[] c = quadraticFit(velocityToBrakingDistance);
      coefficients = List.of(c[0], c[1]);
    }

    return coefficients;
  }

  private double getHeadingPower(Localizer localizer) {
    double angularVel = localizer.velocity().omega;
    double brakeDist =
        headingLinear * angularVel
            + headingQuadratic * angularVel * angularVel * Math.signum(angularVel);
    double headingError = Angle.normalizeSigned(-localizer.pose().heading());
    double error = headingError - brakeDist;
    return Utils.clamp(headingKP * error, -0.3, 1.0) / 2;
  }

  private void drive(Drivetrain drivetrain, Localizer localizer) {
    drivetrain.drive(new DrivePowers(0.0, power * direction, getHeadingPower(localizer)), false);
  }

  private void brake(Drivetrain drivetrain, Localizer localizer) {
    double headingPower = getHeadingPower(localizer);
    double brake = -brakingPower * direction;
    double minBrake = Math.abs(headingPower) + 0.001;

    if (direction > 0) {
      brake = Math.min(brake, -minBrake);
    } else {
      brake = Math.max(brake, minBrake);
    }

    drivetrain.drive(new DrivePowers(0, brake, headingPower), false);
  }

  private void collectTrialData(Localizer localizer, Drivetrain drivetrain) {
    Vector2D endPosition = localizer.pose().toVector2D();
    double brakingDistance = endPosition.minus(startPosition).magnitude();

    velocityToBrakingDistance.add(new double[] {measuredVelocity, brakingDistance});

    iteration++;

    if (iteration >= POWERS.length) {
      drivetrain.stop();
      state = State.DONE;
    } else {
      state = State.WAIT;
      timer.reset();
    }
  }

  private enum State {
    DRIVE,
    BRAKE,
    WAIT,
    DONE
  }

  private static double[] biasedGradient(int count, double max, double min, double bias) {
    if (count < 2) return new double[] {max};
    double[] values = new double[count];
    for (int i = 0; i < count; i++) {
      double t = (double) i / (count - 1);
      double curved = 1 - Math.pow(t, bias);
      values[i] = min + curved * (max - min);
    }
    return values;
  }
}

class ForwardTranslational extends TuningOpMode<List<Double>> {
  Function<HardwareMap, Localizer> localizerFunction;
  Function<HardwareMap, Drivetrain> drivetrainFunction;

  public static double ALPHA_LARGE = 10.2;
  public static double ALPHA_SMALL = 6.2;
  private final double VEL_AGGRESSIVENESS = 0.85;
  private final double POWER = 0.4;
  private final double RUNTIME = 1.2;
  private final int SAMPLES = 15;

  private double tau;
  private double K;
  private double kV;
  private double kA;
  private double vMax = 0;
  private final List<Double> times = new ArrayList<>();
  private final List<Double> velocities = new ArrayList<>();
  private final ElapsedTime timer = new ElapsedTime();
  private boolean done = false;
  private double lastTime = 0.0;

  public ForwardTranslational(
      Function<HardwareMap, Localizer> localizerFunction,
      Function<HardwareMap, Drivetrain> drivetrainFunction) {
    super(
        "Forward Translational",
        "A tuner for finding the Forward Translational kP coefficients using system identification."
            + " This will move around 12-24 inches in front of the robot and then stop.",
        false);
    this.localizerFunction = localizerFunction;
    this.drivetrainFunction = drivetrainFunction;
  }

  @Override
  protected List<Double> runTuningOpMode() throws InterruptedException {
    Localizer localizer = localizerFunction.apply(hardwareMap);
    Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

    localizer.setPose(Pose.zero());
    localizer.update();

    times.clear();
    velocities.clear();
    done = false;
    vMax = 0;
    lastTime = 0.0;

    Thread.sleep(1000);
    waitForStart();
    localizer.setPose(Pose.zero());
    localizer.update();
    timer.reset();
    lastTime = timer.seconds();
    drivetrain.drive(new DrivePowers(POWER, 0.0, 0.0), false);

    while (!done && !isStopRequested()) {
      double now = timer.seconds();
      double dt = now - lastTime;
      if (dt <= 0) dt = 1e-6;
      lastTime = now;

      localizer.update();

      if (!done) {
        times.add(timer.seconds());

        double forwardVelocity = Math.abs(localizer.twist().toVector2D().x());
        vMax = Math.max(vMax, forwardVelocity / POWER);

        velocities.add(forwardVelocity);

        if (timer.seconds() >= RUNTIME) {
          done = true;
          systemIdentification();
          drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), false);
        } else {
          drivetrain.drive(new DrivePowers(POWER, 0.0, 0.0), false);
        }
      }
    }

    drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), true);

    double kP_large = calculatekP(ALPHA_LARGE);
    double kP_small = calculatekP(ALPHA_SMALL);

    //  kP_large, kP_small, coast kV, and brake kV (scaled by aggressiveness factor)
    return List.of(kP_large, kP_small, kV, kV * VEL_AGGRESSIVENESS);
  }

  private double calculatekP(double alpha) {
    kV = 1 / K;
    kA = tau / K;
    return tau * alpha * alpha / K;
  }

  private void systemIdentification() {
    int N = times.size();
    if (N < 4) {
      throw new IllegalArgumentException("Failed calibration.");
    }

    int start = Math.max(0, N - SAMPLES);
    double samples = N - start;
    double sum = 0;
    for (int i = start; i < N; i++) sum += velocities.get(i);
    double A = sum / samples;
    this.K = A / POWER;

    List<Double> y = new ArrayList<>();
    List<Double> x = new ArrayList<>();
    for (int i = 0; i < N; i++) {
      double vel = velocities.get(i) / POWER;
      if (vel > 0.8 * K) continue;
      if (vel < 0.1 * K) continue;
      y.add(Math.log(K - vel));
      x.add(times.get(i));
    }
    double[] linReg = linearFit(x.toArray(new Double[0]), y.toArray(new Double[0]));
    if (linReg[1] == 0) throw new IllegalArgumentException("Failed calibration.");
    this.tau = -1.0 / linReg[1];
  }
}

class StrafeTranslational extends TuningOpMode<List<Double>> {
  Function<HardwareMap, Localizer> localizerFunction;
  Function<HardwareMap, Drivetrain> drivetrainFunction;
  public static double ALPHA_LARGE = 10.2;
  public static double ALPHA_SMALL = 6.2;
  private final double POWER = 0.4;
  private final double RUNTIME = 1.2;
  private final int SAMPLES = 15;

  private double tau;
  private double K;
  private double kV;
  private double kA;
  private double vMax = 0;
  private final List<Double> times = new ArrayList<>();
  private final List<Double> velocities = new ArrayList<>();
  private final ElapsedTime timer = new ElapsedTime();
  private boolean done = false;
  private double lastTime = 0.0;

  public StrafeTranslational(
      Function<HardwareMap, Localizer> localizerFunction,
      Function<HardwareMap, Drivetrain> drivetrainFunction) {
    super(
        "Strafe Translational",
        "A tuner for finding the Strafe Translational kP coefficients using system identification."
            + " This will move around 12-24 inches to the left and right of the robot and then"
            + " stop.",
        false);
    this.localizerFunction = localizerFunction;
    this.drivetrainFunction = drivetrainFunction;
  }

  @Override
  protected List<Double> runTuningOpMode() throws InterruptedException {
    Localizer localizer = localizerFunction.apply(hardwareMap);
    Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

    localizer.setPose(Pose.zero());
    localizer.update();

    times.clear();
    velocities.clear();
    done = false;
    vMax = 0;
    lastTime = 0.0;

    Thread.sleep(1000);
    waitForStart();
    localizer.setPose(Pose.zero());
    localizer.update();
    timer.reset();
    lastTime = timer.seconds();
    drivetrain.drive(new DrivePowers(0.0, POWER, 0.0), false);

    while (!done && !isStopRequested()) {
      double now = timer.seconds();
      double dt = now - lastTime;
      if (dt <= 0) dt = 1e-6;
      lastTime = now;

      localizer.update();

      if (!done) {
        times.add(timer.seconds());

        double lateralVelocity = Math.abs(localizer.twist().toVector2D().y());
        vMax = Math.max(vMax, lateralVelocity / POWER);

        velocities.add(lateralVelocity);

        if (timer.seconds() >= RUNTIME) {
          done = true;
          systemIdentification();
          drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), false);
        } else {
          drivetrain.drive(new DrivePowers(0.0, POWER, 0.0), false);
        }
      }
    }

    drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), true);

    double kP_large = calculatekP(ALPHA_LARGE);
    double kP_small = calculatekP(ALPHA_SMALL);

    return List.of(kP_large, kP_small);
  }

  private double calculatekP(double alpha) {
    kV = 1 / K;
    kA = tau / K;
    return tau * alpha * alpha / K;
  }

  private void systemIdentification() {
    int N = times.size();
    if (N < 4) {
      throw new IllegalArgumentException("Failed calibration.");
    }

    int start = Math.max(0, N - SAMPLES);
    double samples = N - start;
    double sum = 0;
    for (int i = start; i < N; i++) sum += velocities.get(i);
    double A = sum / samples;
    this.K = A / POWER;

    List<Double> y = new ArrayList<>();
    List<Double> x = new ArrayList<>();
    for (int i = 0; i < N; i++) {
      double vel = velocities.get(i) / POWER;
      if (vel > 0.8 * K) continue;
      if (vel < 0.1 * K) continue;
      y.add(Math.log(K - vel));
      x.add(times.get(i));
    }
    double[] linReg = linearFit(x.toArray(new Double[0]), y.toArray(new Double[0]));
    if (linReg[1] == 0) throw new IllegalArgumentException("Failed calibration.");
    this.tau = -1.0 / linReg[1];
  }
}

class Tests extends Procedure {
  enum Test {
    @DisplayName("Hold Test")
    HOLD,
    @DisplayName("Line Test")
    LINE,
    @DisplayName("Curve Test")
    CURVED,
    @DisplayName("Interpolation Test")
    INTERPOLATION_CURVED,
    @DisplayName("Localization Test")
    LOCALIZATION,
    @DisplayName("Odometry Test")
    ODOMETRY,
    @DisplayName("Driving Test")
    DRIVING,
    @DisplayName("Pose Test")
    POSE
  }

  Function<HardwareMap, Drivetrain> drivetrainFunction;
  Function<HardwareMap, Localizer> localizerFunction;
  Supplier<Algorithm> algorithmSupplier;
  Function<HardwareMap, Follower> followerFunction;

  public Tests(
      Function<HardwareMap, Drivetrain> drivetrainFunction,
      Function<HardwareMap, Localizer> localizerFunction,
      Supplier<Algorithm> algorithmSupplier) {
    super("Tests", "A procedure for testing the Follower.");
    this.drivetrainFunction = drivetrainFunction;
    this.localizerFunction = localizerFunction;
    this.algorithmSupplier = algorithmSupplier;
  }

  @Override
  public void run() throws InterruptedException {
    boolean completed = false;
    boolean algorithm = true, localizer = true, drivetrain = true;

    if (algorithmSupplier == null) algorithm = false;

    if (localizerFunction == null) localizer = false;

    if (drivetrainFunction == null) drivetrain = false;

    if (algorithm && localizer && drivetrain)
      followerFunction =
          (hardwareMap) ->
              new Follower(
                  localizerFunction.apply(hardwareMap),
                  drivetrainFunction.apply(hardwareMap),
                  algorithmSupplier.get());

    Inputs inputs = inputs("Select", "Select");
    Inputs.Field<Test> selectedTest = inputs.e("Test", Test.class).withDefault(Test.LINE);
    Inputs.Field<Double> distance = inputs.d("Distance").withDefault(48.0);

    awaitInputs(inputs);

    switch (selectedTest.get()) {
      case HOLD:
        if (!algorithm) throw new IllegalArgumentException("Algorithm is required for Hold Test.");
        completed = runOpMode(new TestsHold(followerFunction));
        break;
      case LINE:
        if (!algorithm) throw new IllegalArgumentException("Algorithm is required for Hold Test.");
        completed = runOpMode(new TestsLine(followerFunction, distance.get()));
        break;
      case CURVED:
        if (!algorithm) throw new IllegalArgumentException("Algorithm is required for Hold Test.");
        completed = runOpMode(new TestsCurve(followerFunction, distance.get()));
        break;
      case INTERPOLATION_CURVED:
        if (!algorithm) throw new IllegalArgumentException("Algorithm is required for Hold Test.");
        completed = runOpMode(new TestsInterpolation(followerFunction, distance.get()));
        break;
      case LOCALIZATION:
        if (!drivetrain)
          throw new IllegalArgumentException("Drivetrain is required for Localization Test.");
        if (!localizer)
          throw new IllegalArgumentException("Localizer is required for Localization Test.");
        completed = runOpMode(new TestsLocalization(drivetrainFunction, localizerFunction));
        break;
      case ODOMETRY:
        if (!drivetrain)
          throw new IllegalArgumentException("Drivetrain is required for Odometry Test.");
        if (!localizer)
          throw new IllegalArgumentException("Localizer is required for Odometry Test.");
        completed = runOpMode(new TestsOdometry(drivetrainFunction, localizerFunction));
        if (!completed)
          abort(
              "Failed odometry test. Please check your odometry pods and ensure they are"
                  + " functioning correctly.");
        break;
      case POSE:
        if (!localizer) throw new IllegalArgumentException("Localizer is required for Pose Test.");
        completed = runOpMode(new TestsPose(localizerFunction));
        break;
      case DRIVING:
        if (!drivetrain)
          throw new IllegalArgumentException("Drivetrain is required for Driving Test.");
        completed = runOpMode(new TestsDriving(drivetrainFunction));
        break;
    }

    result("Completed", completed);
  }
}

class TestsHold extends TuningOpMode<Boolean> {
  Function<HardwareMap, Follower> followerFunction;

  public TestsHold(Function<HardwareMap, Follower> followerFunction) {
    super("Hold Test", "Tests the Follower's ability to hold a position.", true);
    this.followerFunction = followerFunction;
  }

  @Override
  public Boolean runTuningOpMode() throws InterruptedException {
    Follower follower = followerFunction.apply(hardwareMap);
    follower.setPose(Pose.zero());
    Thread.sleep(1000);
    waitForStart();
    follower.setPose(Pose.zero());
    follower.update();
    follower.hold(Pose.zero());
    while (opModeIsActive()) {
      follower.update();
    }
    return true;
  }
}

class TestsLine extends TuningOpMode<Boolean> {
  Function<HardwareMap, Follower> followerFunction;
  double distance;

  public TestsLine(Function<HardwareMap, Follower> followerFunction, double distance) {
    super("Line Test", "Tests the Follower's ability to follow a line.", true);
    this.followerFunction = followerFunction;
    this.distance = distance;
  }

  @Override
  public Boolean runTuningOpMode() throws InterruptedException {
    Follower follower = followerFunction.apply(hardwareMap);
    follower.setPose(Pose.zero());

    double distance = 48;
    boolean forward = true;

    Path path1 = line(Pose.zero(), new Pose(distance, 0, 0)).constant(0);
    Path path2 = line(new Pose(distance, 0, 0), Pose.zero()).constant(0);

    Thread.sleep(1000);
    waitForStart();
    follower.setPose(Pose.zero());
    follower.update();
    follower.follow(path1);

    while (opModeIsActive()) {
      follower.update();
      if (follower.atParametricEnd()) {
        if (forward) {
          follower.follow(path2);
        } else {
          follower.follow(path1);
        }
        forward = !forward;
      }
    }
    return true;
  }
}

class TestsCurve extends TuningOpMode<Boolean> {
  Function<HardwareMap, Follower> followerFunction;
  double distance;

  public TestsCurve(Function<HardwareMap, Follower> followerFunction, double distance) {
    super("Curve Test", "Tests the Follower's ability to follow a curve.", true);
    this.followerFunction = followerFunction;
    this.distance = distance;
  }

  @Override
  public Boolean runTuningOpMode() throws InterruptedException {
    Follower follower = followerFunction.apply(hardwareMap);
    follower.setPose(Pose.zero());

    double distance = 48;
    boolean forward = true;

    Path path1 =
        curve(Pose.zero(), new Pose(distance + 0, 0), new Pose(distance, distance)).tangent();
    Path path2 = curve(new Pose(distance, distance), new Pose(distance, 0), Pose.zero()).tangent();

    Thread.sleep(1000);
    waitForStart();
    follower.setPose(Pose.zero());
    follower.update();
    follower.follow(path1);

    while (opModeIsActive()) {
      follower.update();
      if (follower.atParametricEnd()) {
        if (forward) {
          follower.follow(path2);
        } else {
          follower.follow(path1);
        }
        forward = !forward;
      }
    }
    return true;
  }
}

class TestsInterpolation extends TuningOpMode<Boolean> {
  Function<HardwareMap, Follower> followerFunction;
  double distance;

  public TestsInterpolation(Function<HardwareMap, Follower> followerFunction, double distance) {
    super(
        "Interpolation Curve Test",
        "Tests the Follower's ability to follow a curve with several interpolations.",
        true);
    this.followerFunction = followerFunction;
    this.distance = distance;
  }

  @Override
  public Boolean runTuningOpMode() throws InterruptedException {
    Follower follower = followerFunction.apply(hardwareMap);
    follower.setPose(Pose.zero());

    double distance = 48;
    boolean forward = true;

    Path path1 =
        curve(Pose.zero(), new Pose(distance + 0, 0), new Pose(distance, distance))
            .heading((curve, t) -> Math.PI);
    Path path2 =
        curve(new Pose(distance, distance), new Pose(distance, 0), Pose.zero())
            .heading(
                Interpolator.piecewise()
                    .until(0.5, Interpolator.tangent)
                    .until(1.0, Interpolator.constant(0)));

    Thread.sleep(1000);
    waitForStart();
    follower.setPose(Pose.zero());
    follower.update();
    follower.follow(path1);

    while (opModeIsActive()) {
      follower.update();
      if (follower.atParametricEnd()) {
        if (forward) {
          follower.follow(path2);
        } else {
          follower.follow(path1);
        }
        forward = !forward;
      }
    }
    return true;
  }
}

class TestsLocalization extends TuningOpMode<Boolean> {
  Function<HardwareMap, Drivetrain> drivetrainFunction;
  Function<HardwareMap, Localizer> localizerFunction;

  public TestsLocalization(
      Function<HardwareMap, Drivetrain> drivetrainFunction,
      Function<HardwareMap, Localizer> localizerFunction) {
    super("Localization Test", "Verifies localization and manual control.", true);
    this.drivetrainFunction = drivetrainFunction;
    this.localizerFunction = localizerFunction;
  }

  @Override
  public Boolean runTuningOpMode() throws InterruptedException {
    Localizer localizer = localizerFunction.apply(hardwareMap);
    Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

    localizer.setPose(Pose.zero());

    Thread.sleep(1000);
    waitForStart();
    localizer.setPose(Pose.zero());

    while (opModeIsActive()) {
      drivetrain.drive(
          new DrivePowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x),
          true);
      localizer.update();
      telemetry.addData("Pose", localizer.pose());
      telemetry.update();
    }
    return true;
  }
}

class TestsOdometry extends TuningOpMode<Boolean> {
  Function<HardwareMap, Drivetrain> drivetrainFunction;
  Function<HardwareMap, Localizer> localizerFunction;

  public enum Test {
    FORWARD,
    LEFT,
    TURN,
    IDLE
  }

  private Test test = Test.FORWARD;
  public static double POWER = 0.5;

  double totalHeading;
  double prevHeading;

  private final ElapsedTime timer = new ElapsedTime();

  private boolean passedX = false;
  private boolean passedY = false;
  private boolean passedHeading = false;

  public TestsOdometry(
      Function<HardwareMap, Drivetrain> drivetrainFunction,
      Function<HardwareMap, Localizer> localizerFunction) {
    super("Localization Test", "Verifies localization and manual control.", true);
    this.drivetrainFunction = drivetrainFunction;
    this.localizerFunction = localizerFunction;
  }

  @Override
  public Boolean runTuningOpMode() throws InterruptedException {
    Localizer localizer = localizerFunction.apply(hardwareMap);
    Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

    localizer.setPose(Pose.zero());

    Thread.sleep(1000);
    waitForStart();
    localizer.setPose(Pose.zero());

    timer.reset();

    while (opModeIsActive()) {
      localizer.update();

      if (localizer.pose().x() != Pose.zero().x()
          || localizer.pose().y() != Pose.zero().y()
          || localizer.pose().heading() != Pose.zero().heading()) {
        double currentHeading = localizer.pose().heading();
        totalHeading += Angle.normalizeSigned(currentHeading - prevHeading);
        prevHeading = currentHeading;
      }

      switch (test) {
        case FORWARD:
          drivetrain.drive(new DrivePowers(POWER, 0, 0), false);

          if (timer.seconds() > 1) {
            test = Test.LEFT;
            timer.reset();
          }
          break;
        case LEFT:
          drivetrain.drive(new DrivePowers(0, POWER, 0), false);

          if (timer.seconds() > 1) {
            test = Test.TURN;
            timer.reset();

            prevHeading = localizer.pose().heading();
            totalHeading = 0;
          }
          break;
        case TURN:
          drivetrain.drive(new DrivePowers(0, 0, POWER), false);

          if (timer.seconds() > 1) {
            test = Test.IDLE;
            timer.reset();
          }
          break;
        case IDLE:
          drivetrain.drive(new DrivePowers(0, 0, 0), true);

          telemetry.addData("Test", "Completed");

          Pose pose = localizer.pose();

          if (pose.x() < 0) telemetry.addData("xPod Direction", "Flipped");
          else if (pose.x() < 2) telemetry.addData("xPod Resolution", "Too high");
          else if (pose.x() > 144) telemetry.addData("xPod Resolution", "Too low");
          else {
            telemetry.addData("xPod", "Good");
            passedX = true;
          }

          if (pose.y() < 0) telemetry.addData("yPod Direction", "Flipped");
          else if (pose.y() < 2) telemetry.addData("yPod Resolution", "Too high");
          else if (pose.y() > 144) telemetry.addData("yPod Resolution", "Too low");
          else {
            telemetry.addData("yPod", "Good");
            passedY = true;
          }

          if (totalHeading < 0) telemetry.addData("Heading Direction", "Flipped");
          else if (totalHeading < 0.02) telemetry.addData("Heading Resolution", "Too high");
          else if (totalHeading > 2 * Math.PI) telemetry.addData("Heading Resolution", "Too low");
          else {
            telemetry.addData("Heading", "Good");
            passedHeading = true;
          }

          break;
      }

      telemetry.addData("Pose", localizer.pose());
      telemetry.update();
    }
    return passedX && passedY && passedHeading;
  }
}

class TestsDriving extends TuningOpMode<Boolean> {
  Function<HardwareMap, Drivetrain> drivetrainFunction;

  public TestsDriving(Function<HardwareMap, Drivetrain> drivetrainFunction) {
    super("Driving Test", "Tests raw drivetrain control without localization.", true);
    this.drivetrainFunction = drivetrainFunction;
  }

  @Override
  public Boolean runTuningOpMode() throws InterruptedException {
    Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);
    waitForStart();
    while (opModeIsActive()) {
      drivetrain.drive(
          new DrivePowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x),
          true);
    }
    return true;
  }
}

class TestsPose extends TuningOpMode<Boolean> {
  Function<HardwareMap, Localizer> localizerFunction;

  public TestsPose(Function<HardwareMap, Localizer> localizerFunction) {
    super("Pose Test", "Verifies localizer output without a drivetrain.", true);
    this.localizerFunction = localizerFunction;
  }

  @Override
  public Boolean runTuningOpMode() throws InterruptedException {
    Localizer localizer = localizerFunction.apply(hardwareMap);
    Thread.sleep(1000);
    waitForStart();
    localizer.setPose(Pose.zero());
    localizer.update();
    while (opModeIsActive()) {
      localizer.update();
      telemetry.addData("Pose", localizer.pose());
      telemetry.update();
    }
    return true;
  }
}
