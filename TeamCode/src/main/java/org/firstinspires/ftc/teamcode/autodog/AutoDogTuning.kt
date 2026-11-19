package org.firstinspires.ftc.teamcode.autodog

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.Subsytem.DriveSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.IntakeSubsystem

/** ChaseController uses +left and +CCW; DriveSubsystem.drive uses +strafe right and +turn CW. */
fun DriveSubsystem.driveChase(p: DrivePower) = drive(p.forward, -p.left, -p.turn)

/** Live-tunable from Panels Configurables. Distances in meters unless named. */
@Configurable
object AutoDogTuning {
    @JvmField var kTurn = 1.6
    @JvmField var kForward = 1.2
    @JvmField var kStrafe = 1.5
    // Low caps for first runs on a real robot; raise once the chase looks right.
    @JvmField var maxForward = 0.35
    @JvmField var maxStrafe = 0.25
    @JvmField var maxTurn = 0.40
    @JvmField var intakeOffset = 0.22
    @JvmField var intakePower = 1.0
    @JvmField var searchTurn = 0.25
    /** Stop chasing when the newest camera frame (or laptop target) is older than this (s). */
    @JvmField var maxTargetAgeS = 0.25

    /** Pose Pinpoint is told at init, in the team's center-origin frame (docs/hive-vision.md). */
    @JvmField var startXInches = 0.0
    @JvmField var startYInches = 0.0
    @JvmField var startHeadingDeg = 0.0

    // Ball camera (the global shutter webcam). The mount and resolution are read when
    // START is pressed. Measure the mount on the robot; these are placeholders.
    @JvmField var camWidthPx = 1280
    @JvmField var camHeightPx = 720
    /** Lens ahead of the robot center. */
    @JvmField var camForwardM = 0.15
    /** Lens left of the robot center. */
    @JvmField var camLeftM = 0.0
    /** Lens above the floor. */
    @JvmField var camHeightM = 0.28
    /** Degrees the lens points down from level. */
    @JvmField var camPitchDownDeg = 28.0
    /** Degrees the lens points left of straight ahead. */
    @JvmField var camYawLeftDeg = 0.0
    @JvmField var camRollDeg = 0.0
    /** True when the camera is mounted upside down (the image is turned 180 degrees). */
    @JvmField var camUpsideDown = false
    /** Extra delay between the true exposure and the frame's time stamp (s). */
    @JvmField var captureLatencyS = 0.0
    /** False keeps balls outside the 12 ft field, for a floor where the start pose is not known. */
    @JvmField var gateField = true

    /** Laptop tracker OpMode only: the Limelight pipeline whose stream the laptop reads. */
    @JvmField var limelightPipeline = 0

    fun mount() = CameraMount(
        x = camForwardM, y = camLeftM, z = camHeightM,
        pitchRad = Math.toRadians(camPitchDownDeg),
        yawRad = Math.toRadians(camYawLeftDeg),
        rollRad = Math.toRadians(camRollDeg),
    )

    fun intrinsics() = CameraIntrinsics.GLOBAL_SHUTTER_720P.scaled(camWidthPx, camHeightPx)
}

/**
 * Ball detector thresholds, live-tunable from Panels. HSV on the OpenCV scale
 * (H 0 to 180, S and V 0 to 255). These are starting points: tune them on the
 * Driver Station camera stream under the real lights.
 */
@Configurable
object AutoDogVision {
    @JvmField var yellowHMin = 18
    @JvmField var yellowHMax = 38
    @JvmField var yellowSMin = 100
    @JvmField var yellowVMin = 90
    /** Red wraps around H = 0: it is 0..redHLowMax plus redHHighMin..180. */
    @JvmField var redHLowMax = 8
    @JvmField var redHHighMin = 170
    @JvmField var redSMin = 120
    @JvmField var redVMin = 70
    @JvmField var blueHMin = 100
    @JvmField var blueHMax = 125
    @JvmField var blueSMin = 120
    @JvmField var blueVMin = 50

    /**
     * Smallest blob, in full-resolution pixels. The wide lens makes far balls small:
     * at 1280x720 a POLLEN 3 m away is about 9 px across (65 px of area).
     */
    @JvmField var minAreaPx = 40.0
    @JvmField var maxAreaFraction = 0.25
    /** Blob area / enclosing circle area. A full disc is 1, a strip of tape about 0.1. */
    @JvmField var minFill = 0.55
    /** Morphological opening kernel (px) that removes specks; 0 turns it off. */
    @JvmField var openPx = 3
    /** Detect on an image shrunk by this factor (1 = full resolution). 2 is faster but halves the range. */
    @JvmField var shrink = 1
}

/**
 * The driver interface shared by both Auto Dog OpModes (gamepad 1):
 *   hold A      chase the target with the intake on; with no target the robot stops.
 *               Releasing A is the dead-man switch.
 *   hold A + B  same, but spin slowly in place while there is no target
 *   sticks      normal manual drive whenever A is not held
 */
class ChaseDriver(private val drive: DriveSubsystem, private val intake: IntakeSubsystem) {
    private val chase = ChaseController()
    var mode = "manual"
        private set
    var power = DrivePower.STOP
        private set

    /** Pose and target are in the tracker field frame (m). target is null when there is nothing fresh to chase. */
    fun step(gamepad: Gamepad, rx: Double, ry: Double, theta: Double, target: Pair<Double, Double>?) {
        val tune = AutoDogTuning
        chase.kTurn = tune.kTurn
        chase.kForward = tune.kForward
        chase.kStrafe = tune.kStrafe
        chase.maxForward = tune.maxForward
        chase.maxStrafe = tune.maxStrafe
        chase.maxTurn = tune.maxTurn
        chase.intakeOffset = tune.intakeOffset

        power = DrivePower.STOP
        var intakePower = 0.0
        when {
            gamepad.a && target != null -> {
                power = chase.compute(rx, ry, theta, target.first, target.second)
                intakePower = tune.intakePower
                mode = "CHASE"
            }
            gamepad.a && gamepad.b -> {
                power = DrivePower(0.0, 0.0, tune.searchTurn)
                mode = "SEARCH"
            }
            gamepad.a -> mode = "HOLD (no target)"
            else -> mode = "manual"
        }
        intake.setPower(intakePower)
        if (mode == "manual") {
            drive.drive(
                -gamepad.left_stick_y.toDouble(),
                gamepad.left_stick_x.toDouble(),
                gamepad.right_stick_x.toDouble(),
            )
        } else {
            drive.driveChase(power)
        }
    }

    fun stop() {
        drive.stop()
        intake.setPower(0.0)
    }
}
