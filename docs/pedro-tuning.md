# Pedro tools and shared subsystems

This checkout uses Pedro **3.0.1**, AutoTune **1.0.1**, and FTC **12.0.0**.
The branch tuners were checked against the installed dependency sources. Pedro 3
uses a [robot-hosted AutoTune interface](https://pedropathing.com/docs/pathing/tuning).
The Java factory methods below register procedures; constructing a factory does
not open hardware or run an OpMode.

## Restored branch procedures

`pedro/Tuning.java` registers nine procedures with `@Tuner`:

| Procedure | Source file | Purpose |
| --- | --- | --- |
| Mecanum | `Tuning.java` | Drive motor configuration and drivetrain measurements |
| Pinpoint | `Tuning.java` | Current robot's localizer configuration and calibration |
| Foresight | `Tuning.java` | Installed Pedro follower tuning |
| Tests | `Tuning.java` | Existing path/hold tests |
| OTOS | `procedures/OTOSTuner.java` | Optical odometry scalars and offsets |
| OctoQuad | `procedures/OctoQuadTuner.java` | Heading, pod resolution/direction and offsets |
| Two Wheel | `procedures/TwoWheelTuner.java` | Two encoder pods with IMU |
| Three Wheel | `procedures/ThreeWheelTuner.java` | Three encoder pods |
| Three Wheel + IMU | `procedures/ThreeWheelIMUTuner.java` | Three encoder pods with IMU |

The five restored files come from `origin/main` at `7d2c210`; their branch versions
also exist in `shreyas-josyula` and `ri5w-autos`. Existing Mecanum, Pinpoint,
Foresight and Tests remain consolidated. All paths above are relative to
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/`.

The match robot continues to use `Constants.localizerConfig` with Pinpoint.
Restoring alternatives does not select different hardware or overwrite calibrated
constants. Encoder defaults now match `driveFL`, `driveFR`, `driveBL`, `driveBR`;
the tuning input form must match the actual encoder connections. Alternative
localizers require their own configured sensor and, where applicable, IMU.

The restored passive measurements pause 10 ms between samples. Initialization
uses uncached reads so Pedro's encoder resets cannot leave stale baselines. Each
measurement then owns MANUAL hub caching, clears it once per loop, and restores
prior modes on completion,
cancellation or failure. The three-wheel routines retain drive handles before
setup and attempt every motor stop independently. Invalid distances, missing
motion, invalid scalar results and cancelled offset measurements are rejected.
OctoQuad resolution uses the latest measured sample. No new control thread was
added. No calibration or robot execution was performed during this integration.

## Subsystem map

TeleOp remains **exactly five control files**. The actual subsystem implementations
now live in the separate `subsystems` package, in these five source files:

| Subsystem | File | Ownership |
| --- | --- | --- |
| `DriveSubsystem` | `subsystems/DriveSubsystem.kt` | Buffered wheel requests and Pedro adapter |
| `IntakeSubsystem` | `subsystems/IntakeSubsystem.kt` | Shared intake outputs and feeding logic |
| `ShooterSubsystem` | `subsystems/ShooterSubsystem.kt` | Existing LQR, flywheels and blocker |
| `RobotLocationSubsystem` | `subsystems/RobotLocationSubsystem.kt` | One captured pose, separate red/blue hives, heading and approach targets |
| `VisionSubsystem` | `subsystems/VisionSubsystem.kt` | Optional camera start/update/stop, cached getters and hive solver |

Location and vision adapt the branch subsystem responsibilities to this checkout.
Location uses the existing single Pinpoint read in `RobotHardware`; getters and
Pedro cannot cause another sensor read. Non-finite pose data fails before output
commit, and NextFTC cleanup stops the actuators. Vision starts only when an OpMode
explicitly requests it; no match mode starts a camera. `Vision Diagnostic` offers
an optional camera-only entry without adding another TeleOp source file.

`Shooter Tuning` in `teleop/Telemetry.kt` reuses the current manual-shooter branch
controls and the same LQR/manual subsystem, with no drivetrain requirement. See
[shooter controls and tuning](control/shooter-lqr.md#driver-controls-and-manual-tuning).

The driver mappings, LQR gains, 0.72 preset, routes and feed timing are unchanged.
Fixed hive geometry is computed once instead of during every solver trial, and
output-write lambdas are inlined. These reduce CPU work/allocation; battery-runtime
improvements have not been measured on hardware.
