# Branch integration — updated 2026-10-29

Integrated into the existing `2026-BioBuzz` checkout on `ai-slop-review`.
TeleOp remains exactly five source files; autonomous code uses three files.

## Imported work

| Source branch and revision | Integrated behavior | Current location |
| --- | --- | --- |
| `ri5w-autos` — `d98058d` | Eight red/blue MAIN autos: left-020, left-034, right-004, right-022; all 92 original steps, via points, shot counts and route time limits | `autos/MainAutos.kt` |
| `updates/preload-park-blocker-timing` — `4ae8faf` | Red/blue close/far preload parking curves and MoveForwardLeft | `autos/PreloadAutos.kt` |
| `shreyas-josyula` — `66b6d1e` | Pollen sweep and separate red/blue saved hives, with the branch's approach offset | `teleop/Drive.kt` and `teleop/Robot.kt` |
| `main`, `anirud-ri5w-autos`, `yatishgrandhe/manual-shooter` | Already ancestors of the starting checkout; existing Basic, collection and backup autos, subsystems and driver features retained | `subsystems/`, five TeleOp control files and `autos/Autonomous.kt` |

The old `BioBuzz Auto` entry on ri5w/shreyas is a TODO-only example with no
START/LOOP path execution. It is superseded by the working Basic autos; no empty
example or duplicate TeleOp name was added to the Driver Station menu.

Distinct path exports are retained in `pedro paths/branch-imports/` under their
source branch names. Existing path exports are unchanged. MAIN routes retain the
source branch's generated coordinates directly beside their runner.

This is a feature integration into the current architecture, rather than copying
obsolete subsystem implementations alongside their replacements. The current
Drive, Intake, Shooter, RobotLocationSubsystem and optional VisionSubsystem cover
those responsibilities. No second Pinpoint owner or competing hardware-control loop was added.

## Additional subsystem and tuner integration — 2026-10-28

Fetched all origin branches again on October 28; their source heads remain the
revisions above. Restored OTOS, OctoQuad, Two Wheel, Three Wheel and Three Wheel +
IMU procedures from `main` (`7d2c210`), also present in the other working branches.
All nine Pedro procedure factories are now registered and checked with the
installed scanner. Current Pinpoint calibration remains unchanged. See the
[Pedro tuner and subsystem map](pedro-tuning.md) for source locations and lifecycle
changes.

The branch location and vision responsibilities now have explicit subsystem APIs.
Location shares the existing one-read snapshot and per-alliance hives; vision has
an explicit start/stop lifecycle with cached getters and remains optional.
`Vision Diagnostic` provides a camera-only check. Invalid TeleOp pose samples stop
before output commit. Fixed hive geometry is cached and output-write lambdas are
inlined; these changes do not establish measured battery savings.

Reviewed the current manual-shooter branch (`adb12be`): hold-Y, per-press preset
adjustment, reverse priority and manual blocker control are already integrated.
The user selected retaining LQR. `Shooter Tuning` now exposes the same controller
and manual controls in a dedicated diagnostic. The older blocker/motor diagnostics
now use current 0.78/0.65 blocker endpoints instead of their obsolete 0.24/0.45
values. No PID controller or old Y-toggle mapping was reintroduced. See
[shooter tuning](control/shooter-lqr.md#driver-controls-and-manual-tuning).

The actual subsystem implementations now live in five named files under
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/`: DriveSubsystem,
IntakeSubsystem, ShooterSubsystem, RobotLocationSubsystem and VisionSubsystem.
TeleOp still contains exactly five control files; TeamCode has twenty production
source files. Imports in TeleOp, diagnostics and autos point to those implementations.
The former bundled classes and old `vision/HiveVision.kt` are removed, so there are
no duplicate hardware owners. Manual power adjustment remains in `teleop/Shooter.kt`:
G1 Y selects manual; either D-pad changes the shared preset by 0.005 per press.
No drive routes, match settings or existing button mappings were changed.

## Controls

Existing controls and the TeleOp Driver Station entry are retained. Additions use
previously unused gamepad 2 buttons:

- D-pad left/right selects the blue/red saved hive. Blue is the initial selection.
- G1 X saves the selected hive; G2 Y retains the existing aim behavior.
- G2 X approaches the selected saved hive (3 inches right, 53.369 inches forward).
- G2 A toggles the branch's pollen sweep (10-inch radius, eight passes), with intake.
- Driver sticks, shooting/reverse, or G1 A cancel the sweep/approach immediately.
  Holding the triggering button does not restart a cancelled assist.

Hive positions are saved separately for each alliance during the current INIT.
An approach without a saved hive leaves manual drive available.

## Preserved settings and corrected lifecycle

The existing LQR controller, 0.72 preset, 0.6 feed power, 100 ms blocker delay,
Pedro calibration, motor zero-power modes, and existing routes remain authoritative.
Older PID/PIDF code, the ri5w 0.78 preset, alternative motor braking/calibration,
and the update branch's 200 ms delay/deferred blocker initialization were not
substituted for these settings. The existing blocker closes during safe INIT and
shutdown. Basic and preload autos retain two full seconds of actual feeding;
the original collection auto retains its one-second shots.

The imported MAIN autos preserve the source's maximum three balls at 1.5 seconds
of actual feeding each, seven-second shot timeout, 1.5x drive timeouts, and forced
parking windows. A speed gate and blocker delay precede feed timing. Completing
or timing out a path stops its drive request before shooting or waiting. All
outputs stop at 30 seconds, and completed autos stop holding/polling indefinitely.

The MAIN source branch did not implement camera, hopper-full, or tip sensors.
That limitation remains explicit: camera waits use its 2.5-second fallback,
tip waits use their timeout, and collection cannot end early on a hopper signal.
The runner reports this in telemetry. Optional HiveVision remains available;
no camera is enabled or required by these OpModes.

All 21 match autos share READY/START failure guards and independent actuator
cleanup. MoveForwardLeft uses only the four drive motors and runs its original
half-power diagonal for 250 ms; it does not require Pinpoint or shooter hardware.

## Feature comparison and NextFTC migration — 2026-10-29

Fetched and inspected all seven origin branches. Source heads are unchanged from
the revisions listed above; `main` is `7d2c210`, `anirud-ri5w-autos` is `b1f8145`,
and `yatishgrandhe/manual-shooter` is `adb12be`.

A direct source-data comparison checks all **92 MAIN steps** against
`ri5w-autos/autos/Routines.java`: types, labels, coordinates, via points, shot counts,
timeouts and intake flags match. All four route headers match starting poses and
forced-park windows. Existing Basic, collection, backup and preload routes are
unchanged by the current migration. The shooter, drive and intake control files
are byte-for-byte unchanged from the preceding commit.

The maintained feature set includes LQR and manual power adjustment, reverse,
blocker override, automatic feed delay/readiness, intake assist, shooting position
hold, separate alliance hives, aiming, approach, pollen sweep, optional vision,
21 autos and all nine Pedro tuner factories. This is not a claim that every old
menu entry or controller is identical: older PID/PIDF and Y-toggle controls are
intentionally superseded, separate alliance TeleOps are combined, and the historical
standalone `BasicMecanumDrive` menu entry is not copied (manual mecanum drive remains
in match TeleOp). Unimplemented hopper/camera/tip hooks remain timeout fallbacks.

All **26 selectable team entries** (one match TeleOp, 21 autos and four diagnostics)
now enter through NextFTC. `NextRoutineOpMode` adapts existing guarded controllers
without changing routes; these remain plain SDK OpMode classes internally and are
not independently registered. All five subsystems implement NextFTC `Mechanism`.
Only the coordinated frame owns execution, avoiding competing default commands.
Pedro's nine tuning factories keep the tuning library's required API.

The installed NextFTC scanner accepts the public no-argument entry constructors.
The per-run routine robot is private, leaving `RobotSession` as the sole public
concrete NextRobot candidate. START is deferred until the first active frame to
handle STOP during INIT. The collection auto executes its owned Ivy group directly
inside the frame instead of recursively invoking the global scheduler.

Telemetry is intentionally reduced to **eight normal rows**, with **three optional
rows** on the existing right-stick toggle. Mode, target/measured speeds, feed status
and autonomous failures remain visible. Raw motor dumps and large timing reports
are removed, and display details no longer request optional drive encoder reads.

## Build and validation

The build uses Gradle 9.7.1, Android Gradle Plugin 9.3.3, Kotlin 2.4.21, and JDK 17.
The [Kotlin compatibility table](https://kotlinlang.org/docs/gradle-configure-project.html)
lists Gradle through 9.7.0 and AGP through 9.3.1. We use the newer 9.3.3 patch
because 9.3.1's lint crashes on JDK 17; the fix is listed in the
[AGP 9.3 patch release notes](https://developer.android.com/build/releases/agp-9-3-0-release-notes).
Gradle uses the [9.7.1 maintenance release](https://docs.gradle.org/9.7.1/release-notes.html),
one patch above Kotlin's listed Gradle maximum; this is not a claim of full vendor
certification for the patch combination. It is checked locally with tests, APK
assembly and lint.
The wrapper distribution has a pinned SHA-256 checksum. FTC 12.0.0, Pedro 3.0.1,
NextFTC 0.3.0 and Load/Sloth 0.3.2 remain unchanged. Legacy Android DSL and external
Kotlin support remain enabled for the installed Load/Sloth integration; Gradle
reports deprecation warnings for those compatibility flags.

The two remaining regression files were removed during this work; their deletions
are preserved. Tests now run from `/private/tmp/biobuzz-validation` through an
external Gradle source-set init script. A normal checkout test invocation has no
sources and must not be reported as a passed regression suite. The external suite
includes the prior 133 checks, restored tuner and camera tests, geometry comparison,
non-finite pose shutdown, and the shared-controller shooter tuning lifecycle.
All hardware is doubled; installed Pedro/NextFTC/LQR implementations execute locally.

Local results on 2026-10-29:

- **163 external-suite tests passed**, zero failures, errors or skips.
- Debug APK assembly and Android lint passed offline.
- Lint: **zero errors, 14 warnings** (12 inherited native-library alignment notices,
  one dependency-version notice, one unused resource). No detector was disabled.
- Installed Pedro scanner discovers all nine procedures. Real installed
  ThreeWheelLocalizer tests exercise encoder reset freshness and cache restoration.
- Installed NextFTC/follower/LQR tests cover driver controls, non-finite odometry,
  partial initialization and independent shutdown. Shooter tuning uses the actual
  shared controller with fake motors, including manual/LQR handoff and readiness.
- All 21 autos run through the installed NextFTC wrapper in active and cancelled-INIT
  tests. Cache invalidation is at most once per active frame; completed routines stop
  polling. Scanner tests accept all 25 new entry constructors; the existing match
  TeleOp lifecycle is covered separately. Collection uses one global scheduler tick
  per frame. Fault injection verifies visible failure and independent shutdown.
- Compact-display tests verify eight default rows, three optional rows, no repeated
  timing dumps, and preserved controls without diagnostic drive-encoder reads.
- APK DEX defines all 26 NextFTC entries, including all 21 autos and four diagnostics;
  the location/vision subsystems and five restored tuner classes remain included.
- TeleOp and subsystems each contain exactly five files. Manual increase/decrease
  is verified through both gamepads against the real shared shooter implementation.
  Source comparison confirms the five imported
  tuner files share the same original revision across main, ri5w and shreyas.
- No robot validation or battery-runtime measurement was performed.

Build/lint without accessing a robot:

```sh
./gradlew :TeamCode:assembleDebug :TeamCode:lintDebug --offline --console=plain
```

External regression command on this machine while the temporary sources exist:

```sh
./gradlew :TeamCode:testDebugUnitTest --rerun :TeamCode:assembleDebug :TeamCode:lintDebug \
  --init-script /private/tmp/biobuzz-validation.init.gradle --offline --console=plain
```

Omit `--offline` on a new machine that still needs to download dependencies.
No installation, deployment, app launch or OpMode execution occurred on a robot.
