# Branch integration — 2026-10-25

Integrated into the existing `2026-BioBuzz` checkout on `ai-slop-review`.
TeleOp remains exactly five source files; autonomous code uses three files.

## Imported work

| Source branch and revision | Integrated behavior | Current location |
| --- | --- | --- |
| `ri5w-autos` — `d98058d` | Eight red/blue MAIN autos: left-020, left-034, right-004, right-022; all 92 original steps, via points, shot counts and route time limits | `autos/MainAutos.kt` |
| `updates/preload-park-blocker-timing` — `4ae8faf` | Red/blue close/far preload parking curves and MoveForwardLeft | `autos/PreloadAutos.kt` |
| `shreyas-josyula` — `66b6d1e` | Pollen sweep and separate red/blue saved hives, with the branch's approach offset | `teleop/Drive.kt` and `teleop/Robot.kt` |
| `main`, `anirud-ri5w-autos`, `yatishgrandhe/manual-shooter` | Already ancestors of the starting checkout; existing Basic, collection and backup autos, subsystems and driver features retained | Existing five TeleOp files and `autos/Autonomous.kt` |

The old `BioBuzz Auto` entry on ri5w/shreyas is a TODO-only example with no
START/LOOP path execution. It is superseded by the working Basic autos; no empty
example or duplicate TeleOp name was added to the Driver Station menu.

Distinct path exports are retained in `pedro paths/branch-imports/` under their
source branch names. Existing path exports are unchanged. MAIN routes retain the
source branch's generated coordinates directly beside their runner.

This is a feature integration into the current architecture, rather than copying
obsolete subsystem implementations alongside their replacements. The current
Drive, Intake, Shooter, location snapshot and optional HiveVision cover those
subsystems. No second Pinpoint owner or competing hardware-control loop was added.

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

## Build and validation

The build uses Gradle 9.7.0, Android Gradle Plugin 9.3.3, Kotlin 2.4.21, and JDK 17.
The [Kotlin compatibility table](https://kotlinlang.org/docs/gradle-configure-project.html)
lists Gradle through 9.7.0 and AGP through 9.3.1. We use the newer 9.3.3 patch
because 9.3.1's lint crashes on JDK 17; the fix is listed in the
[AGP 9.3 patch release notes](https://developer.android.com/build/releases/agp-9-3-0-release-notes).
This patch combination is checked with local tests, APK assembly and lint.
The wrapper distribution has a pinned SHA-256 checksum. FTC 12.0.0, Pedro 3.0.1,
NextFTC 0.3.0 and Load/Sloth 0.3.2 remain unchanged. Legacy Android DSL and external
Kotlin support remain enabled for the installed Load/Sloth integration; Gradle
reports deprecation warnings for those compatibility flags.

Two focused regression files live under `TeamCode/src/test/.../regression/`;
they are outside the five-file TeleOp package. Earlier staged test deletions
remain in place. For integration comparison, the previous 118 tests were adapted
to the consolidated package names in a temporary folder and run alongside these
new tests. They exercise actual installed Pedro/LQR with hardware doubles, not a
physical robot. Test/build results and limitations are summarized in the project
README. No robot installation, deployment, app launch or OpMode execution was performed.

Local results on 2026-10-25:

- Combined temporary comparison suite: 133 tests, zero failures/errors.
- Normal checkout: 15 retained tests, zero failures/errors; debug APK and lint pass.
- Lint: zero errors, 17 warnings; no detector was disabled to obtain this result.
- Source comparison: all 92 MAIN steps match their branch data, including via points.
- APK inspection: all 21 annotated match auto classes plus TeleOp are defined in DEX.
- TeleOp directory: exactly five files, including all shared subsystem code.

Reproduce the retained checks without accessing a robot:

```sh
./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug :TeamCode:lintDebug --offline --console=plain
```

Omit `--offline` on a new machine that still needs to download the dependencies.
