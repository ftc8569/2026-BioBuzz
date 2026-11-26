# BioBuzz robot code (FTC 2026–27)

FIRST Tech Challenge robot built on FTC SDK 12.0, NextFTC 0.3.0, Pedro Pathing 3
and Panels. Team code is in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
For the SDK's own docs and release notes, see
[FTC-Docs](https://ftc-docs.firstinspires.org) and the upstream
[FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController) repo.

## Code map

| Path | Responsibility |
| --- | --- |
| `teleop/Robot.kt` | TeleOp entry, driver controls, hardware setup, read → compute → write loop, `SafeHardware` shutdown, `NextRoutineOpMode` adapter |
| `teleop/Drive.kt` | Pedro drivetrain adapter, shot position hold, hive approach / pollen sweep |
| `teleop/Intake.kt` | Feed policy shared by TeleOp and autos (0.6 power, 100 ms blocker delay) |
| `teleop/Shooter.kt` | Shooter buttons and preset (default 0.72 → 3240 RPM) |
| `teleop/Telemetry.kt` | Driver Station + Panels telemetry (50 ms), diagnostic OpModes |
| `subsystems/` | Drive, Intake, Shooter (LQR flywheel), RobotLocation, optional Limelight Vision |
| `autos/Autonomous.kt` | `AutoOpMode` guarded lifecycle, `ShootCommand`, Basic / collection / backup autos |
| `autos/MainAutos.kt` | Step-list runner and the four MAIN routes (red and blue) |
| `autos/PreloadAutos.kt` | Preload-and-park autos and `MoveForwardLeft` |
| `pedro/Constants.java` | Pinpoint, mecanum and Foresight tuning values |
| `pedro/Tuning.java` | Pedro AutoTune procedures for this robot (Mecanum, Pinpoint, Foresight, tests) |

Routes are written for red; `alliancePose()` mirrors them about the field center for blue.

## Driver controls

| Input | Behavior |
| --- | --- |
| G1 left stick / right stick X | Drive / turn |
| Hold G1 right bumper | LQR shooting with position hold and speed-gated feeding |
| Hold G1 Y | Manual shooting with position hold; overrides right bumper |
| Hold G1 left bumper | Reverse flywheels; takes priority over shooting |
| G1 right / left trigger | Forward / reverse intake |
| Hold G1 A | Stop intake, including automatic feeding and collection assist |
| Either D-pad up / down | Adjust shooter preset by 0.005 per press (22.5 RPM in LQR) |
| Either B | Open blocker in manual shooter mode |
| G1 X, then G2 Y | Save selected hive position, then aim at it; manual turning cancels aim |
| G2 D-pad left / right | Select blue / red hive (separate saved positions) |
| G2 X | Approach the selected saved hive |
| G2 A | Toggle pollen sweep and collection; sticks, shooting/reverse or G1 A cancel |
| G1 left-stick click | Toggle collection assist; initially on |
| G1 right-stick click | Toggle extra status rows (pose, intake, loop time) |

Shooter tuning details: [SHOOTER_TUNING.md](TeamCode/SHOOTER_TUNING.md) and
[shooter LQR](docs/control/shooter-lqr.md). Auto reliability rules:
[autonomous guidelines](docs/ftc-autonomous-guidelines.md).

## Build and deploy

```sh
./gradlew :TeamCode:assembleDebug      # local build, no robot needed
./gradlew :TeamCode:installDebug       # first install, or after dependency changes
./gradlew deploySloth                  # fast TeamCode-only reload afterwards
```

Gradle 9.7.1, Kotlin 2.4.21, AGP 9.3.3, JDK 17. Connect over USB or robot Wi-Fi
(`adb connect 192.168.43.1:5555`) first; Sloth does not auto-connect.
Panels: <http://192.168.43.1:8001> (Control Hub).
Agents must not deploy to the robot; see [AGENTS.md](AGENTS.md).

## Robot configuration

[TeamCode/robot-configs/BioBuzz.xml](TeamCode/robot-configs/BioBuzz.xml) is the active
configuration. It has no Limelight, so `VisionSubsystem` and `Vision Diagnostic` stay idle.
Shooter motors use the custom `BioBuzz shooter goBILDA 1:1` motor type (28 ticks/rev).
