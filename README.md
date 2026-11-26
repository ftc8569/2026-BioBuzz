# BioBuzz robot code (FTC 2026–27)

FIRST Tech Challenge robot built on FTC SDK 12.0, Pedro Pathing 3 and Panels. Every
OpMode is a plain SDK `@TeleOp` / `@Autonomous` class: no command framework sits
between the SDK lifecycle and the robot code. Team code is in
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. For the SDK's own docs and
release notes, see [FTC-Docs](https://ftc-docs.firstinspires.org) and the upstream
[FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController) repo.

## Code map

| Path | Responsibility |
| --- | --- |
| `teleop/Robot.kt` | Match TeleOp, driver controls, hardware setup, read → compute → write loop, `SafeHardware` shutdown |
| `teleop/Drive.kt` | Pedro drivetrain adapter, shot position hold, hive approach / pollen sweep |
| `teleop/Intake.kt` | `IntakeFeed`: one feed policy for TeleOp and autos |
| `teleop/Shooter.kt` | Shooter buttons and preset (default 0.72 → 3240 RPM) |
| `teleop/Telemetry.kt` | Driver Station + Panels telemetry, diagnostic and tuning OpModes |
| `subsystems/` | Drive, Intake, Shooter (LQR flywheel), RobotLocation, optional Limelight Vision |
| `autos/Autonomous.kt` | `AutoOpMode` guarded lifecycle, `ShootCommand`, Basic / collection / backup autos |
| `autos/MainAutos.kt` | Step-list runner and the four MAIN routes (red and blue) |
| `autos/PreloadAutos.kt` | Preload-and-park autos and `MoveForwardLeft` |
| `pedro/Constants.java` | Pinpoint, mecanum and Foresight tuning values |
| `pedro/Tuning.java` | Pedro AutoTune procedures for this robot (Mecanum, Pinpoint, Foresight, tests) |

## How each feature works

**One loop, one hardware owner.** Every TeleOp loop copies both gamepads, bulk-reads the
shooter hub once, reads Pinpoint once, computes drive/shooter/intake requests, then writes
only outputs whose value changed. Shutdown (STOP or any exception) forces zero to every
motor, stops the voltage thread, and restores hub cache modes, even if one device fails.

**Blocker stays still in INIT.** As on main, no OpMode moves the blocker servo during INIT.
START closes it (0.78); it opens (0.65) only when feeding is allowed.

**Shooter (LQR).** The preset (0.72) sets a target of preset × 4500 RPM. Each loop the
LQR computes volts = kV × target + K × (target − flywheel 1 RPM); both motors get that
voltage scaled by battery voltage (sampled 1 Hz idle, 4 Hz while spinning, on a
background thread). Feeding unlocks once flywheel 1 is within 1 % and stays unlocked
until the shot is released. Hold Y for manual power instead: no RPM regulation, feed
unlocks after 3 s. `kV`, `kA` and the LQR costs are live-tunable in Panels
(Configurables › ShooterLqrTuning).

**Feeding.** `IntakeFeed` waits for the shooter's go-ahead, then waits 200 ms more for the
blocker to open. Feed power follows main: 1.0 in TeleOp and `ShootCommand`, the 0.72
preset in Basic, preload and MAIN autos. G1 A always stops; reverse trigger always works.
Collection assist (on by default) runs the intake at 0.45 when driving above half power.

**Shot position hold.** While shooting, Pedro holds the pose captured when the button was
pressed; joystick drive resumes on release.

**Hive tools.** G1 X saves the robot pose as the selected alliance's hive (blue by default;
G2 D-pad left/right switches). G2 Y turns toward it (0.8 × error, 0.12–0.35 power, stops
within 2°). G2 X drives to a point 53.4 in in front of the hive. G2 A sweeps eight
semicircles while collecting. Any stick, shot or G1 A cancels.

**Autonomous.** Routes are written for red; `alliancePose()` mirrors them about the field
center for blue. `AutoOpMode` reports any failure with its phase on the Driver Station and
in RobotLog, and never moves before INIT reaches READY.
- *BasicRed / BasicBlue / Preload park*: spin up, feed for 2 s after the blocker opens,
  then follow one Pedro path and stop.
- *BioBuzzRed1.5 / BioBuzz Auto Blue*: shoot, collect along path 1, drive path 2, shoot
  again, drive path 3 (each shot feeds for 1 s).
- *MAIN left/right routes*: step lists (drive, push, shoot, wait); shots feed 1.5 s per
  ball, up to three; a forced park starts before 30 s.
- *Backup*: one line to the park zone, then hold that pose, as on main. No shooter.
- *MoveForwardLeft*: 0.5 power diagonal for 0.25 s.

**Diagnostics (group "Test").** Blocker Diagnostic (A close / B open), Shooter Motor
Diagnostic (raw power, encoders, connection and motor profile), Shooter Tuning (match
shooter controls with no drive or intake), Vision Diagnostic (Limelight, if configured).

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
