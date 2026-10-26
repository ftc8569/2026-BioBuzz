# TeamCode source map

TeleOp uses five files in `teleop/`, with comments explaining the controls and safety rules:

- `Robot.kt`: driver controls, hardware setup, match loop and safe shutdown.
- `Drive.kt`: mecanum drive, Pedro adapter and shooting position hold.
- `Intake.kt`: intake, collection assist and shared feeding rules.
- `Shooter.kt`: flywheels, blocker, LQR, presets and battery sampling.
- `Telemetry.kt`: displays, timing measurements and diagnostic OpModes.

The other six production files are:

- `autos/Autonomous.kt`: eight original match autos, routes and shared lifecycle.
- `autos/MainAutos.kt`: eight MAIN autos and their 92 imported route steps.
- `autos/PreloadAutos.kt`: four preload parking autos and the timed diagonal move.
- `pedro/Constants.java`: calibrated Pedro configuration.
- `pedro/Tuning.java`: tuning and diagnostic tools.
- `vision/HiveVision.kt`: optional camera pose solver.

The [project README](../../../../../../../../README.md) lists the controls, battery
optimizations and local test command. Unit and integration tests are under
`TeamCode/src/test/`. Keep SDK examples in `FtcRobotController` separate from team code.
TeleOp tests live in the `teamcode.regression` package under that test source tree;
the `teamcode.teleop` package contains only the five robot source files.
