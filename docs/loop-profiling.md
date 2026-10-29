# Compact telemetry and loop timing

Normal TeleOp now shows eight rows on both Driver Station and Panels:
mode, preset, target RPM, both measured RPMs, motor power, feed readiness, drive
status and selected hive. Long `Timing:` reports, repeated motor specifications,
raw encoder counts and tuning/model dumps have been removed from match telemetry.

Click **Gamepad 1's right stick** to add only three status rows: pose, intake power
and collection-assist state, and loop interval in milliseconds. Details start off.
Holding the button does not repeat; another click clears the extra rows. This
control changes the display only. Both modes skip the four optional drive-encoder
reads while retaining required Pinpoint and shooter feedback.

## One coordinated frame

NextFTC schedules `DriverRobot`'s default command, which samples controls and inputs,
computes drive and mechanism requests once, commits changed outputs, then reports.
A failed read or computation skips the normal commit and runs shutdown. The command
owns drive, shooter and intake requirements, so no second command separates a read
from its corresponding write.

All 21 autonomous and four diagnostic entries also use NextFTC. Their adapter
copies gamepads before each callback and owns one routine command per active frame.
It constructs fresh hardware during INIT and stops it from `afterEnd`, including
partial initialization and failures. A routine starts only on an active frame;
STOP during INIT cannot request movement. The collection auto's sequential command
group runs inside its owning frame, without nested global scheduler execution.

The shared hardware loop owns bulk-cache refresh and mode restoration. Do not add
NextFTC `BulkReadHook` or separate subsystem threads: either would create competing
hardware reads. The five subsystems implement `Mechanism`; only the coordinated
frame command is installed as a default command.

## Display and hardware cost

One `JoinedTelemetry` formats each value once and shares it between Driver Station
and Panels. A 50 ms gate skips formatting and publishing between display frames.
NextFTC also performs its standard Driver Station flush; the SDK transmission
interval still limits sends. The display gate never throttles control calculations.

Output caches skip unchanged motor and servo requests. Emergency shutdown always
forces zero independently to every motor. Voltage sampling uses the existing worker
at 1 Hz while idle and 4 Hz while powered, with activity checked every 250 ms.
No additional hardware-control threads were added.

`LoopProfiler` retains fixed-size timing counters for local diagnostics and tests.
It no longer formats, publishes or logs a large report automatically. The optional
`Loop (ms)` row shows the interval between driver frames. These values and simulated
sensor-read counts do not establish physical loop speed or battery runtime.

Tests run the installed NextFTC scheduler and Pedro/LQR code with fake physical
devices to check ordering, captured controls, cache freshness and cleanup.
See [branch integration](branch-integration.md#build-and-validation) for local results.

References: [NextFTC lifecycle](https://nextftc.dev/robot/nextopmode/),
[FTC SDK telemetry example](https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/v12.0/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptTelemetry.java).
