# Robot loop timing

`DriverRobot` measures every call to the major loop sections using a monotonic
nanosecond clock. It publishes one-second windows in milliseconds:

- Driver Station and Panels telemetry: rows beginning with `Timing:`.
- Robot Controller logs: tag `BioBuzzLoop`, one report per second.

Each row contains `avg`, `max`, and `n` (number of calls in that window). The maximum
retains occasional stalls that an average would hide. A section that did not run
is marked `not called in this window`; it is not counted as a zero-duration call.
The first report appears after one second of active operation. Formatting is done
once per report, and telemetry reuses those strings between reports.

## Toggle detailed logging

Click **Gamepad 1's right stick** to toggle detailed diagnostics. Detailed logging
starts enabled. Holding the stick button does not toggle repeatedly; release
and click again to switch back.

When disabled, Driver Station and Panels still display `Loop interval (ms)` and
all `Timing:` rows. Section profiling and `BioBuzzLoop` reports continue without
resetting the measurement window. Only non-timing telemetry and diagnostic drive
encoder reads are skipped. A skipped section is shown as `not called in this window`.
Robot controls, required localization and shooter feedback, and background
battery-voltage polling continue normally. Turning details back on restarts drive
encoder deltas from a fresh baseline without clearing timing statistics.

## Frame order and total-loop measurement

NextFTC schedules `DriverRobot`'s default command, which reads inputs, computes drive
and mechanism requests once, commits
outputs, and publishes telemetry when due. A failed read or computation skips the
normal output commit and runs shutdown. The total timer includes every phase and
ends in `finally`. The command owns all three subsystem requirements and performs
the complete frame in order, so scheduling cannot separate a read from its write.

Local regressions exercise this actual loop with deterministic clocks, and run
the real Pedro/Foresight controller against hardware doubles. These checks verify
ordering and calculations, not an achieved hardware loop frequency.

NextFTC 0.3.0 runs inside `LinearOpMode`, whose gamepads can change during a frame.
The driver samples used controls into two reused gamepads before computing.
Diagnostic encoder
reads and Pedro wheel mixing reuse fixed arrays. Encoder deltas have independent
history so a new read cannot overwrite the previous frame. LQR settings retain
their immutable configuration until a live tuning value changes; invalid settings
still stop the controller, and corrected settings take effect on the next frame.
These reduce repeated allocation; hardware speed still requires measurement.

## What is measured

| Section | Includes |
| --- | --- |
| Loop interval | Time between successive loop starts, including between-loop scheduling delays |
| Loop work total | All work in `DriverRobot.update()`, including input reads, computation, output commits and reporting |
| Read inputs phase | Hub bulk refresh, shooter samples, optional drive encoders and Pinpoint |
| Hub bulk read transactions | One refresh per required hub; diagnostics-only hubs are skipped when details are off |
| Control compute phase | Drive, shooter and intake decisions from captured inputs |
| Write outputs phase | Final motor and blocker commands |
| Drive total | The entire driver-control callback, including its nested sections |
| Drive encoder reads | Four cached encoder reads and their result array; hub transactions run in the read phase |
| Pinpoint location read | Normal Pinpoint update and pose conversion |
| Pinpoint initial pose write | Starting-pose write at START |
| Hive save (cached pose) | Reserved timing label; saving the cached pose is included in Drive total |
| Pedro hold update | Shooting hold computation from the captured pose; writes wait for the output phase |
| Drive motor output commit | Final joystick/Pedro requests, suppressing unchanged writes |
| Drive telemetry build | Drive telemetry entries at the display interval |
| Mechanisms total | The entire shooter/intake callback, including nested sections |
| Shooter control total | Shooter mode selection and shooting/off/reverse update |
| Shooter encoder and velocity reads | Both motors' positions and velocities; RPM is derived from these cached values |
| Shooter LQR calculation | Discrete LQR feedback calculation |
| Shooter motor mode writes | Reserved timing label; modes are set once during INIT, never rewritten in the active loop |
| Shooter motor output commit | Cached voltage compensation and final motor writes; unchanged outputs are skipped |
| Blocker servo output commit | Final blocker position write, skipping unchanged positions |
| Intake motor writes | Both intake power setters |
| Main telemetry build and getters | Main telemetry entries, their getters/formatting, and timing reporting |
| Telemetry update both outputs | JoinedTelemetry update call |
| Driver Station telemetry update | Driver Station delegate update call |
| Panels telemetry update | Panels delegate update call |
| Timing report overhead | Statistics snapshot, report formatting/logging when due, and adding timing telemetry rows |
| Async voltage sensor poll (background) | All voltage-sensor reads on the background worker |

Totals include their child measurements: do **not** add every row together.
Telemetry update measures the API call, not end-to-end radio/network delivery.
The explicit hub bulk refresh bears the cost of fetching motor data; subsequent
encoder and velocity reads use that snapshot. Voltage polling runs
concurrently, so its duration is **not** part of the main loop's measured work,
although shared hub communication can still delay main-thread hardware calls.
The existing worker samples voltage at 1 Hz while the shooter is idle and 4 Hz
while powered, checking every 250 ms for activity. This reduces idle sensor
transactions without putting voltage reads on the driver-control thread.

The profiler holds its counter lock only while recording or copying statistics,
never while accessing hardware. It is enabled by TeleOp; other OpModes retain
their existing behavior without per-section profiling.

## Joined display output

`teleop/Telemetry.kt` owns the cached shooter renderer and creates one Panels `JoinedTelemetry` for the lifetime
of the driver object. A 50 ms gate runs before building telemetry, so formatting
and both destination updates are skipped between display frames. Formatted values
are built once and sent to both delegates through the value overload. Timing
strings are cached for one second. Separate timing wrappers preserve the Driver
Station and Panels update measurements. The display gate never skips a hardware
frame or a control calculation.

NextFTC also calls its standard Driver Station telemetry flush each loop. The SDK
transmission interval still limits sends; our joined display is formatted and
published only at the 50 ms gate. No per-session global telemetry backends are added.

This uses the installed Panels telemetry 1.0.7 source: its formatting overload
forwards formatting to each delegate, while its value overload accepts an already
formatted string. FTC's [SDK 12 telemetry example](https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/v12.0/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptTelemetry.java)
explains the transmission interval. Constructor ownership follows Kotlin's
[class and constructor documentation](https://kotlinlang.org/docs/classes.html).
