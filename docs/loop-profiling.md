# BioBuzzTeleOp loop timing

`BioBuzzTeleOp` measures every call to the major loop sections using a monotonic
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

## Scheduler and total-loop measurement

BioBuzzTeleOp works around duplicate scheduling in the pinned NextFTC 1.0.0:
its base OpMode explicitly runs the command manager before `onUpdate`, while its
default manager component would run it again in `postUpdate`. TeleOp wraps that
component to suppress only the second `postUpdate` pass. Other manager lifecycle
hooks, including initialization and shutdown, remain delegated to NextFTC.

The total timer is the first component: it starts before other pre-update hooks
and ends after the reverse-order post-update hooks, including bulk-cache clearing.
Driver and mechanism callback counts should now roughly match the loop count,
rather than being double it. Counts can differ by one at a reporting-window
boundary because the snapshot occurs before the current loop has completed.
A local regression runs the real NextFTC lifecycle with fake command durations
and checks both single execution and the full measured total. Recheck this
workaround if the NextFTC dependency is upgraded.

## What is measured

| Section | Includes |
| --- | --- |
| Loop interval | Time between successive loop starts, including between-loop scheduling delays |
| Loop work total | Work between the NextFTC pre-update and post-update hooks, including reporting and bulk-cache clearing |
| Drive total | The entire driver-control callback, including its nested sections |
| Drive encoder reads | Four encoder reads and their result array; the first read on a hub can fetch its motor bulk data |
| Pinpoint location read | Normal Pinpoint update and pose conversion |
| Pinpoint initial pose write | First-loop starting-pose write |
| Hive save extra Pinpoint read | Extra location read when G1 X saves the hive |
| Pedro hold update | Shooting hold setup/update or release, including Pedro's second Pinpoint read and drive writes; inactive idle calls excluded |
| Manual drive motor writes | Normal joystick motor output, plus the stop when entering shooting |
| Drive telemetry build | Both groups of drive telemetry entries; normally two calls per loop |
| Mechanisms total | The entire shooter/intake callback, including nested sections |
| Shooter control total | Shooter mode selection and shooting/off/reverse update |
| Shooter encoder and velocity reads | Both motors' positions and velocities, plus RPM conversion |
| Shooter PID calculation | FTCLib feedback calculation |
| Shooter motor mode writes | Both run-mode setter calls; cached SDK calls may perform no hardware transaction |
| Shooter cached voltage and motor writes | Cached voltage lookup, compensation and motor power writes; includes direct zero-output paths |
| Blocker update and servo write | Blocker state calculation and servo position setter |
| Intake motor writes | Both intake power setters |
| Main telemetry build and getters | Main telemetry entries, their getters/formatting, and timing reporting |
| Telemetry update both outputs | JoinedTelemetry update call |
| Driver Station telemetry update | Driver Station delegate update call |
| Panels telemetry update | Panels delegate update call |
| Timing report overhead | Statistics snapshot, report formatting/logging when due, and adding timing telemetry rows |
| Async voltage sensor poll (background) | All voltage-sensor reads on the background worker |

Totals include their child measurements: do **not** add every row together.
Telemetry update measures the API call, not end-to-end radio/network delivery.
Motor bulk caching means the first encoder read can bear the cost of fetching
other motors' data; later reads can be cheap cache accesses. Voltage polling runs
concurrently, so its duration is **not** part of the main loop's measured work,
although shared hub communication can still delay main-thread hardware calls.

The profiler holds its counter lock only while recording or copying statistics,
never while accessing hardware. It is enabled by TeleOp; other OpModes retain
their existing behavior without per-section profiling.
