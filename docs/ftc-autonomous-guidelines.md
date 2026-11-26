# FTC autonomous reliability

This project uses FTC SDK 12.0.0, Pedro 3.0.1, and the repository's voltage-based
LQR shooter. Online examples using older `followPath`/`PathChain` APIs must be
translated against the installed library, rather than copied directly.

## Lifecycle and failure handling

Every match auto is a plain SDK `@Autonomous` OpMode built on `AutoOpMode`, which
initializes devices and the route before becoming READY. STOP during INIT never
starts a routine. The collection auto runs its Ivy sequential group directly from
`loop()`. The blocker servo is not moved until START (as on main).
The original eight autos and the shared lifecycle live in `autos/Autonomous.kt`.
`MainAutos.kt` adds eight imported sequence autos; `PreloadAutos.kt` adds four
preload/park autos and a timed move, giving 21 entries.
START only runs a routine after READY. Basic autos advance through the measured
speed gate, shared 200 ms blocker delay, two-second feed, and path following
without blocking waits. The longer routine keeps its one-second feed per shot;
backup routines drive without shooter initialization.
Backup routines keep updating Pedro after arriving, so the robot holds its park
pose until the period ends (as on main). The longer collection routine shuts down
after its whole sequence completes.
A runtime or linkage failure enters FAILED, preserves the original stack trace,
and displays the phase and exception as `Auto error`. FAILED cannot resume by
pressing START. Fix the reported cause and reinitialize.

Cleanup attempts each known motor independently, closes the blocker (once START enabled it), stops the
voltage worker, and restores each hub’s previous bulk-cache mode. It does not call
`follower.update()` to stop: that also polls localization and can fail when the
Pinpoint is the device that disconnected. Cleanup failures are attached to the
first error rather than replacing it. An unsuccessful physical hardware write
cannot be guaranteed to stop that device; the error remains recorded.

Hardware names are case-sensitive: `driveFL`, `driveFR`, `driveBL`, `driveBR`,
`shooter1`, `shooter2`, `intake1`, `intake2`, `blocker`, and `pinpoint`. Do not
silently substitute another port or run a partial auto when a device is missing.
Motor references are retained before setup writes, so partial initialization can
still attempt to stop devices that were acquired.

## Routes and performance

Use the coordinates from the selected `.pp` file. Compare starting pose heading
with the first requested path heading; a 180-degree mismatch commands a turn.
Constant interpolation preserves a heading while translating along a line.
Changing the declared starting pose is not physical evidence of the robot's
orientation. Confirm the user's physical placement when direction is ambiguous.

Read bulk-cached inputs once per loop, compute from snapshots, then write outputs.
Clear MANUAL cache every active loop; cached readings otherwise stay stale.
Keep the compact status and failure telemetry, publishing every 100 ms (`TELEMETRY_INTERVAL_MS`).
The loop-time row is the only measure of the achieved loop rate.
Keep existing LQR settings and the shared 0.72 preset when fixing lifecycle code.

## Debugging evidence

Capture the selected OpMode, INIT/START/LOOP phase, exception type/message, and
RobotLog stack when available. Without the original device error, distinguish
confirmed defects, regression coverage, and unconfirmed causes. A full app build
can detect compile/link problems locally; it cannot prove the deployed app,
hardware configuration, or device connection matches this checkout. Sloth/pinned
class or dependency changes may require a full APK update by the user. Agents
must not deploy or execute robot code in this project.

Local checks:

```sh
./gradlew :TeamCode:assembleDebug --console=plain
```

This repository has no `src/test` sources, so `./gradlew test` runs nothing.
Treat a passing build as compile evidence only; route timing, READY guards and
cleanup still need on-robot validation.

## Primary references reviewed

- [FIRST SDK repository](https://github.com/FIRST-Tech-Challenge/FtcRobotController)
  and its [bulk-read sample](https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptMotorBulkRead.java):
  hub caching and reuse of sampled inputs.
- [Pedro 3.0.1 Follower source](https://github.com/Pedro-Pathing/PedroPathing/blob/v3.0.1/core/src/main/java/com/pedropathing/follower/Follower.java):
  `stop()` changes state; `update()` reads localization and applies outputs.
- [Pedro autonomous lifecycle](https://pedropathing.com/docs/pathing/guide/setting-up-auto),
  [heading interpolation](https://pedropathing.com/docs/pathing/reference/interpolation),
  and [debugging](https://pedropathing.com/docs/pathing/guide/debugging).
- [FIRST control-system troubleshooting](https://ftc-docs.firstinspires.org/en/latest/control_system_troubleshooting/index.html).

## Shared intake and compact TeleOp

`teleop/Intake.kt` contains `IntakeFeed`, used by Robot, the Basic autos and
ShootCommand. It applies the same speed gate and a 200 ms opening delay. Feed power
follows main: 1.0 in TeleOp and ShootCommand, the 0.72 shooter preset in the Basic,
preload and MAIN autos. Basic feed timing begins when intake power actually starts, keeping two
full seconds of transfer. The longer routine retains its one-second transfer.
Manual reverse, stop, collection assist, and the manual-mode blocker override
remain available in TeleOp. Collection path powers and route coordinates are unchanged.

TeleOp is a plain SDK `OpMode`. Each INIT constructs fresh hardware; `stop()` and any
loop failure run the same `SafeHardware` shutdown. `RobotHardware` is the single
bulk-cache owner and restores each hub's cache mode on exit.
Pedro's [v3.0.1 Follower implementation](https://github.com/Pedro-Pathing/PedroPathing/blob/v3.0.1/core/src/main/java/com/pedropathing/follower/Follower.java)
confirms why shutdown stops drivetrain outputs directly instead of calling update.
