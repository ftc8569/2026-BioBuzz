# FTC autonomous reliability

This project uses FTC SDK 12.0.0, Pedro 3.0.1, and the repository's voltage-based
LQR shooter. Online examples using older `followPath`/`PathChain` APIs must be
translated against the installed library, rather than copied directly.

## Lifecycle and failure handling

Every match auto uses AutoOpMode to initialize its devices and route before becoming READY.
START only runs a routine after READY. Basic autos advance through the measured
speed gate, shared 100 ms blocker delay, two-second feed, and path following
without blocking waits. The longer routine keeps its one-second feed per shot;
backup routines drive without shooter initialization.
A runtime or linkage failure enters FAILED, preserves the original stack trace,
and displays the phase and exception as `Auto error`. FAILED cannot resume by
pressing START. Fix the reported cause and reinitialize.

Cleanup attempts each known motor independently, closes the blocker, stops the
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
Keep telemetry content and publish at its configured interval; a mocked 10 ms
clock test demonstrates timing logic, not an achieved 100 Hz hardware rate.
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
./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug --console=plain
```

The Basic auto tests cover both exported routes and shooting time, real Pedro and
LQR startup with hardware doubles, missing-device initialization, START failures,
and a loop fault combined with a failed stop write. AutoLifecycleIntegrationTest also runs all eight match autos with real Pedro code, injecting a Pinpoint failure and failed stop writes. Preserve those distinctions when interpreting results.

DriverControlsIntegrationTest exercises both strafing directions, forward/reverse,
turning, normalized mixed input, shooting-hold handoff, and saved-hive aiming with
the real subsystems and Pedro against fake devices. These are local logic checks,
not confirmation of wheel orientation, encoder wiring or field performance.

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

`subsystems/IntakeSubsystem.kt` contains `IntakeFeed`, used by Robot, the Basic autos and
ShootCommand. It applies the same speed gate, 0.6 feed power and 100 ms opening
delay. Basic feed timing begins when intake power actually starts, keeping two
full seconds of transfer. The longer routine retains its one-second transfer.
Manual reverse, stop, collection assist, and the manual-mode blocker override
remain available in TeleOp. Collection path powers and route coordinates are unchanged.

TeleOp uses [NextFTC's NextOpMode lifecycle](https://nextftc.dev/robot/nextopmode/).
Its hardware-free robot container is discovered once; each INIT constructs fresh
hardware and one coordinated default command. Cleanup uses `OpModeHook.afterEnd`,
which the installed 0.3.0 runtime calls from `finally`, including loop failures.
The shared cache owner remains responsible for refresh and restoration; adding
NextFTC's `BulkReadHook` would duplicate that responsibility.
Pedro's [v3.0.1 Follower implementation](https://github.com/Pedro-Pathing/PedroPathing/blob/v3.0.1/core/src/main/java/com/pedropathing/follower/Follower.java)
confirms why shutdown stops drivetrain outputs directly instead of calling update.
