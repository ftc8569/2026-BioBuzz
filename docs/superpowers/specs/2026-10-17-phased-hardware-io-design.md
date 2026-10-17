# Phased hardware I/O for BioBuzzTeleOp

## Goal

Make each active TeleOp iteration read hardware once, compute entirely from a
stable input snapshot, commit final actuator outputs once, and publish telemetry
without additional hardware access. Preserve current driver controls, shooting
behavior, autonomous routines, async voltage sampling, and timing visibility.

Checkpoint before this refactor: `5122b0d`.

## Constraints

- Local edits, builds, tests, and Git only. No robot installation, deployment,
  app launch, or OpMode execution.
- Keep FTC SDK 12.0.0, NextFTC 1.0.0, Pedro 3.0.1, and FTCLib 2.1.1 unchanged.
- Retain the single-scheduler-pass workaround and its lifecycle regression tests.
- Preserve manual drive mixing/signs and its 0.9 multiplier; preserve Pedro's
  distinct mixing/sign convention, normalization, and hold-controller tuning.
- Preserve the effective Pinpoint calibration in `Constants.localizerConfig`,
  including the reversed Y pod, offsets, and units. Do not substitute the
  different default calibration in `RobotLocationSubsystem`.
- Keep the existing shooter preset, PIDF tuning, RPM conversion/tolerance,
  readiness latch, manual spin-up delay, blocker positions, reverse behavior,
  intake assist, and 100 ms blocker/feed delay.
- Preserve the existing public subsystem constructors/methods used by autos and
  diagnostic OpModes; this migration changes TeleOp's hardware boundary.
- Keep the async battery-voltage worker. Its cadence and 12 V fallback remain;
  each main-loop snapshot copies its latest published voltage once.

## Selected approach

Use explicit input/output data and narrow hardware adapters. Adding more SDK
caches inside the current control methods would leave reads and writes
interleaved. Wrapping every SDK device method would preserve more source code,
but makes accidental hardware access and separate library output caches harder
to audit. Explicit phases provide a boundary that can be tested with fake I/O.

## Loop order

1. **Read inputs:** copy the gamepad controls used by TeleOp; clear manual bulk
   caches at the beginning of the hardware frame; collect required encoder and
   velocity data from at most one bulk response per needed hub; update Pinpoint
   once and capture both pose and velocity; copy cached battery voltage.
2. **Compute:** existing driver and mechanism commands consume that frame's
   inputs and controller state. They produce desired drive powers, flywheel
   powers, intake power, and blocker position. No SDK hardware getters/setters
   run here. Mode changes and blocker decisions settle before outputs commit.
3. **Write outputs:** one owner sends each actuator's final command at most once
   for the frame, suppressing commands whose values have not changed.
4. **Report:** telemetry consumes input snapshots, controller status, committed
   commanded outputs, and timing snapshots. It never asks a motor or servo for
   a value.

NextFTC's pre-update phase gathers inputs before its explicit scheduler pass.
After the commands compute, `onUpdate` commits outputs and publishes telemetry.
The profiler remains the first component so its total ends last. Replace the
existing end-of-loop bulk-cache clearing with the read phase's explicit cache
management; do not run two independent cache-clearing mechanisms.

## Boundaries and responsibilities

| Unit | Responsibility |
| --- | --- |
| `RobotInputs` / `RobotOutputs` | Immutable per-frame input snapshot and final desired outputs; keep static motor metadata in initialization configuration |
| `TeleOpHardware` | Own physical device references, hub-cache management, one Pinpoint sampler, input capture, output cache, initialization, and forced shutdown |
| Shooter controller | Extract existing state/PIDF/readiness logic into calculations consuming shooter feedback, frame time, voltage, and driver requests; expose computed status without SDK getters |
| Existing `ShooterSubsystem` | Keep its public API as an immediate-I/O compatibility adapter for autos/diagnostics using the same controller logic; TeleOp uses the phased path |
| Drive/intake control | Produce desired outputs rather than writing while computing; use one committed drive-output record for intake-assist and telemetry |
| Shared Pedro localizer | Publish the captured pose/velocity as Pedro `MotionState`; follower `update()` reads that snapshot without polling Pinpoint again |
| Buffered Pedro drivetrain | Implement Pedro's `Drivetrain` calculation contract, including normalization and scaling, and submit desired outputs to the same drive owner used by manual control |
| `BioBuzzTeleOp` | Orchestrate read, compute, write, reporting, and existing input bindings |

Keep these units focused; no new general-purpose robot framework or dependency
is needed. Existing classes can retain names where their responsibilities fit.

## Hardware rules

- The sole physical Pinpoint sampler uses the current Pedro calibration. Set the
  initial pose during initialization/start, outside recurring computation.
- Hive-save uses the current captured location instead of triggering another
  read. Pedro hold consumes the same frame's pose and velocity.
- Manual drive and hold select one final drive output. Switching owners cannot
  rely on two independent motor caches, and must not leave old manual/hold power
  active. Do not send an intermediate stop followed by hold power in one frame.
- Initialize run modes, directions, and zero-power behavior explicitly. There
  is no per-frame `RUN_WITHOUT_ENCODER` setter: manual and software PID both use
  that same run mode. If a real configuration change is needed, handle it in
  the write owner rather than in control calculations.
- Keep a software cache of last successfully submitted outputs. It must not
  consult SDK `power`, `position`, or `mode` getters to decide whether to write.
- Motor and servo commands remain individual SDK transactions; this is one
  consolidated application write phase, not a new bulk-write transport.
- Preserve the current manual-drive precision and Pedro hold's configured
  power threshold in the unified output path; do not add arbitrary deadbands.
- Initialization invalidates output caches and establishes known hardware
  commands. Shutdown/exception cleanup explicitly flushes zero motor powers
  and the closed blocker, even when cached values appear unchanged, and stops
  voltage polling. Never commit partially computed outputs after a failure.

## Telemetry and timings

The G1 right-stick toggle continues to hide only non-timing telemetry. All
section timings and `BioBuzzLoop` reports remain active in both modes.

Add read-phase, compute-phase, and write-phase totals and retain useful detailed
measurements. Label bulk-read cost at the actual read boundary so it is not
misattributed to whichever getter happens to run first. Keep separate Pinpoint,
output-group, telemetry, and async voltage timings. Reports remain once per
second with average/max milliseconds and counts.

Non-timing telemetry uses captured encoder values and committed commanded
powers/servo position. These are commanded values, not physical power/position
measurements. Optional diagnostic data can remain conditional on the toggle;
all data required for control is captured every frame.

## Verification

- Record fake hardware events and assert every active frame orders all reads
  before computation, all computation before writes, and telemetry afterward.
- Assert controller computation and telemetry make no hardware calls.
- Assert one Pinpoint read per frame, including hold entry/steady hold and
  hive-save, and that Pedro consumes the captured pose/velocity.
- Verify at most one bulk fetch per needed hub and at most one final command
  per actuator per frame; unchanged outputs produce no repeated SDK setters.
- Verify manual/hold/manual transitions, including identical requested powers
  across ownership changes, and preserve the two drive mixing conventions.
- Verify blocker readiness/manual overrides settle to one final servo command;
  maintain idle, reverse, PID, preset changes, spin-up, and feed-delay behavior.
- Keep autonomous compatibility tests and the real NextFTC scheduler lifecycle
  tests, including cleanup of a still-running command.
- Verify forced stop and read/compute failure cleanup cannot publish stale or
  partial outputs, and telemetry toggling does not interrupt control feedback
  or voltage polling.
- Run focused local tests and a local compile. The full suite previously has
  five unrelated readiness/preset/assist expectation failures; report them
  separately rather than changing those settings as part of this refactor.

No on-robot timing improvement will be claimed from local tests. Subsequent
robot logs can compare phase costs and verify the expected read/write counts.
