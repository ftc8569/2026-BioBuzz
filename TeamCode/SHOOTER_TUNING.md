# Shooter control

The shooter uses a discrete velocity LQR with model-based voltage feedforward,
using flywheel 1 RPM for both motors. Output is normalized against cached battery
voltage and clipped to available motor power. TeleOp captures hardware feedback
once per frame and commits final changed outputs after controller computation.
See [the LQR model and tuning guide](../docs/control/shooter-lqr.md).

## Controls

- Hold gamepad 1 right bumper: LQR shooting.
- Hold gamepad 1 Y: manual power, taking priority over right bumper.
- Release both: stop the shooter and automatic transfer.
- Left bumper: reverse, taking priority over shooting.
- Either D-pad up/down: adjust the preset by 0.005.

## RPM and transfer

Default preset: 0.72. Target: `0.72 * 4500 = 3240 RPM`.
The confirmed goBILDA 1:1 gearbox uses 28 encoder counts per output revolution:
`actual RPM = abs(encoder ticks/second) * 60 / 28`.
The active configuration selects `BioBuzzShooter1To1` for both shooters.

While LQR shooting, forward transfer and the blocker latch on once **flywheel 1
RPM** enters **99% to 101% of target** (3207.6 through 3272.4 RPM at the default
3240 RPM target). They remain enabled through later speed drops. The latch
resets on release, reverse, mode/preset changes, or invalid LQR parameters or feedback. Current
at-speed telemetry still reports the actual RPM band independently of the latch.
B and forward trigger cannot bypass the initial speed check. A stops intake
while held; reverse remains available to clear jams.

Manual mode preserves voltage-normalized power, its three-second feed timer, and the B blocker
override. Releasing Y stops manual shooting.

## Panels tuning

Open Panels > **Configurables** > **ShooterLqrTuning**:

| Field | Initial value | Units / purpose |
| --- | --- | --- |
| kV | 0.002436 | V/RPM; steady-speed model |
| kA | 0.000609 | V/(RPM/s); acceleration model |
| rpmTolerance | 150 | RPM error cost scale; smaller is more aggressive |
| voltageTolerance | 3 | Voltage cost scale; larger is more aggressive |

The model defaults are estimates, not characterized robot measurements. Live edits
apply on the next velocity update. All fields must be finite and strictly positive;
invalid parameters or flywheel-1 feedback stop output and close feeding.
The cost tolerances do not change the 1% readiness band. There is no integral term,
so an inaccurate steady-speed model can leave RPM error.

Manual and reverse controls preserve voltage-normalized power. There is no
zero-speed motor timeout: zero RPM continues to request drive toward the target.
Telemetry shows RPM, target, LQR model, gain, requested volts, interval and frequency.
Display refresh is limited to 20 Hz without skipping controller or hardware updates.

Save chosen defaults in `ShooterLqrTuning.kt`; Panels fields and their owning class
are pinned for shared settings with Sloth. Agents must not install, deploy, launch,
or run anything on the robot.

## Local checks

```sh
./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
```

Host checks verify controls, RPM conversion, live model edits, and the 1% transfer
gate. They do not verify physical tuning or shooting performance.
