# Shooter control

The shooter uses [FTCLib PIDFController](https://docs.ftclib.org/ftclib/features/controllers),
one controller using flywheel 1 RPM for both motors. There is no custom PID calculation or second Hub
velocity loop. Each robot loop calls `calculate(flywheel1Rpm, targetRpm)` and clamps
its output to motor power between 0 and 1 after normalizing for battery voltage at 12 V.

## Controls

- Hold gamepad 1 right bumper: PIDF shooting.
- Hold gamepad 1 Y: manual power, taking priority over right bumper.
- Release both: stop the shooter and automatic transfer.
- Left bumper: reverse, taking priority over shooting.
- Either D-pad up/down: adjust the preset by 0.005.

## RPM and transfer

Default preset: 0.695. Target: `0.695 * 4500 = 3127.5 RPM`.
The confirmed goBILDA 1:1 gearbox uses 28 encoder counts per output revolution:
`actual RPM = abs(encoder ticks/second) * 60 / 28`.
The active configuration selects `BioBuzzShooter1To1` for both shooters.

While PIDF shooting, forward transfer and the blocker latch on once **flywheel 1
RPM** enters **99% to 101% of target** (3096.225 through 3158.775 RPM at the default
3127.5 RPM target). They remain enabled through later speed drops. The latch
resets on release, reverse, mode/preset changes, or disabled PID gains. Current
at-speed telemetry still reports the actual RPM band independently of the latch.
B and forward trigger cannot bypass the initial speed check. A stops intake
while held; reverse remains available to clear jams.

Manual mode preserves voltage-normalized power, its three-second feed timer, and the B blocker
override. Releasing Y stops manual shooting.

## Panels tuning

Open Panels > **Configurables** > **ShooterPidTuning**:

| Field | Initial value |
| --- | --- |
| kP | 0.004 |
| kI | 0 |
| kD | 0.00001 |
| kF | 0.000203 |

Gains use RPM, seconds, and motor power; they are
not the Hub's built-in velocity coefficient units. These are starting values,
not a claim of physical tuning.

Edits apply to the shared library controller on the next loop and reset its
accumulated error. FTCLib's integration bounds limit I's contribution to one
unit of motor power. All-zero or invalid coefficients stop PID output and
feeding. There is no motor-fault timeout or zero-speed shutdown. Zero RPM
continues to drive the PID; transfer waits for the initial speed check and then stays latched. Invalid numeric RPM inputs
are kept out of the controller so motor commands remain finite.
Telemetry shows actual RPM 1/2, the selected target, active target, controller
state, and the coefficients applied to the shared library controller.

The configurable fields are pinned so Panels and Sloth share the same settings.
Live edits reset on app restart; save tuned defaults in `ShooterPidTuning.kt`.
Changing dependencies or pinned classes requires a full APK update by the user.
Agents must not install, deploy, launch, or run anything on the robot.

## Local checks

```sh
./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
```

Host checks verify controls, RPM conversion, live gain edits, and the 1% transfer
gate. They do not verify physical tuning or shooting performance.
