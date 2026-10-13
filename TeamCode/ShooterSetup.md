# BioBuzz shooter setup

Every new TeleOp starts with **velocity PID enabled**, at a **66% preset**.
The full motor speed is `(3600 / 8) * 10 = 4500 RPM`. In PID, the preset selects
`target motor RPM = preset * 4500`, so the default target is **2970 RPM**.
PID adjusts actual output power as needed to reach that target.

- Gamepad 1 right bumper: run the shooter at the selected preset.
- Either gamepad D-pad up/down: change the preset by 2 percentage points, bounded
  to 0–100%. Each step changes the PID target by 90 RPM.
- Gamepad 1 Y: toggle PID/manual power once per press. Manual uses the same
  percentage as raw motor power. The next new TeleOp starts in PID again.
- Gamepad 1 left bumper: reverse the shooter at -0.8, taking priority over right
  bumper. Reverse/release stops automatic feeding and closes the PID blocker.
- Gamepad 1 right/left trigger: intake forward/reverse. With the shooter running,
  forward feed waits for readiness; when ready, right trigger sets intake power,
  otherwise automatic intake uses the selected preset. With the shooter off,
  right trigger collects normally. Reverse can clear a jam at any time.
- Gamepad 1 A: stop intake, including automatic feeding.
- Either gamepad B: manual blocker open **in manual mode only**. In PID, B and
  B + right trigger cannot bypass the speed gate.

PID opens the blocker and starts forward feeding only after **both** motors stay
within 5% of target RPM for 150 ms. A speed drop immediately closes the blocker
and stops forward feeding; it resumes after the same readiness dwell. The
blocker commands are 0.45 open and 0.24 closed. Manual mode retains the
three-second automatic spin-up timer and the B + trigger feed override; it
cannot verify actual speed.

## Motor polarity

Both intake and shooter pairs now use motor 1 `REVERSE` and motor 2 `FORWARD`
(previously motor 1 `FORWARD` and motor 2 `REVERSE`). This swaps the physical
intake/outtake and shooting/reversing directions: every positive command now
applies the opposite electrical polarity, and every negative command does too.
The motor names, buttons, power magnitudes, RPM targets, and feed gates stay the
same. Shooter Motor Diagnostic uses the same updated directions.

## Encoder configuration

The motor names are `shooter1`, `shooter2`, `intake1`, `intake2`, and `blocker`.
Connect both power and encoder cables to matching hub ports. Select the actual
shooter motor models in the Robot Controller configuration. Each configured
`ticksPerRev` must describe the shaft whose full speed is 4500 RPM:

```text
measured motor RPM = abs(encoder ticks per second) * 60 / encoder ticks per revolution
```

The code uses each motor's own configured counts, with no assumed 28-count
encoder or borrowed gearing. If 4500 RPM describes a different shaft from the
configured motor output, adjust the encoder calibration/transmission before
using the RPM reading. A positive but incorrect motor profile cannot be detected
automatically. Invalid/zero counts stop PID motors and block feeding.

A missing encoder blocks PID feeding and holds bounded preset power while
waiting for feedback. Valid readings restore PID automatically; there is no
permanent intake-disable latch. Non-finite feedback is treated as a fault.
Run **Shooter Motor Diagnostic** to verify both encoder readings and profiles;
run **Blocker Diagnostic** to verify physical servo positions.

See [SHOOTER_TUNING.md](SHOOTER_TUNING.md) for controller details, references,
validation, and Gradle/Sloth deployment commands.
