# Shooter LQR

The velocity shooter uses a one-state discrete LQR instead of FTCLib PIDF. Flywheel 1's measured RPM is the state; both motors receive the same voltage command. Manual shooting, reverse, preset RPM conversion, 1% readiness tolerance and readiness latching are preserved. TeleOp uses its existing once-per-frame cached hardware inputs and final output commit. No additional hardware reads or robot-side characterization routines were added.

`subsystems/ShooterSubsystem.kt` contains the actual motor/blocker implementation,
hardware-free `ShooterController`, LQR math, live tuning and sampled feedback.
`teleop/Shooter.kt` contains only the shared button controls and preset adjustment.
Both auto and TeleOp import and call the same subsystem API.

## Driver controls and manual tuning

The current `yatishgrandhe/manual-shooter` branch (`adb12be`) already supplies the
hold-Y manual behavior, 0.005 preset increments and reverse priority used here.
Those are preserved. The older `ri5w-autos`/`shreyas-josyula` PID implementation,
Y-toggle behavior, larger preset increments and old blocker positions are not
substituted for the current LQR settings.

| Control | Result |
| --- | --- |
| Hold G1 right bumper | Automatic LQR; target = preset × 4500 RPM |
| Hold G1 Y | Manual voltage-normalized power; overrides right bumper |
| Hold G1 left bumper | Reverse at −0.8 normalized power; overrides shooting |
| Either D-pad up/down | Up increases / down decreases by 0.005 per press (22.5 RPM in LQR); holding does not repeat |
| Either B, while in manual mode | Open blocker manually; cannot bypass LQR readiness |
| Release shooting buttons | Stop flywheels and clear shot readiness |

The default preset is **0.72**: **3240 RPM** in LQR, or a requested **8.64 V** in
manual mode. For example, manual mode commands 0.72 power at 12 V and 0.864 at
10 V, capped at 1.0. Manual operation does not regulate measured RPM. Automatic
manual-mode feed readiness waits three seconds; a preset or mode change restarts
that wait. Match intake still applies the 100 ms blocker-opening delay and
0.6 feed power. G1 A retains its intake-stop priority.

`Shooter Tuning` is a new diagnostic in `teleop/Telemetry.kt` that reuses the
**same** `ShooterSubsystem`, `ShooterControls`, LQR implementation and telemetry.
It only requires `shooter1`, `shooter2` and `blocker`. INIT keeps outputs stopped
while allowing D-pad preset adjustment; START enables the held-button controls
above. It does not run an intake or position hold. Driver Station and Panels show
both RPMs, commanded power, mode, preset, target and feed readiness in six short
status rows, plus three tuning instructions. Live parameters remain in Panels
configurables instead of repeating in every telemetry frame.
Changes to `ShooterLqrTuning` in Panels apply on the next LQR update, as in match
TeleOp. Preset adjustments are local to this OpMode run; autos remain at 0.72.

Every active loop refreshes each hub's MANUAL cache once. STOP, partial INIT and
loop failures independently stop both motors, close the blocker and restore cache
modes. Failure telemetry includes the phase and exception; RobotLog retains the
stack. This entry was tested only with fake hardware, not run on the robot.

The older `Blocker Diagnostic` and `Shooter Motor Diagnostic` now use the same
current blocker endpoints as match shooting: **closed 0.78, open 0.65**. The raw
motor diagnostic retains its X/Y/B individual/both-motor controls and trigger
power adjustment; it is separate from the shared-controller `Shooter Tuning`.

## Model and controller

In RPM and volts:

    dRPM/dt = -(kV/kA) RPM + volts/kA
    a = exp(-kV/kA * dt)
    b = (1-a)/kV
    nextRPM = a * RPM + b * volts
    volts = kV * targetRPM + K * (targetRPM - measuredRPM)

K is calculated from the stabilizing positive solution of the scalar discrete algebraic Riccati equation. The cost weights are Q = 1/rpmTolerance² and R = 1/voltageTolerance². The controller uses the observed interval since the preceding velocity update; the first update after a reset uses 20 ms. The scalar solver is closed-form and has no iterative matrix solve in the loop. With one measured velocity state, the result is a model-derived proportional feedback gain plus feedforward; there is no integral term or observer.

Requested voltage is converted using the existing cached battery voltage and clipped to available forward motor power. Negative LQR commands coast at zero power rather than reversing. Invalid model parameters, weights, or flywheel-1 feedback stop both flywheels and clear feeding readiness. Flywheel-2 telemetry remains diagnostic.

## Estimated initial model

The user approved estimated defaults. These have **not** been identified from robot measurements:

| Panels field in ShooterLqrTuning | Initial value | Meaning |
| --- | --- | --- |
| kV | 0.00240 V/RPM | Current configured steady-speed voltage coefficient |
| kA | 0.00015 V/(RPM/s) | Current configured time constant kA/kV = 0.0625 seconds |
| rpmTolerance | 10 RPM | LQR state-error cost scale; smaller means stronger correction |
| voltageTolerance | 3 V | LQR control-effort cost scale; larger allows stronger correction |

The cost tolerances do not change the 1% gate-readiness threshold. Live Panels changes take effect on the next velocity-control update; all four parameters must be finite and strictly positive. The old ShooterPidTuning fields have been removed. The two modes are VELOCITY_LQR and MANUAL_POWER; the unused PID compatibility alias has been removed.

An inaccurate kV can leave steady-state RPM error because there is no integrator. Replace kV/kA with fitted values from existing voltage, RPM and timestamp logs when available: fit V = kV*RPM + kA*dRPM/dt. A steady operating point estimates kV = V/RPM; a voltage step estimates the time constant (time to 63.2% of the total speed change), giving kA = timeConstant*kV. Use actual applied voltage after saturation. These estimates describe the complete installed mechanism, including both motors and its load. They are not a claim of measured performance.

## Local verification

The historical regression suite, run from a temporary folder during branch integration, compares the gain with an independent iterative Riccati solution and simulates spin-up and a shot disturbance with varying loop intervals and 10–14 V saturation. Integration tests cover voltage compensation, live configuration, invalid model/feedback, manual and reverse operation, mode changes, autonomous shooting, and cached hardware boundaries. Simulation verifies the assumed model; no code was deployed or executed on the robot.

References: [WPILib flywheel state-space formulation](https://docs.wpilib.org/en/latest/docs/software/advanced-controls/state-space/state-space-intro.html), [MIT discrete-time LQR derivation](https://underactuated.csail.mit.edu/lqr.html).
