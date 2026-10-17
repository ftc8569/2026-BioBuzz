# Shooter LQR

The velocity shooter uses a one-state discrete LQR instead of FTCLib PIDF. Flywheel 1's measured RPM is the state; both motors receive the same voltage command. Manual shooting, reverse, preset RPM conversion, 1% readiness tolerance and readiness latching are preserved. TeleOp uses its existing once-per-frame cached hardware inputs and final output commit. No additional hardware reads or robot-side characterization routines were added.

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
| kV | 0.002436 V/RPM | Previous normalized kF (0.000203) multiplied by 12 V |
| kA | 0.000609 V/(RPM/s) | Assumed time constant kA/kV = 0.25 seconds |
| rpmTolerance | 150 RPM | LQR state-error cost scale; smaller means stronger correction |
| voltageTolerance | 3 V | LQR control-effort cost scale; larger allows stronger correction |

The cost tolerances do not change the 1% gate-readiness threshold. Live Panels changes take effect on the next velocity-control update; all four parameters must be finite and strictly positive. The old ShooterPidTuning fields have been removed. Canonical mode is VELOCITY_LQR; the deprecated VELOCITY_PID enum name delegates to LQR for older callers.

An inaccurate kV can leave steady-state RPM error because there is no integrator. Replace kV/kA with fitted values from existing voltage, RPM and timestamp logs when available: fit V = kV*RPM + kA*dRPM/dt. A steady operating point estimates kV = V/RPM; a voltage step estimates the time constant (time to 63.2% of the total speed change), giving kA = timeConstant*kV. Use actual applied voltage after saturation. These estimates describe the complete installed mechanism, including both motors and its load. They are not a claim of measured performance.

## Local verification

Tests compare the gain with an independent iterative Riccati solution and simulate spin-up and a shot disturbance with varying loop intervals and 10–14 V saturation. Integration tests cover voltage compensation, live configuration, invalid model/feedback, manual and reverse operation, legacy mode compatibility, autonomous shooting, and cached hardware boundaries. Simulation verifies the assumed model; no code was deployed or executed on the robot.

References: [WPILib flywheel state-space formulation](https://docs.wpilib.org/en/latest/docs/software/advanced-controls/state-space/state-space-intro.html), [MIT discrete-time LQR derivation](https://underactuated.csail.mit.edu/lqr.html).
