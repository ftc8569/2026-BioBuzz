# Manual shooter controls

- Hold gamepad 1 right bumper to spin the shooter at the current preset. It starts at `0.80` motor power.
- Tap either gamepad's D-pad up or down to change the preset by `0.02`, within `0.00` to `1.00`.
- Tap gamepad 1 Y to switch between manual motor power (the startup mode) and the hub's encoder velocity PID. The same preset selects manual power or a fraction of the configured motor's achievable maximum encoder speed. PID mode requires both shooter encoders to be connected.
- The blocker starts closed at `0.24` and opens to `0.45` after right bumper has held the shooter on for **0.5 seconds**. Releasing right bumper or reversing the shooter closes it immediately. Changing shooter mode while shooting restarts the delay.
- While right bumper is held, the intake starts with the shooter. Gamepad 1 A stops the intake. Gamepad 1 triggers control the intake when the shooter is off. Hold gamepad 1 left bumper to reverse the shooter and intake with the blocker closed.

Connect both shooter motor encoders and select the correct motor types in the FTC robot configuration before using PID mode. Telemetry displays each measured velocity beside its target and the commanded blocker position. The blocker delay is based on time, not measured flywheel speed. The blocker positions come from DECODE's `IntakeTransfer`: `0.24` engaged/closed and `0.45` disengaged/open. Confirm the motor directions on the robot before feeding a game piece. The reported blocker position is the commanded value; it cannot confirm that the servo physically moved.

To isolate a blocker problem, remove game pieces and run **Blocker Diagnostic** from the Test OpModes. Press gamepad 1 **A** for `0.24` closed and **B** for `0.45` open. This diagnostic does not run the shooter or intake. If the servo moves there but not in the main TeleOp, read the main TeleOp's **Blocker status** and **Blocker commanded position** telemetry.
