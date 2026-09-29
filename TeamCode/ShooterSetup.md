# Manual shooter controls

- Hold gamepad 1 right bumper to spin the shooter at the current preset. It starts at `0.80` motor power.
- Tap gamepad 2 D-pad up or down to change the preset by `0.02`, within `0.00` to `1.00`.
- The blocker stays closed when the shooter is stopped, reversing, or accelerating. It opens only after **both** flywheels have measured at least 95% of their requested speed for 200 ms. A preset change closes it and starts that check again. Releasing right bumper closes it.
- While right bumper is held, the intake starts with the shooter. The blocker holds game pieces until both flywheels reach speed. Gamepad 1 A stops the intake. Gamepad 1 triggers control the intake when the shooter is off. Hold gamepad 1 left bumper to reverse the shooter and intake with the blocker closed.

Connect both shooter motor encoders and select the correct motor types in the FTC robot configuration. The ready check uses their measured encoder velocity in ticks per second. Telemetry displays each measured velocity beside its target and the commanded blocker position. If an encoder is missing or reads below the threshold, the blocker stays closed. Confirm the blocker servo positions (`0.45` closed, `0.24` open) and motor directions on the robot before feeding a game piece. The reported blocker position is the commanded value; it cannot confirm that the servo physically moved.
