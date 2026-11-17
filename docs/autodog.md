# Auto Dog: chase POLLEN and NECTAR with the Limelight

The robot drives itself onto the nearest ball while a driver holds A. The tracker is
[ftc8569/BioBuzzAutoDog](https://github.com/ftc8569/BioBuzzAutoDog) (`python/autodog/`). It
runs on a laptop on the robot's Wi-Fi, reads the Limelight's video stream, and tracks each
ball in field coordinates with a Kalman filter. The robot streams Pinpoint odometry to it
and drives onto the target it sends back.

This is a practice-field setup. Laptops aren't allowed on the field at events, so moving
detection onto the Limelight (a Python SnapScript) is a later step.

```
Limelight 3A --USB--> Control Hub --Wi-Fi MJPEG :5800--> laptop (autodog_run.py)
Pinpoint pose  --UDP 47801 (RC) <-> 47800 (laptop)-->  target (x, y, vx, vy, color)
```

## Files

- `TeamCode/.../autodog/AutoDogChaseTeleOp.kt`: the "Auto Dog Chase" TeleOp (group Test)
- `TeamCode/.../autodog/AutoDogLink.kt`: UDP link. Its byte layout matches `python/autodog/link.py`.
- `TeamCode/.../autodog/ChaseController.kt`: turn toward the ball, then drive onto it
- `TeamCode/src/test/.../autodog/AutoDogTest.kt`: controller, frame conversion, drive
  direction and UDP loopback tests

## Controls (gamepad 1)

| Input | Action |
| --- | --- |
| Hold A | Chase the target with the intake on. With no fresh target (more than 250 ms old), the robot stops. Releasing A is the dead-man switch. |
| Hold A + B | Same, but spin slowly while there is no target |
| Sticks, A released | Normal manual drive |

The speed caps start low (forward 0.35, turn 0.40). You can tune them and the gains live in
Panels Configurables, under `AutoDogTuning`.

## Running it

1. Keep the robot still during init: `RobotLocationSubsystem` resets the Pinpoint IMU.
   Set `AutoDogTuning.startX/Y/HeadingDeg` to the robot's starting pose in the team's
   center-origin frame, or leave them at 0 and run the laptop with `--no-field-gate`
   when you're not on a real field.
2. Connect the laptop to the robot's Wi-Fi. Check that `http://192.168.43.1:5801` (the
   Limelight UI) loads and `http://192.168.43.1:5800` shows video.
3. Init and start "Auto Dog Chase" on the Driver Station.
4. On the laptop, from `BioBuzzAutoDog/python`:

   ```
   python autodog_run.py robot --rc 192.168.43.1 --source limelight \
       --height <lens height m> --pitch <deg down> --cam-x <m ahead of center> \
       --show --log runs/ [--no-field-gate]
   ```

   The Driver Station should change from "waiting for ping" to "synced".

Telemetry goes to the Driver Station, to Panels at `http://192.168.43.1:8001`, and to logcat
(`adb connect 192.168.43.1:5555; adb logcat -s AutoDog`). The laptop writes
`runs/<time>/ticks.jsonl`, which holds every detection, track and target sent, along with
sampled raw frames.

## Before trusting it

- **Limelight intrinsics.** The defaults are nominal Limelight 3A values (640x480,
  54.5 x 42 degrees). To calibrate, hold a chessboard in front of the Limelight and run
  `python -m autodog.calib capture --source limelight`, then `python -m autodog.calib solve`,
  and pass `--calib limelight_calib.json`.
- **Mount.** Measure the lens height, pitch and forward offset. Then refine them with
  `autodog_run.py mount --source limelight`, using balls on tile corners.
- **Color thresholds.** The HSV ranges in `python/autodog/detect.py` are starting points.
  Tune them under the real lights on frames from `runs/*/frames/`
  (`python -m autodog.detect runs/*/frames/*.jpg` writes the masks).
- **Latency.** Pass the shutter-to-laptop delay as `--capture-latency`. Wi-Fi MJPEG
  is typically 50 to 150 ms.
- **Stream content.** The 5800 stream shows the active pipeline's output. Crosshairs and
  overlays can confuse the color detector, so turn them off or use a pipeline with no
  overlays (`AutoDogTuning.limelightPipeline`).
