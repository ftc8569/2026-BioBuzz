# Auto Dog: chase POLLEN and NECTAR with the Limelight

The robot drives itself onto the nearest ball while a driver holds A. The tracker is
[ftc8569/BioBuzzAutoDog](https://github.com/ftc8569/BioBuzzAutoDog) (`python/autodog/`). It
runs on a laptop on the robot's Wi-Fi, reads the Limelight's video stream, and tracks each
ball in field coordinates with a Kalman filter. The robot streams Pinpoint odometry to it
and drives onto the target it sends back.

This is a practice-field setup. Laptops aren't allowed on the field at events, so use
**Auto Dog Chase (Onboard)** there (see [Onboard version](#onboard-version-no-laptop)).

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

## Onboard version (no laptop)

"Auto Dog Chase (Onboard)" (`AutoDogOnboardTeleOp.kt`) does the laptop's job on the robot, so
it's legal at events. The controls are the same.

```
Limelight 3A runs limelight/autodog_balls.py -> llpython: ball circles in pixels
Control Hub: pixels -> floor point (camera mount) -> field point (Pinpoint pose at capture time)
             -> tracker (alpha-beta per ball) -> nearest ball to the intake -> ChaseController
```

| File | Role |
| --- | --- |
| `limelight/autodog_balls.py` | Limelight SnapScript: HSV masks, circles, llpython output |
| `limelight/test_autodog_balls.py` | Synthetic-frame tests (`python -m pytest limelight/`) |
| `TeamCode/.../autodog/BallVision.kt` | Parsing, projection, pose history, tracker, target selection. Pure Kotlin. |
| `TeamCode/.../autodog/AutoDogOnboardTeleOp.kt` | The OpMode and `AutoDogVisionTuning` |
| `TeamCode/src/test/.../autodog/BallVisionTest.kt` | Unit tests for all of the above |

The laptop and onboard versions share `ChaseController`, `chooseChaseMode` and
`AutoDogTuning`, so they chase the same way.

### Setup

1. In the Limelight UI, add a **Python** pipeline in slot `AutoDogVisionTuning.pipeline`
   (default 1; slot 0 is the HIVE AprilTag pipeline). Paste `limelight/autodog_balls.py` and
   save.
2. **Tune `COLOR_RANGES` in the script.** The defaults (yellow POLLEN, blue NECTAR) are
   placeholders, not measured game-piece colors. Check them on the Limelight stream under
   the real lights.
3. Measure the camera position once in **`HiveCameraMount`** (Panels): forward, left, height
   and pitch, then set `configured = true`. The onboard chase uses the same mount as the HIVE
   localizer, and it won't track anything until the mount is set.
4. Set `AutoDogTuning.startX/Y/HeadingDeg` to the real starting pose, or set
   `AutoDogVisionTuning.fieldGate = false` for off-field testing.

### How a detection becomes a target

- **Projection.** The script reports each ball's center pixel. The robot casts a ray through
  that pixel from the camera mount, using the field of view to get the focal length, and
  intersects it with the plane of ball centers (`ballDiameterInches / 2` above the floor).
  It rejects a ball whose radius in pixels disagrees with its distance by more than
  `radiusTolerance`. That catches balls in the air and same-colored robots.
- **Timing.** The frame is placed with the Pinpoint pose from when it was captured. That time
  is the result's staleness plus the Limelight's capture and targeting latency. The pose
  comes from a short interpolated history, so turning while the frame is in flight doesn't
  move the ball.
- **Tracking.** Each ball is matched to the nearest existing track of the same color within
  `trackGateM`. Each track runs an alpha-beta filter for position and velocity. A track needs
  `confirmHits` sightings before the robot chases it, and it's dropped after `dropAfterS`
  without one.
- **Selection.** The robot picks the confirmed ball nearest the intake, filtered by
  `colorFilter` (0 any, 1 POLLEN, 2 NECTAR). It only switches balls when another is
  `switchMarginM` closer.
- **Safety.** These rules are unchanged: a target older than `AutoDogTuning.maxTargetAgeS`
  never drives the robot, and releasing A always stops the chase.

### Still to check on a real robot

- The HSV ranges for the real POLLEN and NECTAR.
- That the camera-mount signs are right. Put a ball 1 m straight ahead and check that
  telemetry shows it 1 m ahead of the robot pose.
- That the Limelight's 3A field of view matches the pipeline resolution. Calibrate it if
  balls at the edge of the image land in the wrong place.
- Two same-colored balls touching merge into one blob, and the fill check may reject it.
  This is acceptable for chasing, because the robot picks up a neighbor next.
