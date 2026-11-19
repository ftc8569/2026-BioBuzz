# Auto Dog: chase POLLEN and NECTAR

The robot drives itself onto the nearest ball while a driver holds A. Everything runs on
the robot: a wide global shutter webcam on the Control Hub finds the balls, a tracker
follows each one in field coordinates using Pinpoint odometry, and a controller drives the
intake onto the nearest.

```
global shutter webcam --USB--> Control Hub
  BallVisionProcessor   camera thread: yellow / red / blue round blobs, per frame
  BallTracker           OpMode loop: pixel -> field projection, one Kalman filter per ball
  ChaseDriver           turn toward the target, then drive onto it with the intake on
```

The tracker is a Kotlin port of [ftc8569/BioBuzzAutoDog](https://github.com/ftc8569/BioBuzzAutoDog)
(`python/autodog/`), which also has the design doc, the simulator and the reasons behind
each tuning default.

## Files (`TeamCode/.../teamcode/autodog/`)

- `AutoDogChaseTeleOp.kt`: the "Auto Dog Chase" TeleOp (group Test)
- `BallVision.kt`: `BallDetector` (HSV blobs with a roundness check) and the
  `BallVisionProcessor` that runs it on the VisionPortal camera thread
- `BallTracker.kt`, `BallTrack.kt`, `Projector.kt`, `TrackerConfig.kt`: the tracker
- `FrameMath.kt`: frame time stamps and pixel bookkeeping
- `ChaseController.kt`: the control law. `AutoDogTuning.kt`: every tunable, and `ChaseDriver`
- `AutoDogLaptopChaseTeleOp.kt`, `AutoDogLink.kt`: the laptop-tracker debugging setup (below)

Tests are in `TeamCode/src/test/.../autodog/`:

- `BallTrackerGoldenTest` replays a simulated chase recorded from the Python tracker
  (`src/test/resources/autodog/tracker_golden.txt`) and checks the Kotlin tracker produces
  the same tracks and targets on every frame.
- `BallDetectorTest` runs the real detector on drawn camera frames using desktop OpenCV
  (`org.openpnp:opencv`, a test-only dependency), including a camera frame to chase target
  run with the camera upright and upside down.
- `OnRobotTrackerTest` and `AutoDogTest` cover projection, the pose buffer, the driver
  interface, the controller and the laptop link.

## Hardware configuration

- A webcam named **`globalShutter`** on the Control Hub (a USB 3 port if one is free).
- `pinpoint`, the drive motors and `intake1` / `intake2`, as in the other OpModes.

The camera runs at 1280x720 MJPEG. The built-in intrinsics (`CameraIntrinsics.GLOBAL_SHUTTER_720P`)
come from the chessboard calibration in BioBuzzAutoDog, about 117 degrees wide.

## Controls (gamepad 1)

| Input | Action |
| --- | --- |
| Hold A | Chase the target with the intake on. With no target the robot stops. Releasing A is the dead-man switch. |
| Hold A + B | Same, but spin slowly while there is no target |
| Sticks, A released | Normal manual drive |

The robot also stops if no camera frame has arrived for 250 ms (`maxTargetAgeS`).
The speed caps start low (forward 0.35, turn 0.40).

## Running it

1. Measure the camera on the robot and set these in Panels Configurables, under
   `AutoDogTuning`: `camForwardM`, `camLeftM`, `camHeightM` (lens position from the robot
   center and the floor), `camPitchDownDeg`, and `camUpsideDown` if it is mounted inverted.
   They are read when START is pressed. The defaults are placeholders.
2. Keep the robot still during init: `RobotLocationSubsystem` resets the Pinpoint IMU. Set
   `startXInches`, `startYInches` and `startHeadingDeg` to the robot's starting pose in the
   team's center-origin frame. Off a real field, leave them at 0 and set `gateField` to
   false, so balls "outside the field" are kept.
3. Init "Auto Dog Chase". During init the telemetry shows the camera state and how many
   yellow, red and blue balls it sees, and the Driver Station camera stream circles each
   detection. Check both before pressing START.
4. Press START, then hold A.

Telemetry goes to the Driver Station, to Panels at `http://192.168.43.1:8001`, and to logcat
(`adb connect 192.168.43.1:5555; adb logcat -s AutoDog`).

## Before trusting it

None of this has run on the robot yet. The tests check the math and the detector on drawn
frames, not the real camera.

- **Mount.** Range comes from the camera's height and pitch, so a few degrees of pitch
  error moves a far ball a long way. Check it by placing a ball at a measured spot and
  comparing the "Target" telemetry with the tape measure.
- **Color thresholds.** The HSV ranges in `AutoDogVision` are starting points. Tune them
  live from Panels while watching the Driver Station camera stream, under the real lights.
- **Small balls.** The lens is wide, so a POLLEN 3 m away is only about 9 px across.
  `AutoDogVision.minAreaPx` (40) sets the smallest blob accepted. Raise it if specks get
  through; lower range is the cost.
- **Frame rate.** Detection runs on the full 1280x720 image. Telemetry shows "detect N ms"
  and the camera fps. If the Control Hub is too slow, set `AutoDogVision.shrink` to 2: it
  is faster but halves the range.
- **Latency.** Frames are stamped by the camera driver and the pose is looked up at that
  time. If balls smear sideways while the robot turns, raise `captureLatencyS`.
- **Intrinsics.** Recalibrate if the camera or lens differs from the one that was
  calibrated, or if a resolution with another aspect ratio is used.
- **Ramps.** `TrackerConfig.rampPolygons` is empty. Anything not on the flat floor is
  projected to the wrong place, so measure the field's keep-out areas and add them.

## Laptop tracker (debugging only)

"Auto Dog Chase (laptop tracker)" is the earlier setup: the Python tracker runs on a laptop
on the robot's Wi-Fi, reads the Limelight's video stream (`http://192.168.43.1:5800`) and
sends targets back over UDP (laptop 47800, robot 47801). It is useful for the Python
tracker's top-down debug view and run logs, and it is not legal at events.

```
python autodog_run.py robot --rc 192.168.43.1 --source limelight \
    --height <lens height m> --pitch <deg down> --cam-x <m ahead of center> --show --log runs/
```

`AutoDogLink.kt` must match `python/autodog/link.py` byte for byte.
