# HIVE AprilTag localization

The TeleOp reads one Limelight frame at a time. It estimates field X/Y/heading and a rotation angle for each visible HIVE from the frame's AprilTag poses. No odometry or measurements from earlier frames enter the estimate. Panels displays the raw pose, tag IDs, fit residual, frame age, and HIVE angles. A missing or stale frame removes the pose from the field view.

## Robot and camera setup

1. Name the Limelight 3A `limelight` in the Robot Controller configuration.
2. Set pipeline 0 to **Fiducial Markers**, family **AprilTag 36h11**, tag size **3.25 in (82.55 mm)**, and enable **Full 3D**. Include IDs 30–45. The native `botpose` assumes a static field map and is not used here.
3. Measure the camera lens position relative to the robot origin. In Panels Configurables, set `HiveCameraMount` forward, left, and height in inches; yaw left, pitch up, and roll in degrees. Then set `configured = true`. The default 12 in height is a placeholder. The code uses Limelight's camera pose in tag space, so Limelight's robot-space camera setting does not enter this calculation.
4. Run `BioBuzz NextFTC Kotlin TeleOp`, connect to Panels, and open its Telemetry and Field widgets.

Field coordinates are meters internally and inches on Panels: origin at the field center, +X toward the blue HIVE, +Y away from the audience, heading 0 along +X with counterclockwise positive rotation. `Red HIVE delta deg` and `Blue HIVE delta deg` are signed rotations **relative to the initial pose in FIRST's field CAD**, not the CELL's absolute tilt from horizontal. Each HIVE angle is displayed only while that HIVE is visible.

The model uses the four sticker assembly transforms from FIRST's [official STEP CAD](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/field-cad-step), the 43.95 in pivot height, and the tag spacing and ID order from [Competition Manual §§9.6 and 9.9](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20TU02.htm). The sticker origins and tag spacing are nominal. The camera-space adapter uses Limelight's legacy right/down/out camera and target axes. Validate against measured robot positions and both stable HIVE positions on a real field; adjust the model if the assembled field differs from the CAD. In particular, verify the Limelight pose axis convention and the local order of the four tag centers by checking a stationary robot from both sides of the field. Check Limelight firmware release notes before updating to a version with different 3D axes.

The solver rejects frames with no usable tags, stale results (over 250 ms), or an excessive geometry residual. With at least three observations, it can discard one inconsistent tag within a frame. It does not carry the previous estimate forward when vision fails.
