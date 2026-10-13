# Shooter RPM PID and validation

## Target and units

The shooter now starts in PID at 66%. It uses the supplied full-speed calculation:

```text
maximum motor RPM = (3600 / 8) * 10 = 4500
RPM target = selected fraction * 4500
66% -> 2970 RPM; 68% -> 3060 RPM; 100% -> 4500 RPM
measured RPM = abs(DcMotorEx.getVelocity()) * 60 / motorType.ticksPerRev
```

Both motors have the same RPM target, but each converts its own encoder reading
using its own motor profile. The configured achievable maximum is displayed for
diagnostics and does not cap this software target. Correct motor configuration
is required: ticks/rev and 4500 RPM must refer to the same shaft. There is no
assumed encoder count or flywheel transmission multiplier. Motor model and
physical calibration still require confirmation on the robot.

The previous spin-up capture and saved measured targets have been removed.
Changing the preset changes RPM immediately. Battery sag, bumper release, mode
changes, and shots cannot redefine the target by learning a lower speed.

## Software PIDF

Both shooters use `RUN_WITHOUT_ENCODER` and `setPower`; encoder readings remain
available. The software loop is the only velocity controller. PID is enabled
by the selected control mode, rather than the SDK hub's `RUN_USING_ENCODER` loop.
Each motor has its own integral state, reset on release, reverse, preset/mode
changes, and feedback faults.

For target `T`, measured motor RPM `V`, and elapsed time `dt` in seconds:

```text
error = T - V
kF = 1 / 4500
kP = 0.6 / 4500
kI = 0.25 / 4500
output = kF*T + kP*error + integralPower
integralPower += kI*error*dt
```

Gains are constant across presets. `dt` is capped at 0.1 seconds, integral power
is bounded to +/-0.20, and integration freezes when it would worsen output
saturation. Forward output is clamped to [0, 1]. D remains zero until actual
encoder noise is measured. These are initial gains, requiring physical tuning;
host simulation checks behavior but cannot establish robot-specific gains.

## Blocker and intake

Both motors must be within 5% of the RPM target for 150 continuous milliseconds
before the blocker opens and forward feeding starts. Falling outside tolerance
immediately resets the dwell, closes the blocker, and stops forward intake.
Recovery resumes feeding automatically. B never overrides this gate in PID,
including while idle. Y toggles to manual power, where the old three-second
timed readiness and B override remain available.

Right trigger controls forward intake power once ready; without a trigger,
automatic feeding uses the preset. A stops intake. Intake collection with the
shooter off and reverse jam clearing remain available. Both intake motors are
initialized to `RUN_WITHOUT_ENCODER` and zero power so earlier OpModes cannot
leave a position/encoder mode active. Invalid intake commands become zero.

Non-finite feedback faults immediately. Zero feedback on either motor faults
after a three-second startup grace, or immediately once both encoders have
reported motion. Faults close feeding, reset controllers, and hold bounded raw
preset power while feedback is unavailable. They do not latch forever: valid
feedback restores RPM control and requires a fresh dwell before feeding.
Invalid encoder counts stop PID motors instead of attempting RPM control.
Targets remain defined by the requested percentage, including during faults.

## References checked

- [FTC SDK DcMotorEx API](https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotorEx.html): the no-argument velocity getter reports ticks per second; angular-unit overloads and hub PIDF APIs are separate interfaces.
- [FIRST motor modes and encoders](https://ftc-docs.firstinspires.org/en/latest/tech_tips/tech-tips/tech-tip-motor-modes/tech-tip-motor-modes.html): explains power versus hub velocity control and the importance of motor configuration.
- [ftc8569 reference shooter](https://github.com/ftc8569/2025-Decode/blob/017bf74fad8a5089da38294f19b21aed42a174b7/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/common/subsystems/Shooter.kt): compares actual and target RPM and clamps software PIDF output. Its readiness comparison uses `> 0.03` despite a less-than comment; BioBuzz uses `<= 0.05` on both motors with dwell. Its robot-specific gains and gearing are not copied.
- The local Decode shooter uses an LQR in rad/s, with a 28-count assumption in its IO. That controller and calibration do not supply BioBuzz's RPM PID gains.
- [Kotlin Gradle compatibility](https://kotlinlang.org/docs/gradle-configure-project.html): Kotlin 2.3.21 supports Gradle through 9.3 and AGP through 9.0.
- [Gradle 9.3.0 release](https://docs.gradle.org/9.3.0/release-notes.html).

## Build and Sloth

Build versions: JDK 17, Gradle **9.3.0**, AGP 8.13.2, Kotlin **2.3.21**, NextFTC
1.0.0, FTC SDK 12.0.0, Sloth/Load 0.3.2. The wrapper verifies the official Gradle
9.3.0 distribution checksum. Kotlin plugin and stdlib versions match. TeamCode compilation and host tests
use a JDK 17 toolchain, even when Gradle is launched with a newer JDK. Generated
Kotlin project state is ignored by Git. Sloth/Load remain paired at 0.3.2.

Host validation:

```sh
JAVA_HOME=/Library/Java/JavaVirtualMachines/temurin-17.jdk/Contents/Home ./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug :TeamCode:assembleRelease :TeamCode:lintDebug :TeamCode:assembleSloth
```

Use JDK 17 in Android Studio's Gradle settings. The path above is this Mac's
installed JDK; other machines should use their own JDK 17 location.

For initial Sloth setup or changed dependencies, connect the Robot Controller
over ADB and perform a full install:

```sh
./gradlew :TeamCode:installDebug
```

Load removes old remote Sloth bundles before full installation. Later TeamCode
changes can use `./gradlew :TeamCode:deploySloth`; dependency/resource/controller
changes require another full install. `assembleSloth` builds without a robot.
No additional robot-side Sloth OpMode is needed.

## Validation results

All 20 host tests passed with zero failures. Debug and release APK builds,
Android lint, and the Sloth bundle build passed on this Mac. Lint reports
dependency-update, bundled SDK native-alignment, and unused-resource warnings,
with no errors. Robot installation
and on-hardware tuning have not been performed.

## Robot verification

Host tests cover RPM targets and conversion, both-wheel dwell, shot recovery,
feedback recovery without release, PID override prevention, trigger/automatic
intake decisions, mode changes, reverse/stop, invalid data, intake motor modes,
and a simulated flywheel under battery sag and ball impact. Host validation
does not verify encoder wiring, physical gearing, servo travel, or shot accuracy.

1. Confirm motor models, counts/rev, and the shaft corresponding to 4500 RPM.
2. Full-install the updated APK, initialize TeleOp, and verify PID/66% startup.
3. With no game pieces, hold right bumper: verify target 2970 RPM, both actual
   RPM readings, and readiness after 150 ms within tolerance. B + trigger must
   not open/feed early. D-pad steps should change the target by 90 RPM.
4. Fire a shot and verify speed loss closes/stops feeding, then feeding resumes
   after recovery. Verify A stops intake and reverse clears jams.
5. Check each intake motor turns, then tune gains using measured speed data.
6. Verify blocker travel using Blocker Diagnostic and Sloth reload on hardware.
