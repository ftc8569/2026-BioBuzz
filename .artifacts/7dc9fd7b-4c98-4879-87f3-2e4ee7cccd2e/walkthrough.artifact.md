# TeleOp Implementation and Subsystem Fixes

Implemented a functional TeleOp OpMode and updated the drivetrain subsystem to support command-based control.

## Changes Made

### Drivetrain Subsystem
- **Updated [Drivetrain.kt](file:///C:/Users/shrey/OneDrive/Documents/GitHub/2026-BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Subsystems/Drivetrain.kt)**:
    - Overrode `defaultCommand` as a `var`. This allows the OpMode to set a custom default command (like joystick driving) during TeleOp.
    - Added necessary imports for `Command`.

### TeleOp OpMode
- **Fixed [FirstTelop.kt](file:///C:/Users/shrey/OneDrive/Documents/GitHub/2026-BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/teleop/FirstTelop.kt)**:
    - Removed hardware instantiation from the `periodic()` loop (which causes performance issues and memory errors).
    - Implemented `MyTeleop` using the NextFTC `NextOpMode` base class.
    - Configured the Drivetrain to move using `gamepad1` left stick (drive/strafe) and right stick (turn).
    - Bound Gamepad A to the Intake command and Gamepad B to the Outtake command.
    - Used `whileTrue` bindings so the mechanisms run while the buttons are held and stop when released.

## Verification
- Successfully built the project using `:TeamCode:assembleDebug`.
- The code now adheres to NextFTC v2 best practices:
    - Hardware is managed in Subsystems.
    - Inputs are managed via Commands and Triggers.
    - The main loop remains clean of low-level hardware logic.
