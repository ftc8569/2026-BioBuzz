package org.firstinspires.ftc.teamcode.Subsytem

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.core.subsystems.Subsystem

class ShooterSubsystem(hardwareMap: HardwareMap) : Subsystem {
    // These names must match the motor and servo names in the Robot Controller
    // hardware configuration.
    private val shooter1 = hardwareMap.get(DcMotorEx::class.java, "shooter1")
    private val shooter2 = hardwareMap.get(DcMotorEx::class.java, "shooter2")
    private val blocker = hardwareMap.get(Servo::class.java, "blocker")

    init {
        // The shooter motors face opposite directions. Reversing one lets both
        // flywheels spin together in the direction that launches the game piece.
        shooter1.direction = DcMotorSimple.Direction.FORWARD
        shooter2.direction = DcMotorSimple.Direction.REVERSE

        // BRAKE makes the flywheels stop instead of freely coasting when power
        // is removed.
        shooter1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        shooter2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun setFlywheelPower(power: Double) {
        // FTC motor power ranges from -1.0 to 1.0. Both flywheels receive the
        // same requested power so they work as a pair.
        val clipped = power.coerceIn(-1.0, 1.0)
        shooter1.power = clipped
        shooter2.power = clipped
    }

    fun setBlockerPosition(open: Boolean) {
        // A servo does not spin continuously like a motor. Its position normally
        // ranges from 0.0 to 1.0. These two values were chosen for this robot's
        // physical blocker mounting and may need tuning if the mechanism changes.
        blocker.position = if (open) 0.24 else 0.45
    }
}
