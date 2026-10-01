package org.firstinspires.ftc.teamcode.Subsytem

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.nextftc.core.subsystems.Subsystem

class IntakeSubsystem(hardwareMap: HardwareMap) : Subsystem {
    // HardwareMap is the robot's address book. These names must match the names
    // used in the Robot Controller configuration.
    private val intake1 = hardwareMap.get(DcMotorEx::class.java, "intake1")
    private val intake2 = hardwareMap.get(DcMotorEx::class.java, "intake2")

    init {
        // The motors are mounted opposite each other, so one direction is reversed
        // to make both intake wheels pull game pieces the same way.
        intake1.direction = DcMotorSimple.Direction.FORWARD
        intake2.direction = DcMotorSimple.Direction.REVERSE

        // BRAKE makes the wheels resist spinning when power becomes zero.
        // This stops the intake more quickly than letting it coast.
        intake1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intake2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun setPower(power: Double) {
        // Motor power must be between -1.0 (full reverse) and 1.0 (full forward).
        // Clipping protects the motors if a caller accidentally gives a larger value.
        val clipped = power.coerceIn(-1.0, 1.0)
        intake1.power = clipped
        intake2.power = clipped
    }
}
