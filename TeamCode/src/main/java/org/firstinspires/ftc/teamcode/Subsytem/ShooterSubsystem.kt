package org.firstinspires.ftc.teamcode.Subsytem

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.core.subsystems.Subsystem

class ShooterSubsystem(hardwareMap: HardwareMap) : Subsystem {
    private val shooter1 = hardwareMap.get(DcMotorEx::class.java, "shooter1")
    private val shooter2 = hardwareMap.get(DcMotorEx::class.java, "shooter2")
    private val blocker = hardwareMap.get(Servo::class.java, "blocker")

    init {
        shooter1.direction = DcMotorSimple.Direction.FORWARD
        shooter2.direction = DcMotorSimple.Direction.REVERSE
        shooter1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        shooter2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun setFlywheelPower(power: Double) {
        val clipped = power.coerceIn(-1.0, 1.0)
        shooter1.power = clipped
        shooter2.power = clipped
    }

    fun setBlockerPosition(open: Boolean) {
        blocker.position = if (open) 0.24 else 0.45
    }
}
