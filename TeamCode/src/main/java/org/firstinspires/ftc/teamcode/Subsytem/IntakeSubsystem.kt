package org.firstinspires.ftc.teamcode.Subsytem

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.nextftc.core.subsystems.Subsystem

class IntakeSubsystem(hardwareMap: HardwareMap) : Subsystem {
    private val intake1 = hardwareMap.get(DcMotorEx::class.java, "intake1")
    private val intake2 = hardwareMap.get(DcMotorEx::class.java, "intake2")

    init {
        intake1.direction = DcMotorSimple.Direction.FORWARD
        intake2.direction = DcMotorSimple.Direction.REVERSE
        intake1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intake2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun setPower(power: Double) {
        val clipped = power.coerceIn(-1.0, 1.0)
        intake1.power = clipped
        intake2.power = clipped
    }
}
