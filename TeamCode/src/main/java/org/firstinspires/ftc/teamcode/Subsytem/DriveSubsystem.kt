package org.firstinspires.ftc.teamcode.Subsytem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.nextftc.core.subsystems.Subsystem
import kotlin.math.abs

class DriveSubsystem(hardwareMap: HardwareMap) : Subsystem {
    private val frontLeft = hardwareMap.get(DcMotor::class.java, "driveFL")
    private val frontRight = hardwareMap.get(DcMotor::class.java, "driveFR")
    private val backLeft = hardwareMap.get(DcMotor::class.java, "driveBL")
    private val backRight = hardwareMap.get(DcMotor::class.java, "driveBR")

    init {
        frontLeft.direction = DcMotorSimple.Direction.REVERSE
        backLeft.direction = DcMotorSimple.Direction.REVERSE
    }

    fun drive(forward: Double, strafe: Double, turn: Double) {
        val denominator = maxOf(abs(forward) + abs(strafe) + abs(turn), 1.0)
        frontLeft.power = (forward + strafe + turn) / denominator * 0.9
        frontRight.power = (forward - strafe - turn) / denominator * 0.9
        backLeft.power = (forward - strafe + turn) / denominator * 0.9
        backRight.power = (forward + strafe - turn) / denominator * 0.9
    }

    fun stop() = drive(0.0, 0.0, 0.0)
}
