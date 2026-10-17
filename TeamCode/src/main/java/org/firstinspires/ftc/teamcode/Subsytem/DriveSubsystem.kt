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

    var commandedPowerMagnitude = 0.0
        private set

    init {
        frontLeft.direction = DcMotorSimple.Direction.REVERSE
        backLeft.direction = DcMotorSimple.Direction.REVERSE
    }

    fun drive(forward: Double, strafe: Double, turn: Double) {
        val denominator = maxOf(abs(forward) + abs(strafe) + abs(turn), 1.0)
        val fl = (forward + strafe + turn) / denominator * 0.9
        val fr = (forward - strafe - turn) / denominator * 0.9
        val bl = (forward - strafe + turn) / denominator * 0.9
        val br = (forward + strafe - turn) / denominator * 0.9
        frontLeft.power = fl
        frontRight.power = fr
        backLeft.power = bl
        backRight.power = br
        commandedPowerMagnitude = maxOf(abs(fl), abs(fr), abs(bl), abs(br))
    }

    fun stop() = drive(0.0, 0.0, 0.0)
}
