package org.firstinspires.ftc.teamcode.Subsytem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.nextftc.core.subsystems.Subsystem
import kotlin.math.abs
import kotlin.math.sign

class DriveSubsystem(hardwareMap: HardwareMap) : Subsystem {
    private val frontLeft = hardwareMap.get(DcMotorEx::class.java, "driveFL")
    private val frontRight = hardwareMap.get(DcMotor::class.java, "driveFR")
    private val backLeft = hardwareMap.get(DcMotor::class.java, "driveBL")
    private val backRight = hardwareMap.get(DcMotor::class.java, "driveBR")

    init {
        frontLeft.direction = DcMotorSimple.Direction.REVERSE
        backLeft.direction = DcMotorSimple.Direction.REVERSE
        listOf(frontLeft, frontRight, backLeft, backRight).forEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    fun drive(forward: Double, strafe: Double, turn: Double) {
        val denominator = maxOf(abs(forward) + abs(strafe) + abs(turn), 1.0)
        val flPower = (forward + strafe + turn) / denominator * 0.9
        frontLeft.power = if (forward == 0.0 && strafe == 0.0 && turn == 0.0) frontLeftHardStop() else flPower
        frontRight.power = (forward - strafe - turn) / denominator * 0.9
        backLeft.power = (forward - strafe + turn) / denominator * 0.9
        backRight.power = (forward + strafe - turn) / denominator * 0.9
    }

    /**
     * The front-left wheel was coasting longer than the other three, so when the sticks are released
     * push against its motion until it has nearly stopped. Returns the power to apply (0 once stopped).
     */
    private fun frontLeftHardStop(): Double {
        val v = frontLeft.velocity   // ticks/s
        return if (abs(v) > STOP_VELOCITY) -sign(v) * STOP_POWER else 0.0
    }

    fun stop() = drive(0.0, 0.0, 0.0)

    private companion object {
        /** Below this speed (ticks/s) the wheel counts as stopped. */
        const val STOP_VELOCITY = 80.0
        /** Reverse power used to kill the coast; raise it if the wheel still drifts, lower it if it twitches backwards. */
        const val STOP_POWER = 0.25
    }
}
