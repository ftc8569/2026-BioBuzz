package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.teleop.outputWrites

/** Owns both intake motors; feeding decisions are shared by TeleOp and autonomous. */
class IntakeSubsystem(hardwareMap: HardwareMap) {
    private val motors = listOf("intake1", "intake2")
        .map { hardwareMap.get(DcMotorEx::class.java, it) }
    private var phasedIo = false
    private var desiredPower = 0.0
    private val committed = DoubleArray(2) { Double.NaN }
    internal fun enablePhasedIo() { phasedIo = true; forceStop() }
    internal fun commitOutputs(force: Boolean = false) {
        outputWrites(force, motors.size) { i ->
            val motor = motors[i]
            if (force || desiredPower != committed[i]) {
                motor.power = desiredPower
                committed[i] = desiredPower
            }
        }
    }
    fun forceStop() { desiredPower = 0.0; commitOutputs(force = true) }

    init {
        motors.forEachIndexed { index, motor ->
            motor.direction = if (index == 0) DcMotorSimple.Direction.REVERSE
                else DcMotorSimple.Direction.FORWARD
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
        forceStop()
    }

    fun setPower(power: Double) {
        desiredPower = if (power.isFinite()) power.coerceIn(-1.0, 1.0) else 0.0
        if (!phasedIo) commitOutputs()
    }
}
