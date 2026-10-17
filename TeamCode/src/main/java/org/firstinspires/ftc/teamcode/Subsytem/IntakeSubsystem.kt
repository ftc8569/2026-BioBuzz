package org.firstinspires.ftc.teamcode.Subsytem

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.nextftc.core.subsystems.Subsystem

class IntakeSubsystem(hardwareMap: HardwareMap) : Subsystem {
    private val intake1 = hardwareMap.get(DcMotorEx::class.java, "intake1")
    private val intake2 = hardwareMap.get(DcMotorEx::class.java, "intake2")

    private val motors = listOf(intake1, intake2)
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
    internal fun forceStop() { desiredPower = 0.0; commitOutputs(force = true) }

    init {
        // Reverse both motor polarities while preserving logical forward/reverse controls.
        intake1.direction = DcMotorSimple.Direction.REVERSE
        intake2.direction = DcMotorSimple.Direction.FORWARD
        // A previous OpMode may leave an encoder/position mode active on these ports.
        intake1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intake2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intake1.power = 0.0
        intake2.power = 0.0
        intake1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intake2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun setPower(power: Double) {
        val clipped = if (power.isFinite()) power.coerceIn(-1.0, 1.0) else 0.0
        desiredPower = clipped
        if (!phasedIo) {
            intake1.power = clipped
            intake2.power = clipped
        }
    }
}
