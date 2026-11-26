package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.teleop.outputWrites

/** Owns the four drive motors and caches output commands for the shared loop. */
class DriveSubsystem(hardwareMap: HardwareMap) {
    private val motors = listOf("driveFL", "driveFR", "driveBL", "driveBR")
        .map { hardwareMap.get(DcMotor::class.java, it) }
    private var phasedIo = false
    private val desired = DoubleArray(4)
    private val committed = DoubleArray(4) { Double.NaN }
    private var threshold = 0.0

    internal fun enablePhasedIo() {
        phasedIo = true
        // Establish modes once; no recurring mode/direction traffic in the loop.
        motors.forEachIndexed { index, motor ->
            motor.direction = if (index == 0 || index == 2) DcMotorSimple.Direction.REVERSE
                else DcMotorSimple.Direction.FORWARD
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        }
        forceStop()
    }

    /** Both manual and Pedro requests go through this one output cache. */
    internal fun requestWheelPowers(powers: DoubleArray, powerThreshold: Double = 0.0) {
        require(powers.size == 4)
        for (i in desired.indices) {
            desired[i] = if (powers[i].isFinite()) powers[i].coerceIn(-1.0, 1.0) else 0.0
        }
        threshold = powerThreshold
        // Intake assist computes in this frame, so expose what the writer will actually
        // send/retain, including Pedro's deadband against the last successful command.
        commandedPowerMagnitude = desired.indices.maxOf { i ->
            abs(if (shouldWrite(i)) desired[i] else committed[i])
        }
        if (!phasedIo) commitOutputs(force = true)
    }

    private fun shouldWrite(i: Int): Boolean {
        val previous = committed[i]
        val value = desired[i]
        return previous.isNaN() || (value != previous &&
            (abs(value - previous) >= threshold ||
                kotlin.math.sign(value) != kotlin.math.sign(previous)))
    }

    internal fun commitOutputs(force: Boolean = false) {
        outputWrites(force, motors.size) { i ->
            val motor = motors[i]
            val value = desired[i]
            if (force || shouldWrite(i)) {
                motor.power = value
                committed[i] = value
            }
        }
        commandedPowerMagnitude = committed.maxOf { if (it.isFinite()) abs(it) else 0.0 }
    }

    internal fun forceStop() {
        desired.fill(0.0)
        commitOutputs(force = true)
    }

    var commandedPowerMagnitude = 0.0
        private set

    init {
        motors[0].direction = DcMotorSimple.Direction.REVERSE
        motors[2].direction = DcMotorSimple.Direction.REVERSE
    }

    fun drive(forward: Double, strafe: Double, turn: Double) {
        val denominator = maxOf(abs(forward) + abs(strafe) + abs(turn), 1.0)
        desired[0] = (forward + strafe + turn) / denominator * 0.9
        desired[1] = (forward - strafe - turn) / denominator * 0.9
        desired[2] = (forward - strafe + turn) / denominator * 0.9
        desired[3] = (forward + strafe - turn) / denominator * 0.9
        requestWheelPowers(desired)
    }

    fun stop() = drive(0.0, 0.0, 0.0)
}
