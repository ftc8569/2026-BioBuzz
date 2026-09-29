package org.firstinspires.ftc.teamcode.Subsytem

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.core.subsystems.Subsystem
import kotlin.math.abs

class ShooterSubsystem(hardwareMap: HardwareMap) : Subsystem {
    private val shooter1 = hardwareMap.get(DcMotorEx::class.java, "shooter1")
    private val shooter2 = hardwareMap.get(DcMotorEx::class.java, "shooter2")
    private val blocker = hardwareMap.get(Servo::class.java, "blocker")
    private val maxTicksPerSecond1 = shooter1.motorType.achieveableMaxTicksPerSecond
    private val maxTicksPerSecond2 = shooter2.motorType.achieveableMaxTicksPerSecond

    var targetSpeed1 = 0.0
        private set
    var targetSpeed2 = 0.0
        private set
    var measuredSpeed1 = 0.0
        private set
    var measuredSpeed2 = 0.0
        private set
    val flywheelPower1: Double get() = shooter1.power
    val flywheelPower2: Double get() = shooter2.power
    val blockerCommandedPosition: Double get() = blocker.position
    var atSpeed = false
        private set

    private var lastRequestedPower = 0.0
    private var speedReachedAtNanos = 0L

    companion object {
        private const val READY_SPEED_FRACTION = 0.95
        private const val READY_HOLD_NANOS = 200_000_000L
        private const val BLOCKER_OPEN = 0.45
        private const val BLOCKER_CLOSED = 0.24
    }

    init {
        shooter1.direction = DcMotorSimple.Direction.FORWARD
        shooter2.direction = DcMotorSimple.Direction.REVERSE
        shooter1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        shooter2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        shooter1.mode = DcMotor.RunMode.RUN_USING_ENCODER
        shooter2.mode = DcMotor.RunMode.RUN_USING_ENCODER
        closeBlocker()
    }

    /** Runs both flywheels and opens the blocker only after both reach speed. */
    fun updateShot(power: Double): Boolean {
        val requestedPower = power.coerceIn(0.0, 1.0)
        if (requestedPower <= 0.0) {
            setFlywheelPower(0.0)
            return false
        }

        if (abs(requestedPower - lastRequestedPower) > 1e-6) {
            speedReachedAtNanos = 0L
            atSpeed = false
            closeBlocker()
        }
        lastRequestedPower = requestedPower
        shooter1.power = requestedPower
        shooter2.power = requestedPower

        targetSpeed1 = requestedPower * maxTicksPerSecond1
        targetSpeed2 = requestedPower * maxTicksPerSecond2
        measuredSpeed1 = abs(shooter1.velocity)
        measuredSpeed2 = abs(shooter2.velocity)

        val bothFastEnough = targetSpeed1 > 0.0 && targetSpeed2 > 0.0 &&
            measuredSpeed1.isFinite() && measuredSpeed2.isFinite() &&
            measuredSpeed1 >= targetSpeed1 * READY_SPEED_FRACTION &&
            measuredSpeed2 >= targetSpeed2 * READY_SPEED_FRACTION

        if (!bothFastEnough) {
            speedReachedAtNanos = 0L
            atSpeed = false
        } else {
            val now = System.nanoTime()
            if (speedReachedAtNanos == 0L) speedReachedAtNanos = now
            atSpeed = now - speedReachedAtNanos >= READY_HOLD_NANOS
        }

        if (atSpeed) openBlocker() else closeBlocker()
        return atSpeed
    }

    fun setFlywheelPower(power: Double) {
        val clipped = power.coerceIn(-1.0, 1.0)
        closeBlocker()
        shooter1.power = clipped
        shooter2.power = clipped
        targetSpeed1 = 0.0
        targetSpeed2 = 0.0
        measuredSpeed1 = abs(shooter1.velocity)
        measuredSpeed2 = abs(shooter2.velocity)
        lastRequestedPower = 0.0
        speedReachedAtNanos = 0L
        atSpeed = false
    }

    private fun openBlocker() { blocker.position = BLOCKER_OPEN }
    private fun closeBlocker() { blocker.position = BLOCKER_CLOSED }
}
