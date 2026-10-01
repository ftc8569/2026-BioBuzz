package org.firstinspires.ftc.teamcode.Subsytem

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.core.subsystems.Subsystem
import kotlin.math.abs

class ShooterSubsystem(hardwareMap: HardwareMap) : Subsystem {
    enum class ControlMode { MANUAL_POWER, VELOCITY_PID }

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
    var blockerOpen = false
        private set
    var blockerStatus = "Shooter off"
        private set

    private var shotStartedAtNanos = 0L
    var controlMode = ControlMode.MANUAL_POWER
        private set

    companion object {
        private const val BLOCKER_OPEN_DELAY_NANOS = 500_000_000L
        private const val BLOCKER_OPEN = 0.45
        private const val BLOCKER_CLOSED = 0.24
    }

    init {
        shooter1.direction = DcMotorSimple.Direction.FORWARD
        shooter2.direction = DcMotorSimple.Direction.REVERSE
        shooter1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        shooter2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        shooter1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        shooter2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        closeBlocker()
    }

    /** Runs both flywheels and opens the blocker half a second after shooting starts. */
    fun updateShot(power: Double, mode: ControlMode): Boolean {
        val requestedPower = power.coerceIn(0.0, 1.0)
        if (requestedPower <= 0.0) {
            setFlywheelPower(0.0)
            return false
        }

        switchMode(mode)
        val now = System.nanoTime()
        if (shotStartedAtNanos == 0L) shotStartedAtNanos = now
        targetSpeed1 = requestedPower * maxTicksPerSecond1
        targetSpeed2 = requestedPower * maxTicksPerSecond2
        if (mode == ControlMode.VELOCITY_PID) {
            shooter1.velocity = targetSpeed1
            shooter2.velocity = targetSpeed2
        } else {
            shooter1.power = requestedPower
            shooter2.power = requestedPower
        }
        measuredSpeed1 = abs(shooter1.velocity)
        measuredSpeed2 = abs(shooter2.velocity)
        blockerOpen = now - shotStartedAtNanos >= BLOCKER_OPEN_DELAY_NANOS
        blockerStatus = if (blockerOpen) "Open after 0.5 s" else "Waiting 0.5 s"
        if (blockerOpen) openBlocker() else closeBlocker()
        return blockerOpen
    }

    fun setFlywheelPower(power: Double) {
        val clipped = power.coerceIn(-1.0, 1.0)
        switchMode(ControlMode.MANUAL_POWER)
        closeBlocker()
        shooter1.power = clipped
        shooter2.power = clipped
        targetSpeed1 = 0.0
        targetSpeed2 = 0.0
        measuredSpeed1 = abs(shooter1.velocity)
        measuredSpeed2 = abs(shooter2.velocity)
        shotStartedAtNanos = 0L
        blockerOpen = false
        blockerStatus = "Shooter off or reversing"
    }

    private fun switchMode(mode: ControlMode) {
        if (mode == controlMode) return
        shooter1.power = 0.0
        shooter2.power = 0.0
        closeBlocker()
        blockerOpen = false
        blockerStatus = "Changing shooter mode"
        shotStartedAtNanos = 0L
        val runMode = if (mode == ControlMode.VELOCITY_PID) {
            DcMotor.RunMode.RUN_USING_ENCODER
        } else {
            DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        shooter1.mode = runMode
        shooter2.mode = runMode
        controlMode = mode
    }

    private fun openBlocker() { blocker.position = BLOCKER_OPEN }
    private fun closeBlocker() { blocker.position = BLOCKER_CLOSED }
}
