package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.hardware.FlywheelFeedback
import org.firstinspires.ftc.teamcode.control.ShooterController
import org.firstinspires.ftc.teamcode.control.ShooterController.ControlMode
import org.firstinspires.ftc.teamcode.diagnostics.LoopProfiler
import org.firstinspires.ftc.teamcode.diagnostics.LoopStage
import org.firstinspires.ftc.teamcode.diagnostics.measureOrRun
import org.firstinspires.ftc.teamcode.hardware.BatteryVoltageMonitor
import org.firstinspires.ftc.teamcode.hardware.outputWrites

/** Owns two flywheels and their blocker; shot decisions live in the shared controller. */
class ShooterSubsystem(hardwareMap: HardwareMap, nanoTime: () -> Long = System::nanoTime) {
    val control = ShooterController(nanoTime)
    private val motors = listOf("shooter1", "shooter2")
        .map { hardwareMap.get(DcMotorEx::class.java, it) }
    private val blocker = hardwareMap.get(Servo::class.java, "blocker")
    val wheels = motors.map {
        FlywheelFeedback(it.motorType.ticksPerRev, it.motorType.achieveableMaxTicksPerSecond)
    }
    internal val batteryVoltage =
        BatteryVoltageMonitor(hardwareMap.getAll(VoltageSensor::class.java))
    internal var loopProfiler: LoopProfiler? = null
        set(value) { field = value; control.profiler = value; batteryVoltage.profiler = value }
    private var phasedIo = false
    private var frameVoltage = 12.0
    private val committed = DoubleArray(3) { Double.NaN }
    val flywheelPower: Double
        get() = (control.normalizedPower * (12.0 / frameVoltage)).coerceIn(-1.0, 1.0)

    init {
        motors.forEachIndexed { i, motor ->
            motor.direction = if (i == 0) DcMotorSimple.Direction.FORWARD
                else DcMotorSimple.Direction.REVERSE
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        forceStop()
    }

    fun startVoltagePolling() = batteryVoltage.start()
    fun stopVoltagePolling() = batteryVoltage.close()
    internal fun enablePhasedIo() { phasedIo = true; forceStop() }

    internal fun beginFrame() {
        frameVoltage = batteryVoltage.voltage
        loopProfiler.measureOrRun(LoopStage.SHOOTER_ENCODERS) {
            motors.forEachIndexed { i, motor ->
                wheels[i].sample(motor.currentPosition, motor.velocity)
            }
        }
    }

    internal fun commitOutputs(force: Boolean = false) {
        // Forced shutdown attempts every output, even if a motor or servo write fails.
        outputWrites(force, 2) { group ->
            val stage = if (group == 0) LoopStage.SHOOTER_OUTPUT else LoopStage.BLOCKER_OUTPUT
            loopProfiler.measureOrRun(stage) {
                outputWrites(force, if (group == 0) 2 else 1) { motorIndex ->
                    val i = if (group == 0) motorIndex else 2
                    val value = if (i < 2) flywheelPower else control.blockerPosition
                    if (force || value != committed[i]) {
                        if (i < 2) motors[i].power = value else blocker.position = value
                        committed[i] = value
                    }
                }
            }
        }
    }

    fun forceStop() {
        control.stop()
        commitOutputs(force = true)
    }

    fun updateShot(power: Double, mode: ControlMode): Boolean {
        if (!phasedIo) beginFrame()
        val ready = control.updateShot(power, mode, wheels[0].rpm)
        flushImmediate()
        return ready
    }

    fun setFlywheelPower(power: Double) {
        if (!phasedIo) beginFrame()
        control.setFlywheelPower(power)
        flushImmediate()
    }

    fun setControlMode(mode: ControlMode) {
        control.setControlMode(mode)
        flushImmediate()
    }

    fun setManualBlockerOpen(open: Boolean) {
        control.setManualBlockerOpen(open)
        flushImmediate()
    }

    private fun flushImmediate() {
        if (!phasedIo) {
            frameVoltage = batteryVoltage.voltage
            commitOutputs()
        }
    }

}
