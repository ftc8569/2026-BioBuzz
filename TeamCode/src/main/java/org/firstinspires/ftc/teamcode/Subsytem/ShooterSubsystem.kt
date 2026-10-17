package org.firstinspires.ftc.teamcode.Subsytem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.core.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.diagnostics.LoopProfiler
import org.firstinspires.ftc.teamcode.diagnostics.LoopStage
import kotlin.math.abs

class ShooterSubsystem(
    hardwareMap: HardwareMap,
    private val nanoTime: () -> Long = System::nanoTime
) : Subsystem {
    enum class ControlMode {
        MANUAL_POWER, VELOCITY_LQR,
        /** Source compatibility for older autonomous callers; uses the LQR controller. */
        @Deprecated("Use VELOCITY_LQR") VELOCITY_PID
    }

    private val shooter1 = hardwareMap.get(DcMotorEx::class.java, "shooter1")
    private val shooter2 = hardwareMap.get(DcMotorEx::class.java, "shooter2")
    internal val batteryVoltage = BatteryVoltageMonitor(hardwareMap.getAll(VoltageSensor::class.java))
    internal var loopProfiler: LoopProfiler? = null
        set(value) {
            field = value
            batteryVoltage.profiler = value
        }
    private val blocker = hardwareMap.get(Servo::class.java, "blocker")

    private val flywheels = listOf(shooter1, shooter2)
    private var phasedIo = false
    private var frameVoltage = 12.0
    private var desiredPower = 0.0
    private var desiredBlockerPosition = BLOCKER_CLOSED
    private val committedPower = DoubleArray(2) { Double.NaN }
    private var committedBlocker = Double.NaN

    /** TeleOp alone opts in; autonomous keeps the same immediate controller API. */
    internal fun enablePhasedIo() { phasedIo = true; forceStop() }

    internal fun beginFrame() {
        frameVoltage = batteryVoltage.voltage
        sampleSpeeds()
    }

    internal fun commitOutputs(force: Boolean = false) {
        // A forced stop tries both flywheels AND the blocker, even after a device failure.
        outputWrites(force, 2) { group ->
            if (group == 0) timed(LoopStage.SHOOTER_OUTPUT) {
                outputWrites(force, flywheels.size) { i ->
                    if (force || desiredPower != committedPower[i]) {
                        flywheels[i].power = desiredPower
                        committedPower[i] = desiredPower
                    }
                }
            } else timed(LoopStage.BLOCKER_OUTPUT) {
                if (force || desiredBlockerPosition != committedBlocker) {
                    blocker.position = desiredBlockerPosition
                    committedBlocker = desiredBlockerPosition
                }
            }
        }
    }

    fun forceStop() {
        resetSpeedHold()
        manualBlockerOpen = false
        blockerOpen = false
        desiredPower = 0.0
        desiredBlockerPosition = BLOCKER_CLOSED
        commitOutputs(force = true)
    }

    // Counts per shaft revolution: the confirmed 1:1 goBILDA profile uses 28.
    val configuredTicksPerRev1 = shooter1.motorType.ticksPerRev
    val configuredTicksPerRev2 = shooter2.motorType.ticksPerRev
    val configuredMaxTicksPerSecond1 = shooter1.motorType.achieveableMaxTicksPerSecond
    val configuredMaxTicksPerSecond2 = shooter2.motorType.achieveableMaxTicksPerSecond
    val rpmConfigurationValid: Boolean get() =
        configuredTicksPerRev1.isFinite() && configuredTicksPerRev1 > 0.0

    var measuredTicksPerSecond1 = 0.0
        private set
    var measuredTicksPerSecond2 = 0.0
        private set
    var measuredRpm1 = 0.0
        private set
    var measuredRpm2 = 0.0
        private set
    var targetRpm = 0.0
        private set
    val targetTicksPerSecond1: Double get() =
        if (rpmConfigurationValid) targetRpm * configuredTicksPerRev1 / 60.0 else 0.0
    val targetTicksPerSecond2: Double get() =
        if (configuredTicksPerRev2.isFinite() && configuredTicksPerRev2 > 0.0)
            targetRpm * configuredTicksPerRev2 / 60.0 else 0.0
    var encoderPosition1 = 0
        private set
    var encoderPosition2 = 0
        private set
    val flywheelPower1: Double get() = desiredPower
    val flywheelPower2: Double get() = desiredPower
    val blockerCommandedPosition: Double get() = desiredBlockerPosition
    var blockerOpen = false
        private set
    var blockerStatus = "Shooter off"
        private set
    var atSpeed = false
        private set
    var feedReady = false
        private set
    var feedReadiness = "Shooter off"
        private set
    var controlMode = ControlMode.VELOCITY_LQR
        private set
    var speedStatus = "Off"
        private set

    private var requestedPower = 0.0
    private var spinupStartedNanos = -1L
    private var manualBlockerOpen = false
    private val lqr = FlywheelLqr()
    private var lastLqrUpdateNanos: Long? = null
    var lqrIntervalMs = 0.0
        private set
    val lqrFrequencyHz: Double get() = if (lqrIntervalMs > 0.0) 1000.0 / lqrIntervalMs else 0.0
    private var lastLqrModel: FlywheelLqrConfig? = null
    var lqrFeedbackGain = 0.0
        private set
    var lqrRequestedVolts = 0.0
        private set
    var lqrModelRead = "Not sampled"
        private set

    companion object {
        // Application reference for preset -> RPM, not the motor's rated 6000 RPM.
        const val MAX_MOTOR_RPM = (3600.0 / 8.0) * 10.0
        const val DEFAULT_SHOOTER_PRESET = 0.72
        private const val NOMINAL_VOLTAGE = 12.0
        private const val POWER_SPINUP_NANOS = 3_000_000_000L
        private const val READY_SPEED_FRACTION = 0.01
        private const val BLOCKER_OPEN = 0.65
        private const val BLOCKER_CLOSED = 0.78
    }

    init {
        // Reverse both motor polarities while preserving logical forward/reverse controls.
        shooter1.direction = DcMotorSimple.Direction.FORWARD
        shooter2.direction = DcMotorSimple.Direction.REVERSE
        shooter1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        shooter2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        // LQR commands voltage through motor power; Hub velocity control stays disabled.
        setRunWithoutEncoder()
        shooter1.power = 0.0
        shooter2.power = 0.0
        updateBlocker()
    }

    fun startVoltagePolling() = batteryVoltage.start()

    fun stopVoltagePolling() = batteryVoltage.close()

    /** Also sync the mode while idle, so a stale manual override cannot bypass velocity control. */
    fun setControlMode(mode: ControlMode) {
        val selected = if (mode == ControlMode.VELOCITY_PID) ControlMode.VELOCITY_LQR else mode
        if (selected != controlMode) {
            resetSpeedHold()
            controlMode = selected
            stopFlywheels()
            updateBlocker()
        }
    }

    /** The selected fraction sets a fixed RPM target immediately, without learning a speed. */
    fun updateShot(power: Double, mode: ControlMode): Boolean {
        setControlMode(mode)
        val clipped = if (power.isFinite()) power.coerceIn(0.0, 1.0) else 0.0
        if (clipped == 0.0) {
            setFlywheelPower(0.0)
            return false
        }
        if (clipped != requestedPower) resetSpeedHold()
        requestedPower = clipped
        readSpeeds()
        val now = nanoTime()
        if (spinupStartedNanos == -1L) spinupStartedNanos = now

        if (mode == ControlMode.MANUAL_POWER) {
            applyNormalizedPower(clipped)
            feedReady = now - spinupStartedNanos >= POWER_SPINUP_NANOS
            feedReadiness = if (feedReady) "Timed feed ready - speed unverified"
                else "Timed spin-up: wait 3 seconds - speed unverified"
            speedStatus = "Voltage-normalized power"
            updateBlocker()
            return false
        }

        targetRpm = clipped * MAX_MOTOR_RPM
        val model = ShooterLqrTuning.snapshot()
        if (model != lastLqrModel) {
            lastLqrModel = model
            lqrModelRead = "kV=${model.kV}, kA=${model.kA}, RPM cost=${model.rpmTolerance}, V cost=${model.voltageTolerance}"
        }
        val previous = lastLqrUpdateNanos
        lqrIntervalMs = if (previous != null) (now-previous).coerceAtLeast(0L)/1_000_000.0 else 0.0
        val dt = if (previous != null && now > previous) (now-previous)/1_000_000_000.0
            else ShooterLqrTuning.NOMINAL_PERIOD_SECONDS
        lastLqrUpdateNanos = now
        // Flywheel 1 remains the measured state; both motors receive the same voltage.
        val outputVolts = timed(LoopStage.SHOOTER_LQR) { lqr.calculate(measuredRpm1, targetRpm, dt, model) }
        if (outputVolts == null) {
            lastLqrUpdateNanos = null
            lqrFeedbackGain = 0.0
            lqrRequestedVolts = 0.0
            atSpeed = false
            feedReady = false
            stopFlywheels()
            speedStatus = "LQR disabled: invalid model, weights, or RPM feedback"
            feedReadiness = "LQR disabled - feeding blocked"
            updateBlocker()
            return false
        }
        lqrFeedbackGain = lqr.feedbackGain
        lqrRequestedVolts = outputVolts.coerceAtLeast(0.0)
        // Existing compensation converts the 12 V fraction to volts / cached battery V.
        applyNormalizedPower(lqrRequestedVolts / NOMINAL_VOLTAGE)

        atSpeed = withinTolerance(measuredRpm1)
        // Latch feeding once ready; release, mode/preset changes, or disabled gains reset it.
        feedReady = feedReady || atSpeed
        speedStatus = if (atSpeed) "LQR holding target RPM" else "LQR reaching target RPM"
        feedReadiness = if (feedReady) "Feeding latched: target speed reached"
            else "Waiting for flywheel 1 RPM"
        updateBlocker()
        return atSpeed
    }

    /** Manual mode only: Velocity control never allows B to open the blocker before measured readiness. */
    fun setManualBlockerOpen(open: Boolean) {
        manualBlockerOpen = open
        updateBlocker()
    }

    fun setFlywheelPower(power: Double) {
        val clipped = if (power.isFinite()) power.coerceIn(-1.0, 0.0) else 0.0
        applyNormalizedPower(clipped)
        resetSpeedHold()
        readSpeeds()
        speedStatus = if (clipped < 0.0) "Reversing" else "Off"
        updateBlocker()
    }

    /** Manual requests and LQR volts/12 express power at 12 V; clamp only after compensation. */
    private fun applyNormalizedPower(power: Double) {
        val voltage = if (phasedIo) frameVoltage else batteryVoltage.voltage
        desiredPower = if (power.isFinite())
            (power * (NOMINAL_VOLTAGE / voltage)).coerceIn(-1.0, 1.0) else 0.0
        if (!phasedIo) timed(LoopStage.SHOOTER_OUTPUT) {
            shooter1.power = desiredPower
            shooter2.power = desiredPower
        }
    }

    private inline fun <T> timed(stage: LoopStage, block: () -> T): T {
        val timing = loopProfiler ?: return block()
        return timing.measure(stage, block)
    }

    private fun setRunWithoutEncoder() {
        if (phasedIo) return
        timed(LoopStage.SHOOTER_MODES) {
            shooter1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            shooter2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    private fun stopFlywheels() {
        applyNormalizedPower(0.0)
    }

    private fun readSpeeds() { if (!phasedIo) sampleSpeeds() }

    private fun sampleSpeeds() {
        timed(LoopStage.SHOOTER_ENCODERS) {
            encoderPosition1 = shooter1.currentPosition
            encoderPosition2 = shooter2.currentPosition
            measuredTicksPerSecond1 = abs(shooter1.velocity)
            measuredTicksPerSecond2 = abs(shooter2.velocity)
            measuredRpm1 = if (rpmConfigurationValid)
                measuredTicksPerSecond1 * 60.0 / configuredTicksPerRev1 else Double.NaN
            measuredRpm2 = if (configuredTicksPerRev2.isFinite() && configuredTicksPerRev2 > 0.0)
                measuredTicksPerSecond2 * 60.0 / configuredTicksPerRev2 else Double.NaN
        }
    }

    private fun withinTolerance(measuredRpm: Double): Boolean =
        targetRpm > 0.0 && measuredRpm.isFinite() &&
            abs(measuredRpm - targetRpm) <= targetRpm * READY_SPEED_FRACTION

    private fun resetSpeedHold() {
        requestedPower = 0.0
        spinupStartedNanos = -1L
        targetRpm = 0.0
        lastLqrUpdateNanos = null
        lqrIntervalMs = 0.0
        lqrFeedbackGain = 0.0
        lqrRequestedVolts = 0.0
        atSpeed = false
        feedReady = false
        feedReadiness = "Shooter off"
    }

    private fun updateBlocker() {
        val manualOpenAllowed = manualBlockerOpen && controlMode == ControlMode.MANUAL_POWER
        blockerOpen = manualOpenAllowed || feedReady
        blockerStatus = when {
            manualOpenAllowed -> "Manual open"
            requestedPower > 0.0 -> feedReadiness
            manualBlockerOpen -> "Manual override disabled in LQR"
            else -> "Shooter off"
        }
        desiredBlockerPosition = if (blockerOpen) BLOCKER_OPEN else BLOCKER_CLOSED
        if (!phasedIo) timed(LoopStage.BLOCKER_OUTPUT) { blocker.position = desiredBlockerPosition }
    }
}
