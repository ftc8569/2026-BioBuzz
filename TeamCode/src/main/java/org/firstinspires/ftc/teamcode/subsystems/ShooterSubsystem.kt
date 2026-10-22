package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType
import dev.frozenmilk.sinister.loading.Pinned
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.expm1
import kotlin.math.hypot
import kotlin.math.roundToInt
import org.firstinspires.ftc.robotcore.external.navigation.Rotation
import org.firstinspires.ftc.teamcode.diagnostics.LoopProfiler
import org.firstinspires.ftc.teamcode.diagnostics.LoopStage
import org.firstinspires.ftc.teamcode.diagnostics.LoopTiming
import org.firstinspires.ftc.teamcode.diagnostics.measureOrRun
import org.firstinspires.ftc.teamcode.teleop.outputWrites
import org.firstinspires.ftc.teamcode.subsystems.ShooterController.ControlMode

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

/** Hold Y for manual shooting; preset adjustments happen once per D-pad press. */
internal class ShooterControls {
    private var presetSteps = (ShooterController.DEFAULT_SHOOTER_PRESET / 0.005).roundToInt()
    val preset: Double get() = presetSteps * 0.005
    var mode = ControlMode.VELOCITY_LQR
        private set
    val velocityTargetRpm: Double?
        get() = if (mode == ControlMode.VELOCITY_LQR)
            preset * ShooterController.MAX_MOTOR_RPM else null
    private val upButton = PressEdge()
    private val downButton = PressEdge()

    fun update(y: Boolean, up: Boolean, down: Boolean) {
        mode = if (y) ControlMode.MANUAL_POWER else ControlMode.VELOCITY_LQR
        val increase = upButton.pressed(up)
        val decrease = downButton.pressed(down)
        val step = if (increase && !down) 1 else if (decrease && !up) -1 else 0
        presetSteps = (presetSteps + step).coerceIn(0, 200)
    }

    fun requestedPower(reverse: Boolean, velocityShoot: Boolean): Double = when {
        reverse -> -0.8
        mode == ControlMode.MANUAL_POWER || velocityShoot -> preset
        else -> 0.0
    }
}

/** Shot decisions use sampled RPM; this controller never accesses hardware. */
class ShooterController(private val nanoTime: () -> Long = System::nanoTime) {
    enum class ControlMode { MANUAL_POWER, VELOCITY_LQR }
    private enum class State { OFF, REVERSE, MANUAL, LQR, DISABLED }
    private var state = State.OFF
    private var requestedPower = 0.0
    private var spinupStarted: Long? = null
    private var manualBlockerOpen = false
    private val lqr = FlywheelLqr()
    private val timing = LoopTiming(nanoTime)
    private var model: FlywheelLqrConfig? = null
    internal var profiler: LoopProfiler? = null
    var controlMode = ControlMode.VELOCITY_LQR
        private set
    var normalizedPower = 0.0
        private set
    var targetRpm = 0.0
        private set
    var atSpeed = false
        private set
    var feedReady = false
        private set
    var lqrRequestedVolts = 0.0
        private set
    var lqrModelRead = "Not sampled"
        private set
    val lqrIntervalMs: Double get() = timing.intervalMs
    val lqrFrequencyHz: Double get() = timing.frequencyHz
    val lqrFeedbackGain: Double get() = lqr.feedbackGain
    private val manualOpenAllowed: Boolean
        get() = manualBlockerOpen && controlMode == ControlMode.MANUAL_POWER
    val blockerOpen: Boolean get() = manualOpenAllowed || feedReady
    val blockerPosition: Double get() = if (blockerOpen) 0.65 else BLOCKER_CLOSED
    val blockerStatus: String get() = when {
        manualOpenAllowed -> "Manual open"
        requestedPower > 0.0 -> feedReadiness
        manualBlockerOpen -> "Manual override disabled in LQR"
        else -> "Shooter off"
    }
    val speedStatus: String get() = when (state) {
        State.OFF -> "Off"
        State.REVERSE -> "Reversing"
        State.MANUAL -> "Voltage-normalized power"
        State.LQR -> if (atSpeed) "LQR holding target RPM" else "LQR reaching target RPM"
        State.DISABLED -> "LQR disabled: invalid model, weights, or RPM feedback"
    }
    val feedReadiness: String get() = when (state) {
        State.MANUAL -> if (feedReady) "Timed feed ready - speed unverified"
            else "Timed spin-up: wait 3 seconds - speed unverified"
        State.LQR -> if (feedReady) "Feeding latched: target speed reached"
            else "Waiting for flywheel 1 RPM"
        State.DISABLED -> "LQR disabled - feeding blocked"
        else -> "Shooter off"
    }

    fun stop() { setFlywheelPower(0.0); manualBlockerOpen = false }
    fun setManualBlockerOpen(open: Boolean) { manualBlockerOpen = open }

    fun setControlMode(mode: ControlMode) {
        if (mode == controlMode) return
        setFlywheelPower(0.0)
        controlMode = mode
    }

    fun setFlywheelPower(power: Double) {
        resetShot()
        normalizedPower = if (power.isFinite()) power.coerceIn(-1.0, 0.0) else 0.0
        state = if (normalizedPower < 0.0) State.REVERSE else State.OFF
    }

    fun updateShot(power: Double, mode: ControlMode, measuredRpm: Double): Boolean {
        setControlMode(mode)
        val clipped = if (power.isFinite()) power.coerceIn(0.0, 1.0) else 0.0
        if (clipped == 0.0) { setFlywheelPower(0.0); return false }
        if (clipped != requestedPower) resetShot()
        requestedPower = clipped
        if (mode == ControlMode.MANUAL_POWER) {
            val now = nanoTime()
            val start = spinupStarted ?: now.also { spinupStarted = it }
            normalizedPower = clipped
            feedReady = now - start >= 3_000_000_000L
            state = State.MANUAL
        } else {
            updateVelocity(measuredRpm)
        }
        return atSpeed
    }

    private fun updateVelocity(rpm: Double) {
        targetRpm = requestedPower * MAX_MOTOR_RPM
        val liveModel = ShooterLqrTuning.snapshot(model)
        if (liveModel !== model) {
            model = liveModel
            lqrModelRead = "kV=${liveModel.kV}, kA=${liveModel.kA}, " +
                "RPM cost=${liveModel.rpmTolerance}, V cost=${liveModel.voltageTolerance}"
        }
        timing.update()
        val dt = (timing.intervalMs / 1000.0).takeIf { it > 0.0 }
            ?: ShooterLqrTuning.NOMINAL_PERIOD_SECONDS
        val volts = profiler.measureOrRun(LoopStage.SHOOTER_LQR) {
            lqr.calculate(rpm, targetRpm, dt, liveModel)
        }
        if (volts == null) {
            resetVelocity()
            normalizedPower = 0.0
            state = State.DISABLED
            return
        }
        lqrRequestedVolts = volts.coerceAtLeast(0.0)
        normalizedPower = lqrRequestedVolts / 12.0
        atSpeed = targetRpm > 0.0 && abs(rpm - targetRpm) <= targetRpm * 0.01
        feedReady = feedReady || atSpeed
        state = State.LQR
    }

    private fun resetVelocity() {
        timing.reset()
        lqr.reset()
        lqrRequestedVolts = 0.0
        atSpeed = false
        feedReady = false
    }

    private fun resetShot() {
        requestedPower = 0.0
        spinupStarted = null
        targetRpm = 0.0
        resetVelocity()
    }

    companion object {
        const val MAX_MOTOR_RPM = 4500.0
        const val DEFAULT_SHOOTER_PRESET = 0.72
        const val BLOCKER_CLOSED = 0.78
    }
}

internal data class FlywheelLqrConfig(
    val kV: Double, // V / RPM
    val kA: Double, // V / (RPM/s)
    val rpmTolerance: Double,
    val voltageTolerance: Double
) {
    val valid = doubleArrayOf(kV, kA, rpmTolerance, voltageTolerance)
        .all { it.isFinite() && it > 0.0 }
}

/** Single measured velocity state: dRPM/dt = -(kV/kA) RPM + volts/kA.
 * No hardware, observer, numerical matrix solver, or accumulated integral error.
 */
internal class FlywheelLqr {
    var feedbackGain = 0.0
        private set

    fun reset() { feedbackGain = 0.0 }

    /** Returns requested volts, or null when the model/sample cannot be used. */
    fun calculate(rpm: Double, targetRpm: Double, dt: Double, config: FlywheelLqrConfig): Double? {
        feedbackGain = 0.0
        if (!rpm.isFinite() || rpm < 0.0 || !targetRpm.isFinite() || targetRpm < 0.0 ||
            !dt.isFinite() || dt <= 0.0 || !config.valid) return null

        // Exact zero-order-hold discretization. expm1 avoids cancellation at short dt.
        val oneMinusA = -expm1(-config.kV / config.kA * dt)
        val a = 1.0 - oneMinusA
        val b = oneMinusA / config.kV
        // Normalize Q/R by the configured RPM/voltage tolerances.
        val bn = b * config.voltageTolerance / config.rpmTolerance
        val b2 = bn * bn
        if (!b2.isFinite() || b2 <= 0.0) return null
        // Stabilizing positive root of the scalar discrete Riccati equation.
        val c = 1.0 - a*a - b2
        val discriminant = hypot(c, 2.0*bn)
        val p = if (c >= 0.0) 2.0/(c+discriminant) else (discriminant-c)/(2.0*b2)
        val gain = (bn*p*a/(1.0+b2*p)) * config.voltageTolerance/config.rpmTolerance
        val volts = config.kV*targetRpm + gain*(targetRpm-rpm)
        if (!p.isFinite() || p <= 0.0 || !gain.isFinite() || gain < 0.0 ||
            !volts.isFinite()) return null
        feedbackGain = gain
        return volts
    }
}

/** Panels > Configurables > ShooterLqrTuning. Initial model is ESTIMATED, not characterized. */
@Configurable
@Pinned
class ShooterLqrTuning {
    @Pinned
    companion object {
        // Preserve the current voltage/RPM model; replace only with characterized values.
        const val DEFAULT_KV = 0.00240
        // Current first-order time constant kA/kV is 0.0625 s.
        const val DEFAULT_KA = 0.00015
        const val DEFAULT_RPM_TOLERANCE = 10.0
        const val DEFAULT_VOLTAGE_TOLERANCE = 3.0
        internal const val NOMINAL_PERIOD_SECONDS = 0.020

        @JvmField @Volatile var kV = DEFAULT_KV
        @JvmField @Volatile var kA = DEFAULT_KA
        // These choose LQR cost weights, not the feed-readiness threshold (still 1%).
        @JvmField @Volatile var rpmTolerance = DEFAULT_RPM_TOLERANCE
        @JvmField @Volatile var voltageTolerance = DEFAULT_VOLTAGE_TOLERANCE

        internal fun snapshot(previous: FlywheelLqrConfig? = null): FlywheelLqrConfig =
            previous?.takeIf {
                it.kV.compareTo(kV) == 0 && it.kA.compareTo(kA) == 0 &&
                    it.rpmTolerance.compareTo(rpmTolerance) == 0 &&
                    it.voltageTolerance.compareTo(voltageTolerance) == 0
            } ?: FlywheelLqrConfig(kV, kA, rpmTolerance, voltageTolerance)
    }
}

/** Hardware reads stay on one worker; control loops only read the volatile snapshot. */
internal class BatteryVoltageMonitor(private val sensors: List<VoltageSensor>) : AutoCloseable {
    @Volatile var voltage = 12.0
        private set
    @Volatile internal var profiler: LoopProfiler? = null
    private var executor: ScheduledExecutorService? = null
    private var closed = false

    @Synchronized fun start() {
        if (closed || executor != null || sensors.isEmpty()) return
        executor = Executors.newSingleThreadScheduledExecutor { task ->
            Thread(task, "BioBuzz battery voltage").apply { isDaemon = true }
        }.also {
            it.scheduleWithFixedDelay({ pollOnce() }, 0, 250, TimeUnit.MILLISECONDS)
        }
    }

    internal fun pollOnce() = profiler.measureOrRun(LoopStage.VOLTAGE_POLL) {
        var minimum = Double.POSITIVE_INFINITY
        for (sensor in sensors) {
            if (Thread.currentThread().isInterrupted) return@measureOrRun
            val sample = try {
                sensor.voltage
            } catch (_: RuntimeException) {
                // A failed hub must not prevent future polls or reading the other hub.
                continue
            }
            if (sample.isFinite() && sample > 0.0) minimum = minOf(minimum, sample)
        }
        voltage = if (minimum.isFinite()) minimum else 12.0
    }

    @Synchronized override fun close() {
        closed = true
        executor?.shutdownNow()
        executor = null
    }
}

/** Cached motor feedback and its configured conversion; telemetry performs no hardware reads. */
class FlywheelFeedback(val ticksPerRev: Double, val maxTicksPerSecond: Double) {
    var position = 0
        private set
    var ticksPerSecond = 0.0
        private set
    val valid = ticksPerRev.isFinite() && ticksPerRev > 0.0
    val rpm: Double get() = if (valid) ticksPerSecond * 60.0 / ticksPerRev else Double.NaN

    fun sample(position: Int, velocity: Double) {
        this.position = position
        ticksPerSecond = abs(velocity)
    }

    fun targetTicksPerSecond(rpm: Double): Double = if (valid) rpm * ticksPerRev / 60.0 else 0.0
}

/** Confirmed goBILDA 1:1 gearbox, rather than the SDK's 99.5:1 default. */
@MotorType(ticksPerRev = 28.0, gearing = 1.0, maxRPM = 6000.0, orientation = Rotation.CCW)
@DeviceProperties(xmlTag = "BioBuzzShooter1To1", name = "BioBuzz shooter goBILDA 1:1")
interface BioBuzzShooterMotor
