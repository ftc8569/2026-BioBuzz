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
import kotlin.math.exp
import kotlin.math.sqrt
import org.firstinspires.ftc.robotcore.external.navigation.Rotation
import org.firstinspires.ftc.teamcode.subsystems.ShooterController.ControlMode
import org.firstinspires.ftc.teamcode.teleop.LoopTiming
import org.firstinspires.ftc.teamcode.teleop.outputWrites

/**
 * Two flywheels and the blocker. TeleOp uses phased I/O (beginFrame → update → commitOutputs
 * once per loop); autos use immediate I/O, where each call reads and writes on its own.
 * The blocker is never written before [startBlockerControl] (START), as on main.
 */
class ShooterSubsystem(hardwareMap: HardwareMap, nanoTime: () -> Long = System::nanoTime) {
    val control = ShooterController(nanoTime)
    private val motors = listOf("shooter1", "shooter2").map { hardwareMap.get(DcMotorEx::class.java, it) }
    private val blocker = hardwareMap.get(Servo::class.java, "blocker")
    private val ticksPerRev = motors.map { it.motorType.ticksPerRev }
    /** Latest sampled RPM of each flywheel (NaN if the motor type has no valid ticks/rev). */
    val rpm = DoubleArray(2)
    private val battery = BatteryVoltage(hardwareMap.getAll(VoltageSensor::class.java))
    private var phasedIo = false
    private var blockerStarted = false
    private var frameVoltage = 12.0
    private val committed = DoubleArray(3) { Double.NaN }
    /** Motor power after scaling the requested 12 V-normalized power by battery voltage. */
    val flywheelPower: Double get() = (control.normalizedPower * 12.0 / frameVoltage).coerceIn(-1.0, 1.0)

    init {
        motors.forEachIndexed { i, motor ->
            motor.direction = if (i == 0) DcMotorSimple.Direction.FORWARD else DcMotorSimple.Direction.REVERSE
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        forceStop()
    }

    fun startVoltagePolling() = battery.start()
    fun stopVoltagePolling() = battery.close()
    internal fun enablePhasedIo() { phasedIo = true; forceStop() }

    fun startBlockerControl() {
        if (blockerStarted) return
        blockerStarted = true
        control.setManualBlockerOpen(false)
        commitOutputs()
    }

    internal fun beginFrame() {
        frameVoltage = battery.voltage
        for (i in 0..1) rpm[i] = if (ticksPerRev[i] > 0.0) abs(motors[i].velocity) * 60.0 / ticksPerRev[i] else Double.NaN
    }

    /** Writes only changed outputs; [force] (shutdown) attempts every output even after a failure. */
    internal fun commitOutputs(force: Boolean = false) {
        battery.active = control.normalizedPower != 0.0
        outputWrites(force, 3) { i ->
            if (i == 2 && !blockerStarted) return@outputWrites
            val value = if (i < 2) flywheelPower else control.blockerPosition
            if (force || value != committed[i]) {
                if (i < 2) motors[i].power = value else blocker.position = value
                committed[i] = value
            }
        }
    }

    fun forceStop() { control.stop(); commitOutputs(force = true) }

    fun updateShot(power: Double, mode: ControlMode): Boolean {
        if (!phasedIo) beginFrame()
        return control.updateShot(power, mode, rpm[0]).also { flush() }
    }

    fun setFlywheelPower(power: Double) {
        // Stopping needs no RPM read, so zero is sent even if an encoder is unavailable.
        if (!phasedIo && power.isFinite() && power != 0.0) beginFrame()
        control.setFlywheelPower(power)
        flush()
    }

    fun setControlMode(mode: ControlMode) { control.setControlMode(mode); flush() }
    fun setManualBlockerOpen(open: Boolean) { control.setManualBlockerOpen(open); flush() }

    private fun flush() { if (!phasedIo) { frameVoltage = battery.voltage; commitOutputs() } }
}

/** Shot decisions from sampled RPM. Never touches hardware. */
class ShooterController(private val nanoTime: () -> Long = System::nanoTime) {
    enum class ControlMode { MANUAL_POWER, VELOCITY_LQR }
    private enum class State { OFF, REVERSE, MANUAL, LQR, DISABLED }
    private var state = State.OFF
    private var requestedPower = 0.0
    private var spinupStarted: Long? = null
    private var manualBlockerOpen = false
    private val timing = LoopTiming(nanoTime)
    var controlMode = ControlMode.VELOCITY_LQR
        private set
    /** Requested power as a fraction of 12 V. */
    var normalizedPower = 0.0
        private set
    var targetRpm = 0.0
        private set
    var atSpeed = false
        private set
    /** Latched once flywheel 1 is within 1 % (LQR) or after 3 s (manual); cleared on release. */
    var feedReady = false
        private set
    private val manualOpen get() = manualBlockerOpen && controlMode == ControlMode.MANUAL_POWER
    val blockerOpen: Boolean get() = manualOpen || feedReady
    val blockerPosition: Double get() = if (blockerOpen) BLOCKER_OPEN else BLOCKER_CLOSED

    val blockerStatus: String get() = when {
        manualOpen -> "Manual open"
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
        State.MANUAL -> if (feedReady) "Timed feed ready - speed unverified" else "Timed spin-up: wait 3 seconds - speed unverified"
        State.LQR -> if (feedReady) "Feeding latched: target speed reached" else "Waiting for flywheel 1 RPM"
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

    /** Zero or reverse only; forward shooting goes through [updateShot]. */
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
            return atSpeed
        }
        targetRpm = clipped * MAX_MOTOR_RPM
        timing.update()
        val dt = if (timing.intervalMs > 0.0) timing.intervalMs / 1000.0 else 0.020 // first update
        val volts = FlywheelLqr.volts(measuredRpm, targetRpm, dt)
        if (volts == null) {
            resetShot(keepTarget = true)
            normalizedPower = 0.0
            state = State.DISABLED
            return false
        }
        normalizedPower = volts.coerceAtLeast(0.0) / 12.0 // Never brake by reversing.
        atSpeed = abs(measuredRpm - targetRpm) <= targetRpm * 0.01
        feedReady = feedReady || atSpeed
        state = State.LQR
        return atSpeed
    }

    private fun resetShot(keepTarget: Boolean = false) {
        if (!keepTarget) { requestedPower = 0.0; spinupStarted = null; targetRpm = 0.0 }
        timing.reset()
        atSpeed = false
        feedReady = false
    }

    companion object {
        const val MAX_MOTOR_RPM = 4500.0
        const val DEFAULT_SHOOTER_PRESET = 0.72
        const val BLOCKER_OPEN = 0.65
        const val BLOCKER_CLOSED = 0.78
    }
}

/**
 * One-state velocity LQR. Model: dRPM/dt = -(kV/kA)·RPM + volts/kA, discretized over dt as
 * RPM' = a·RPM + b·volts. Costs q = 1/rpmTolerance², r = 1/voltageTolerance². The scalar
 * discrete Riccati equation b²p² + (r(1−a²) − qb²)p − qr = 0 has one positive root p, giving
 * K = abp / (r + b²p). Output: volts = kV·target + K·(target − rpm). No integral term.
 */
internal object FlywheelLqr {
    /** Requested volts, or null when the model or the RPM sample cannot be used. */
    fun volts(rpm: Double, targetRpm: Double, dt: Double): Double? {
        val kV = ShooterLqrTuning.kV
        val kA = ShooterLqrTuning.kA
        val rpmTol = ShooterLqrTuning.rpmTolerance
        val voltTol = ShooterLqrTuning.voltageTolerance
        if (!(rpm >= 0.0 && targetRpm >= 0.0 && dt > 0.0 && kV > 0.0 && kA > 0.0 &&
                rpmTol > 0.0 && voltTol > 0.0) ||
            !(rpm + targetRpm + dt + kV + kA + rpmTol + voltTol).isFinite()) return null
        val a = exp(-kV / kA * dt)
        val b = (1.0 - a) / kV
        val q = 1.0 / (rpmTol * rpmTol)
        val r = 1.0 / (voltTol * voltTol)
        val b2 = b * b
        val c = r * (1.0 - a * a) - q * b2
        val root = sqrt(c * c + 4.0 * b2 * q * r)
        // Same root, written two ways to avoid subtracting nearly equal numbers.
        val p = if (c <= 0.0) (root - c) / (2.0 * b2) else 2.0 * q * r / (c + root)
        val gain = a * b * p / (r + b2 * p)
        val volts = kV * targetRpm + gain * (targetRpm - rpm)
        return volts.takeIf { it.isFinite() && gain.isFinite() && gain >= 0.0 }
    }
}

/** Panels > Configurables > ShooterLqrTuning. Initial model is ESTIMATED, not measured. */
@Configurable
@Pinned
class ShooterLqrTuning {
    @Pinned
    companion object {
        @JvmField @Volatile var kV = 0.00240 // V / RPM
        @JvmField @Volatile var kA = 0.00015 // V / (RPM/s); time constant kA/kV = 0.0625 s
        // LQR cost scales only; the feed-readiness band stays 1 %.
        @JvmField @Volatile var rpmTolerance = 10.0
        @JvmField @Volatile var voltageTolerance = 3.0
    }
}

/** Background battery reads (4 Hz while the flywheels are powered, 1 Hz idle), off the loop. */
internal class BatteryVoltage(private val sensors: List<VoltageSensor>) : AutoCloseable {
    @Volatile var voltage = 12.0
        private set
    @Volatile var active = false
    private var idleTicks = 0
    private var executor: ScheduledExecutorService? = null
    private var closed = false

    @Synchronized fun start() {
        if (closed || executor != null || sensors.isEmpty()) return
        executor = Executors.newSingleThreadScheduledExecutor { Thread(it, "BioBuzz battery voltage").apply { isDaemon = true } }
            .apply { scheduleWithFixedDelay(::poll, 0, 250, TimeUnit.MILLISECONDS) }
    }

    private fun poll() {
        if (!active && idleTicks++ % 4 != 0) return
        var minimum = Double.POSITIVE_INFINITY
        for (sensor in sensors) {
            // A failed hub must not stop future polls or the other hub's reading.
            val v = try { sensor.voltage } catch (_: RuntimeException) { continue }
            if (v.isFinite() && v > 0.0) minimum = minOf(minimum, v)
        }
        voltage = if (minimum.isFinite()) minimum else 12.0
    }

    @Synchronized override fun close() {
        closed = true
        executor?.shutdownNow()
        executor = null
    }
}

/** Confirmed goBILDA 1:1 gearbox, rather than the SDK's 99.5:1 default. */
@MotorType(ticksPerRev = 28.0, gearing = 1.0, maxRPM = 6000.0, orientation = Rotation.CCW)
@DeviceProperties(xmlTag = "BioBuzzShooter1To1", name = "BioBuzz shooter goBILDA 1:1")
interface BioBuzzShooterMotor
