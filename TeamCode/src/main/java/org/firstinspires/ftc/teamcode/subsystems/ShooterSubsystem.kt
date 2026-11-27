package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType
import dev.frozenmilk.sinister.loading.Pinned
import dev.nextftc.core.subsystems.Subsystem
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.sqrt
import org.firstinspires.ftc.robotcore.external.navigation.Rotation
import org.firstinspires.ftc.teamcode.teleop.outputWrites

/**
 * Two flywheels and the blocker. Call [update] once per loop: it reads RPM (bulk-cached),
 * runs the LQR, decides whether feeding may start, and writes only outputs that changed.
 */
class ShooterSubsystem(hardwareMap: HardwareMap, private val nanoTime: () -> Long = System::nanoTime) : Subsystem {
    enum class Mode { MANUAL, LQR }

    private val motors = listOf("shooter1", "shooter2").map { hardwareMap.get(DcMotorEx::class.java, it) }
    private val ticksPerRev = motors.map { it.motorType.ticksPerRev }
    private val blocker = hardwareMap.get(Servo::class.java, "blocker")
    private val battery = BatteryVoltage(hardwareMap.getAll(VoltageSensor::class.java))
    private val written = DoubleArray(3) { Double.NaN }
    private var blockerEnabled = false // The blocker never moves before START.
    private var request = 0.0
    private var shotStart = 0L
    private var lastLqr: Long? = null

    /** Latest RPM of each flywheel (NaN if the motor type has no ticks/rev). */
    val rpm = DoubleArray(2)
    var mode = Mode.LQR; private set
    var targetRpm = 0.0; private set
    /** Motor power after battery scaling. */
    var power = 0.0; private set
    /** Latched once flywheel 1 is within 1 % (LQR) or after 3 s (manual); cleared on any change. */
    var feedReady = false; private set
    var blockerOpen = false; private set
    var status = "Off"; private set

    init {
        motors.forEachIndexed { i, m ->
            m.direction = if (i == 0) DcMotorSimple.Direction.FORWARD else DcMotorSimple.Direction.REVERSE
            m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            m.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        forceStop()
    }

    fun startVoltagePolling() = battery.start()
    fun stopVoltagePolling() = battery.close()
    fun startBlockerControl() { blockerEnabled = true; write(force = false) }

    /** [power] > 0 shoots at that preset, < 0 reverses, 0 stops. [manualBlocker] only works in manual mode. */
    fun update(power: Double, mode: Mode, manualBlocker: Boolean = false) {
        val now = nanoTime()
        val wanted = if (power.isFinite()) power.coerceIn(-1.0, 1.0) else 0.0
        if (mode != this.mode || wanted != request) {
            this.mode = mode; request = wanted; shotStart = now; lastLqr = null; feedReady = false
        }
        for (i in 0..1) rpm[i] = if (ticksPerRev[i] > 0.0) abs(motors[i].velocity) * 60.0 / ticksPerRev[i] else Double.NaN
        var volts = 12.0 * wanted
        targetRpm = 0.0
        status = when {
            wanted < 0.0 -> "Reversing"
            wanted == 0.0 -> "Off"
            mode == Mode.MANUAL -> {
                feedReady = now - shotStart >= 3_000_000_000L
                if (feedReady) "Manual: feed ready (speed unverified)" else "Manual: 3 s spin-up"
            }
            else -> {
                targetRpm = wanted * MAX_MOTOR_RPM
                val dt = lastLqr?.let { (now - it) / 1e9 }?.takeIf { it > 0.0 } ?: 0.020
                lastLqr = now
                val lqr = FlywheelLqr.volts(rpm[0], targetRpm, dt)
                volts = lqr?.coerceAtLeast(0.0) ?: 0.0 // Never brake by reversing.
                feedReady = lqr != null && (feedReady || abs(rpm[0] - targetRpm) <= targetRpm * 0.01)
                when {
                    lqr == null -> "LQR disabled: check tuning and encoder"
                    feedReady -> "LQR: at speed, feeding"
                    else -> "LQR: reaching target"
                }
            }
        }
        this.power = (volts / battery.voltage).coerceIn(-1.0, 1.0)
        blockerOpen = feedReady || (mode == Mode.MANUAL && manualBlocker)
        write(force = false)
    }

    /** Zero everything without reading hardware; attempts every output even if one fails. */
    fun forceStop() {
        request = 0.0; targetRpm = 0.0; power = 0.0; feedReady = false; blockerOpen = false; status = "Off"
        write(force = true)
    }

    private fun write(force: Boolean) = outputWrites(force, if (blockerEnabled) 3 else 2) { i ->
        val value = if (i < 2) power else if (blockerOpen) BLOCKER_OPEN else BLOCKER_CLOSED
        if (force || value != written[i]) {
            if (i < 2) motors[i].power = value else blocker.position = value
            written[i] = value
        }
    }

    companion object {
        const val MAX_MOTOR_RPM = 4500.0
        const val DEFAULT_SHOOTER_PRESET = 0.72
        const val BLOCKER_OPEN = 0.65
        const val BLOCKER_CLOSED = 0.78
    }
}

/**
 * One-state velocity LQR. RPM' = a·RPM + b·volts, with a = e^(-kV/kA·dt) and b = (1-a)/kV.
 * Costs q = 1/rpmTol², r = 1/voltTol². The positive Riccati root p gives K = abp/(r + b²p).
 * Output: volts = kV·target + K·(target − rpm), or null when the model or RPM is unusable.
 */
internal object FlywheelLqr {
    fun volts(rpm: Double, targetRpm: Double, dt: Double): Double? {
        val kV = ShooterLqrTuning.kV
        val kA = ShooterLqrTuning.kA
        val rpmTol = ShooterLqrTuning.rpmTolerance
        val voltTol = ShooterLqrTuning.voltageTolerance
        // Comparisons are false for NaN; infinities end up non-finite and fail the last check.
        if (!(kV > 0.0 && kA > 0.0 && rpmTol > 0.0 && voltTol > 0.0 && dt > 0.0 && rpm >= 0.0 && targetRpm >= 0.0)) return null
        val a = exp(-kV / kA * dt)
        val b = (1.0 - a) / kV
        val q = 1.0 / (rpmTol * rpmTol)
        val r = 1.0 / (voltTol * voltTol)
        val b2 = b * b
        val c = r * (1.0 - a * a) - q * b2
        val root = sqrt(c * c + 4.0 * b2 * q * r)
        val p = if (c <= 0.0) (root - c) / (2.0 * b2) else 2.0 * q * r / (c + root) // Stable either way.
        val gain = a * b * p / (r + b2 * p)
        return (kV * targetRpm + gain * (targetRpm - rpm)).takeIf { it.isFinite() && gain >= 0.0 }
    }
}

/** Panels > Configurables > ShooterLqrTuning. Initial model is ESTIMATED, not measured. */
@Configurable
@Pinned
class ShooterLqrTuning {
    @Pinned
    companion object {
        @JvmField @Volatile var kV = 0.00240 // V / RPM
        @JvmField @Volatile var kA = 0.00015 // V / (RPM/s)
        @JvmField @Volatile var rpmTolerance = 10.0
        @JvmField @Volatile var voltageTolerance = 3.0
    }
}

/** Battery voltage read 4 times a second on a background thread, so the loop never waits on it. */
internal class BatteryVoltage(private val sensors: List<VoltageSensor>) : AutoCloseable {
    @Volatile var voltage = 12.0
        private set
    private var executor: ScheduledExecutorService? = null
    private var closed = false

    @Synchronized fun start() {
        if (closed || executor != null || sensors.isEmpty()) return
        executor = Executors.newSingleThreadScheduledExecutor { Thread(it, "BioBuzz battery voltage").apply { isDaemon = true } }
            .apply { scheduleWithFixedDelay(::poll, 0, 250, TimeUnit.MILLISECONDS) }
    }

    // A failed hub must not stop future polls or the other hub's reading.
    private fun poll() {
        voltage = sensors.mapNotNull { runCatching { it.voltage }.getOrNull()?.takeIf { v -> v.isFinite() && v > 0.0 } }
            .minOrNull() ?: 12.0
    }

    @Synchronized override fun close() { closed = true; executor?.shutdownNow(); executor = null }
}

/** Confirmed goBILDA 1:1 gearbox, rather than the SDK's 99.5:1 default. */
@MotorType(ticksPerRev = 28.0, gearing = 1.0, maxRPM = 6000.0, orientation = Rotation.CCW)
@DeviceProperties(xmlTag = "BioBuzzShooter1To1", name = "BioBuzz shooter goBILDA 1:1")
interface BioBuzzShooterMotor
