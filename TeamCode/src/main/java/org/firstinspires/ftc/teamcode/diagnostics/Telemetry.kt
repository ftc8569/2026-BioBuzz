package org.firstinspires.ftc.teamcode.diagnostics

import com.bylazar.telemetry.JoinedTelemetry
import com.qualcomm.robotcore.util.RobotLog
import java.util.Locale
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.control.ShooterController
import org.firstinspires.ftc.teamcode.control.ShooterControls
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem

internal enum class LoopStage(val label: String) {
    LOOP_INTERVAL("Loop interval"),
    LOOP_WORK("Loop work total"),
    READ_INPUTS("Read inputs phase"),
    BULK_READ("Hub bulk read transactions"),
    COMPUTE("Control compute phase"),
    WRITE_OUTPUTS("Write outputs phase"),
    DRIVE_TOTAL("Drive total"),
    DRIVE_ENCODERS("Drive encoder reads"),
    PINPOINT("Pinpoint location read"),
    LOCATION_RESET("Pinpoint initial pose write"),
    HIVE_SAVE("Hive save (cached pose)"),
    DRIVE_HOLD("Pedro hold update"),
    DRIVE_OUTPUT("Drive motor output commit"),
    DRIVE_TELEMETRY("Drive telemetry build"),
    MECHANISMS_TOTAL("Mechanisms total"),
    SHOOTER_CONTROL("Shooter control total"),
    SHOOTER_ENCODERS("Shooter encoder and velocity reads"),
    SHOOTER_LQR("Shooter LQR calculation"),
    SHOOTER_MODES("Shooter motor mode writes"),
    SHOOTER_OUTPUT("Shooter motor output commit"),
    BLOCKER_OUTPUT("Blocker servo output commit"),
    INTAKE_OUTPUT("Intake motor writes"),
    TELEMETRY_BUILD("Main telemetry build and getters"),
    TELEMETRY_UPDATE("Telemetry update both outputs"),
    DS_TELEMETRY_UPDATE("Driver Station telemetry update"),
    PANELS_TELEMETRY_UPDATE("Panels telemetry update"),
    TIMING_REPORT("Timing report overhead"),
    VOLTAGE_POLL("Async voltage sensor poll (background)")
}

internal data class TimingSample(
    val stage: LoopStage,
    val calls: Long,
    val averageMs: Double,
    val maxMs: Double
)

/** Locks protect counters only, never hardware calls or report formatting. */
internal class LoopProfiler(@PublishedApi internal val nanoTime: () -> Long = System::nanoTime) {
    private val counts = LongArray(LoopStage.entries.size)
    private val totals = LongArray(counts.size)
    private val maxima = LongArray(counts.size)
    private var lastReportNanos = nanoTime()
    private var previousLoopStart: Long? = null
    private var loopStart: Long? = null

    inline fun <T> measure(stage: LoopStage, block: () -> T): T {
        val start = nanoTime()
        try {
            return block()
        } finally {
            record(stage, nanoTime() - start)
        }
    }

    @PublishedApi
    @Synchronized internal fun record(stage: LoopStage, elapsedNanos: Long) {
        val i = stage.ordinal
        val duration = elapsedNanos.coerceAtLeast(0L)
        counts[i]++
        totals[i] += duration
        maxima[i] = maxOf(maxima[i], duration)
    }

    fun beginLoop() {
        val now = nanoTime()
        previousLoopStart?.let { record(LoopStage.LOOP_INTERVAL, now - it) }
        previousLoopStart = now
        loopStart = now
    }

    fun endLoop() {
        loopStart?.let { record(LoopStage.LOOP_WORK, nanoTime() - it) }
        loopStart = null
    }

    @Synchronized fun reset() {
        counts.fill(0)
        totals.fill(0)
        maxima.fill(0)
        previousLoopStart = null
        loopStart = null
        lastReportNanos = nanoTime()
    }

    /** Drain once per second. Uncalled stages are absent, not reported as zero-cost calls. */
    @Synchronized fun snapshotIfDue(): List<TimingSample>? {
        val now = nanoTime()
        if (now - lastReportNanos < 1_000_000_000L) return null
        val samples = LoopStage.entries.mapNotNull { stage ->
            val i = stage.ordinal
            if (counts[i] == 0L) null else TimingSample(
                stage, counts[i], totals[i].toDouble() / counts[i] / 1_000_000.0,
                maxima[i] / 1_000_000.0
            )
        }
        counts.fill(0)
        totals.fill(0)
        maxima.fill(0)
        lastReportNanos = now
        return samples
    }
}

/** Interval between the same point in consecutive TeleOp loops, including all loop work. */
internal class LoopTiming(private val nanoTime: () -> Long = System::nanoTime) {
    private var previousNanos: Long? = null
    var intervalMs = 0.0
        private set
    val frequencyHz: Double get() = if (intervalMs > 0.0) 1000.0 / intervalMs else 0.0

    fun reset() {
        previousNanos = null
        intervalMs = 0.0
    }

    fun update() {
        val now = nanoTime()
        intervalMs = previousNanos?.let { (now - it).coerceAtLeast(0L) / 1_000_000.0 } ?: 0.0
        previousNanos = now
    }
}

/** Match the 50 ms display interval without formatting or publishing on every control tick. */
class TelemetryRefresh @JvmOverloads constructor(
    private val nanoTime: () -> Long = System::nanoTime
) {
    private var lastRefresh: Long? = null

    fun reset() { lastRefresh = null }

    fun isDue(): Boolean {
        val now = nanoTime()
        val previous = lastRefresh
        if (previous != null && now - previous < 50_000_000L) return false
        lastRefresh = now
        return true
    }
}

/** One display clock and one formatting pass for both Driver Station and Panels. */
internal class RobotTelemetry(
    driverStation: Telemetry,
    panels: Telemetry,
    private val profiler: LoopProfiler,
    nanoTime: () -> Long = System::nanoTime
) {
    private val refresh = TelemetryRefresh(nanoTime)
    private val joined = JoinedTelemetry(
        timed(driverStation, LoopStage.DS_TELEMETRY_UPDATE),
        timed(panels, LoopStage.PANELS_TELEMETRY_UPDATE)
    ).apply { msTransmissionInterval = 50 }
    private var timingRows: List<Pair<String, String>> = emptyList()

    fun addData(caption: String, value: Any?) { joined.addData(caption, value) }
    fun addData(caption: String, format: String, vararg values: Any?) {
        joined.addData(caption, String.format(Locale.US, format, *values))
    }
    fun publish(build: RobotTelemetry.() -> Unit) {
        if (!refresh.isDue()) return
        profiler.measure(LoopStage.TELEMETRY_BUILD) { build() }
        profiler.measure(LoopStage.TELEMETRY_UPDATE) { joined.update() }
    }
    fun reset() { refresh.reset(); timingRows = emptyList() }
    fun clear() { refresh.reset(); joined.clearAll() }

    fun reportTimings() = profiler.measure(LoopStage.TIMING_REPORT) {
        profiler.snapshotIfDue()?.let { samples ->
            val byStage = samples.associateBy { it.stage }
            timingRows = LoopStage.entries.map { stage ->
                val sample = byStage[stage]
                "Timing: ${stage.label}" to if (sample == null) "not called in this window" else
                    String.format(Locale.US, "avg=%.3f ms max=%.3f ms n=%d",
                        sample.averageMs, sample.maxMs, sample.calls)
            }
            val report = timingRows.joinToString("; ") { "${it.first}: ${it.second}" }
            RobotLog.ii("BioBuzzLoop", report)
        }
        for ((label, value) in timingRows) joined.addData(label, value)
    }

    private fun timed(delegate: Telemetry, stage: LoopStage): Telemetry =
        object : Telemetry by delegate {
            override fun update(): Boolean = profiler.measure(stage) { delegate.update() }
        }
}

internal inline fun <T> LoopProfiler?.measureOrRun(stage: LoopStage, block: () -> T): T =
    if (this == null) block() else measure(stage, block)

/** Display cached shooter state without reading hardware or changing controller state. */
internal fun RobotTelemetry.reportShooter(
    shooter: ShooterSubsystem,
    controls: ShooterControls,
    requestedPower: Double,
    manualBlocker: Boolean,
    up: Boolean,
    down: Boolean
) {
    val shot = shooter.control
    val (first, second) = shooter.wheels
    addData("LQR interval (ms)", "%.2f", shot.lqrIntervalMs)
    addData("LQR frequency (Hz)", "%.1f", shot.lqrFrequencyHz)
    shooter.wheels.forEachIndexed { i, wheel ->
        addData("Shooter flywheel ${i + 1} RPM", "%.0f", wheel.rpm)
    }
    val selectedVelocityTarget = controls.velocityTargetRpm
    if (selectedVelocityTarget != null) {
        addData("LQR target RPM", "%.0f", selectedVelocityTarget)
    } else {
        addData("LQR target RPM", "Manual mode: no RPM target")
    }
    addData("LQR shooting active", selectedVelocityTarget != null && requestedPower > 0.0)
    addData("LQR commanded RPM", "%.0f", shot.targetRpm)
    addData("LQR model and cost tolerances", shot.lqrModelRead)
    addData("LQR feedback gain (V/RPM)", shot.lqrFeedbackGain)
    addData("LQR requested voltage (V)", shot.lqrRequestedVolts)
    addData("LQR feedback", "Flywheel 1 RPM (shared motor power)")
    addData("Shooter preset (G1/G2 up/down)", "%.3f", controls.preset)
    addData("Shooter mode (hold G1 Y for manual)", controls.mode)
    addData("Shooter speed state", shot.speedStatus)
    addData("D-pad seen (up/down)", "%s / %s", up, down)
    addData("Shooter requested fraction", "%.2f", requestedPower)
    addData("Shooter motor power 1 / 2", "%.2f / %.2f",
        shooter.flywheelPower, shooter.flywheelPower)
    addData("Manual blocker (G1/G2 B)", manualBlocker)
    addData("Blocker open", shot.blockerOpen)
    addData("Flywheel 1 within 1 percent of target", shot.atSpeed)
    addData("Automatic feed ready", shot.feedReady)
    addData("Feed readiness", shot.feedReadiness)
    addData("Blocker status", shot.blockerStatus)
    addData("Blocker commanded position", "%.2f", shot.blockerPosition)
    addData("Shooter preset reference RPM", "%.0f", ShooterController.MAX_MOTOR_RPM)
    addData("RPM configuration valid", first.valid)
    addData("Raw ticks/s 1 / 2", "%.0f / %.0f",
        first.ticksPerSecond, second.ticksPerSecond)
    addData("Encoder position ticks 1 / 2", "%d / %d",
        first.position, second.position)
    addData("LQR target ticks/s 1 / 2", "%.0f / %.0f",
        first.targetTicksPerSecond(shot.targetRpm), second.targetTicksPerSecond(shot.targetRpm))
    addData("Motor config ticks/rev 1 / 2", "%.0f / %.0f",
        first.ticksPerRev, second.ticksPerRev)
    addData("Motor config max ticks/s 1 / 2", "%.0f / %.0f",
        first.maxTicksPerSecond, second.maxTicksPerSecond)
}
