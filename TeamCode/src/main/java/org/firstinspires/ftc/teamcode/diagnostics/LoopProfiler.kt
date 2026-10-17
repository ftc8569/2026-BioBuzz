package org.firstinspires.ftc.teamcode.diagnostics

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
