package org.firstinspires.ftc.teamcode.Subsytem

import com.qualcomm.robotcore.hardware.VoltageSensor
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit

/** Hardware reads stay on one worker; control loops only read the volatile snapshot. */
internal class BatteryVoltageMonitor(private val sensors: List<VoltageSensor>) : AutoCloseable {
    @Volatile var voltage = 12.0
        private set
    private var executor: ScheduledExecutorService? = null
    private var closed = false

    @Synchronized fun start() {
        if (closed || executor != null) return
        executor = Executors.newSingleThreadScheduledExecutor { task ->
            Thread(task, "BioBuzz battery voltage").apply { isDaemon = true }
        }.also {
            it.scheduleWithFixedDelay({ pollOnce() }, 0, 250, TimeUnit.MILLISECONDS)
        }
    }

    internal fun pollOnce() {
        var minimum = Double.POSITIVE_INFINITY
        for (sensor in sensors) {
            if (Thread.currentThread().isInterrupted) return
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
