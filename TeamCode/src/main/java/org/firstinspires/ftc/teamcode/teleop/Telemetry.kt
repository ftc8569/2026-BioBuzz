package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ShooterController
import com.bylazar.telemetry.JoinedTelemetry
import java.util.Locale
import org.firstinspires.ftc.robotcore.external.Telemetry

// Shared telemetry: refresh clock, loop timing and the shooter rows. Diagnostics are in Diagnostics.kt.

/**
 * Driver Station / Panels refresh period. People cannot read numbers faster than this, and
 * every publish costs loop time (string formatting plus a network send), so control loops
 * run many times between refreshes. The SDK's own default is 250 ms.
 */
const val TELEMETRY_INTERVAL_MS = 100L

/** Interval between the same point in consecutive loops, including all loop work. */
internal class LoopTiming(private val nanoTime: () -> Long = System::nanoTime) {
    private var previousNanos: Long? = null
    var intervalMs = 0.0
        private set
    /** Smoothed loop rate, so the telemetry row is readable at 10 refreshes per second. */
    var averageHz = 0.0
        private set

    fun reset() {
        previousNanos = null
        intervalMs = 0.0
        averageHz = 0.0
    }

    fun update() {
        val now = nanoTime()
        intervalMs = previousNanos?.let { (now - it).coerceAtLeast(0L) / 1_000_000.0 } ?: 0.0
        previousNanos = now
        if (intervalMs > 0.0) {
            val hz = 1000.0 / intervalMs
            averageHz = if (averageHz == 0.0) hz else averageHz + 0.05 * (hz - averageHz)
        }
    }
}

/** Publish at most once per [TELEMETRY_INTERVAL_MS]; skipped loops do no formatting. */
class TelemetryRefresh @JvmOverloads constructor(
    private val nanoTime: () -> Long = System::nanoTime
) {
    private var lastRefresh: Long? = null

    fun reset() { lastRefresh = null }

    fun isDue(): Boolean {
        val now = nanoTime()
        val previous = lastRefresh
        if (previous != null && now - previous < TELEMETRY_INTERVAL_MS * 1_000_000L) return false
        lastRefresh = now
        return true
    }
}

/** One display clock and one formatting pass for both Driver Station and Panels. */
internal class RobotTelemetry(
    driverStation: Telemetry,
    panels: Telemetry,
    nanoTime: () -> Long = System::nanoTime
) {
    private val refresh = TelemetryRefresh(nanoTime)
    private val joined = JoinedTelemetry(driverStation, panels).apply { msTransmissionInterval = TELEMETRY_INTERVAL_MS.toInt() }

    fun addData(caption: String, value: Any?) { joined.addData(caption, value) }
    fun addData(caption: String, format: String, vararg values: Any?) {
        // Any selects the value overload, avoiding a second formatting pass per destination.
        val text: Any = if (values.isEmpty()) format else String.format(Locale.US, format, *values)
        joined.addData(caption, text)
    }
    fun publish(build: RobotTelemetry.() -> Unit) {
        if (!refresh.isDue()) return
        build()
        joined.update()
    }
    fun reset() { refresh.reset() }
    fun clear() { refresh.reset(); joined.clearAll() }
}

/** Display cached shooter state without reading hardware or changing controller state. */
internal fun RobotTelemetry.reportShooter(shooter: ShooterSubsystem, controls: ShooterControls) {
    val shot = shooter.control
    addData("Mode", if (controls.mode == ShooterController.ControlMode.MANUAL_POWER) "Manual" else "LQR")
    addData("Preset", "%.3f", controls.preset)
    val target = controls.velocityTargetRpm
    if (target == null) addData("Target RPM", "Manual power")
    else addData("Target RPM", "%.0f", target)
    addData("RPM 1 / 2", "%.0f / %.0f", shooter.wheels[0].rpm, shooter.wheels[1].rpm)
    addData("Motor power", "%.2f", shooter.flywheelPower)
    addData("Feed", shot.feedReadiness)
}
