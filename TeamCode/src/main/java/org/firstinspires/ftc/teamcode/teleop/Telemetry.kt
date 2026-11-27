package org.firstinspires.ftc.teamcode.teleop

import com.bylazar.telemetry.JoinedTelemetry
import java.util.Locale
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem

/** Telemetry refresh period. Loops in between skip all telemetry work (the SDK default is 250 ms). */
const val TELEMETRY_INTERVAL_MS = 100L

/** Time between loops and a smoothed loop rate for the "Loop" row. */
internal class LoopTiming(private val nanoTime: () -> Long = System::nanoTime) {
    private var last: Long? = null
    var intervalMs = 0.0; private set
    var averageHz = 0.0; private set

    fun reset() { last = null; intervalMs = 0.0; averageHz = 0.0 }
    fun update() {
        val now = nanoTime()
        last?.let { if (now > it) intervalMs = (now - it) / 1e6 }
        last = now
        if (intervalMs > 0.0) averageHz += if (averageHz == 0.0) 1000.0 / intervalMs else 0.05 * (1000.0 / intervalMs - averageHz)
    }
}

/** True at most once per [TELEMETRY_INTERVAL_MS]. */
class TelemetryRefresh(private val nanoTime: () -> Long = System::nanoTime) {
    private var last: Long? = null
    fun reset() { last = null }
    fun isDue(): Boolean {
        val now = nanoTime()
        if (last?.let { now - it < TELEMETRY_INTERVAL_MS * 1_000_000L } == true) return false
        last = now
        return true
    }
}

/** Driver Station and Panels together, formatted once and published at most every 100 ms. */
internal class RobotTelemetry(driverStation: Telemetry, panels: Telemetry, nanoTime: () -> Long = System::nanoTime) {
    private val refresh = TelemetryRefresh(nanoTime)
    private val out = JoinedTelemetry(driverStation, panels).apply { msTransmissionInterval = TELEMETRY_INTERVAL_MS.toInt() }

    fun addData(caption: String, value: Any?) { out.addData(caption, value) }
    fun addData(caption: String, format: String, vararg values: Any?) {
        out.addData(caption, String.format(Locale.US, format, *values) as Any)
    }
    fun publish(build: RobotTelemetry.() -> Unit) { if (refresh.isDue()) { build(); out.update() } }
    fun reset() = refresh.reset()
    fun clear() { refresh.reset(); out.clearAll() }
}

/** Shooter rows from values already read this loop (no hardware access). */
internal fun RobotTelemetry.reportShooter(shooter: ShooterSubsystem, controls: ShooterControls) {
    val lqr = controls.mode == ShooterSubsystem.Mode.LQR
    addData("Preset / target RPM", if (lqr) "%.3f / %.0f" else "%.3f / manual power",
        controls.preset, controls.preset * ShooterSubsystem.MAX_MOTOR_RPM)
    addData("RPM 1 / 2", "%.0f / %.0f", shooter.rpm[0], shooter.rpm[1])
    addData("Motor power", "%.2f", shooter.power)
    addData("Shooter", shooter.status)
}
