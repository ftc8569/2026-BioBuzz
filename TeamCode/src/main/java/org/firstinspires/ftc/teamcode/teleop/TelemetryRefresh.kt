package org.firstinspires.ftc.teamcode.teleop

/** Match the 50 ms display interval without formatting or publishing on every PID tick. */
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
