package org.firstinspires.ftc.teamcode.teleop

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
