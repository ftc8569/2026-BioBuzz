package org.firstinspires.ftc.teamcode.teleop

/** Allow the blocker 100 ms to open before applying forward intake power. */
internal class BlockerFeedDelay(private val nanoTime: () -> Long = System::nanoTime) {
    private var openedAt: Long? = null

    fun apply(power: Double, blockerOpen: Boolean): Double {
        if (!blockerOpen) {
            openedAt = null
            return power
        }
        val now = nanoTime()
        val start = openedAt ?: now.also { openedAt = it }
        return if (power > 0.0 && now - start < 100_000_000L) 0.0 else power
    }
}
