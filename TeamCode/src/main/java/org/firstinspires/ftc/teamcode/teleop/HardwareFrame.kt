package org.firstinspires.ftc.teamcode.teleop

/** A failed read invalidates the entire frame; stale outputs must never be committed. */
internal class HardwareFrame(private val read: () -> Unit, private val write: () -> Unit) {
    private var ready = false
    fun begin() {
        ready = false
        read()
        ready = true
    }
    fun commit() {
        check(ready) { "No complete hardware snapshot for this frame" }
        ready = false
        write()
    }
}
