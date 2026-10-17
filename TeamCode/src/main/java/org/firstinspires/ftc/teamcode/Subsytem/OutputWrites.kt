package org.firstinspires.ftc.teamcode.Subsytem

/** Normal commits fail fast; forced shutdown attempts every device before rethrowing. */
internal fun outputWrites(force: Boolean, count: Int, write: (Int) -> Unit) {
    var failure: Exception? = null
    for (index in 0 until count) {
        try { write(index) } catch (t: Exception) {
            if (!force) throw t
            if (failure == null) failure = t else failure.addSuppressed(t)
        }
    }
    failure?.let { throw it }
}
