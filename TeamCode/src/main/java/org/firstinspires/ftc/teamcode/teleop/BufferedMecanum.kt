package org.firstinspires.ftc.teamcode.teleop

import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.drivetrain.Drivetrain
import org.firstinspires.ftc.teamcode.pedro.Constants
import kotlin.math.*

/** Pedro 3.0.1 Mecanum math, with hardware writes delegated to the shared output owner. */
internal class BufferedMecanum(private val submit: (DoubleArray, Double) -> Unit) : Drivetrain {
    private var lastPowers = DoubleArray(4)
    private fun wheels(p: DrivePowers) = doubleArrayOf(
        p.forward() - p.strafe() - p.turn(), p.forward() + p.strafe() + p.turn(),
        p.forward() + p.strafe() - p.turn(), p.forward() - p.strafe() + p.turn())

    override fun drive(powers: DrivePowers, manual: Boolean) {
        val raw = wheels(powers)
        val scale = maxOf(1.0, raw.maxOf { abs(it) })
        lastPowers = raw.map { it / scale }.toDoubleArray()
        submit(lastPowers, Constants.drivetrainConfig.powerThreshold.get())
    }
    override fun maxScaling(current: DrivePowers, delta: DrivePowers): Double {
        val a = wheels(current)
        val b = wheels(delta)
        var scale = 1.0
        for (i in a.indices) {
            if (abs(b[i]) < 1e-9) continue
            for (t in doubleArrayOf((1.0-a[i])/b[i], (-1.0-a[i])/b[i])) {
                if (t >= 0.0 && t < scale) scale = t
            }
        }
        return scale.coerceIn(0.0, 1.0)
    }
    override fun stop() = stop(false)
    override fun stop(brake: Boolean) {
        lastPowers = DoubleArray(4)
        submit(lastPowers, 0.0)
    }
    override fun debug(): Map<String, Any> = mapOf("wheelPowers" to lastPowers.toList())
    override fun interpolateVelocity(xRadius: Double, yRadius: Double, theta: Double): Double =
        1.0 / (abs(cos(theta))/xRadius + abs(sin(theta))/yRadius)
}
