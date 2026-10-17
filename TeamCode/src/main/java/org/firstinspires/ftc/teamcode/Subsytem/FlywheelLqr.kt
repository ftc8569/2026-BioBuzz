package org.firstinspires.ftc.teamcode.Subsytem

import kotlin.math.expm1
import kotlin.math.hypot

internal data class FlywheelLqrConfig(
    val kV: Double, // V / RPM
    val kA: Double, // V / (RPM/s)
    val rpmTolerance: Double,
    val voltageTolerance: Double
)

/** Single measured velocity state: dRPM/dt = -(kV/kA) RPM + volts/kA.
 * No hardware, observer, numerical matrix solver, or accumulated integral error.
 */
internal class FlywheelLqr {
    var feedbackGain = 0.0
        private set

    /** Returns requested volts, or null when the model/sample cannot be used. */
    fun calculate(rpm: Double, targetRpm: Double, dt: Double, config: FlywheelLqrConfig): Double? {
        feedbackGain = 0.0
        if (!rpm.isFinite() || rpm < 0.0 || !targetRpm.isFinite() || targetRpm < 0.0 ||
            !dt.isFinite() || dt <= 0.0 ||
            !config.kV.isFinite() || config.kV <= 0.0 || !config.kA.isFinite() || config.kA <= 0.0 ||
            !config.rpmTolerance.isFinite() || config.rpmTolerance <= 0.0 ||
            !config.voltageTolerance.isFinite() || config.voltageTolerance <= 0.0) return null

        // Exact zero-order-hold discretization. expm1 avoids cancellation at short dt.
        val oneMinusA = -expm1(-config.kV / config.kA * dt)
        val a = 1.0 - oneMinusA
        val b = oneMinusA / config.kV
        // Normalize state/input by their tolerances, making Q=R=1. In RPM/volts
        // this is Q=1/rpmTolerance^2 and R=1/voltageTolerance^2 (Bryson's rule).
        val bn = b * config.voltageTolerance / config.rpmTolerance
        val b2 = bn * bn
        if (!b2.isFinite() || b2 <= 0.0) return null
        // Scalar discrete algebraic Riccati equation:
        // b^2 P^2 + (1-a^2-b^2) P - 1 = 0. Select the stabilizing positive root.
        val c = 1.0 - a*a - b2
        val discriminant = hypot(c, 2.0*bn)
        val p = if (c >= 0.0) 2.0/(c+discriminant) else (discriminant-c)/(2.0*b2)
        val gain = (bn*p*a/(1.0+b2*p)) * config.voltageTolerance/config.rpmTolerance
        val volts = config.kV*targetRpm + gain*(targetRpm-rpm)
        if (!p.isFinite() || p <= 0.0 || !gain.isFinite() || gain < 0.0 || !volts.isFinite()) return null
        feedbackGain = gain
        return volts
    }
}
