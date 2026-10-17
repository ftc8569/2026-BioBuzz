package org.firstinspires.ftc.teamcode.Subsytem

/** Software PIDF: target and feedback in motor RPM, time in seconds, output in power. */
internal class VelocityPidfController {
    companion object {
        // Defaults. Live tuning: Panels > Configurables > ShooterPidfTuning.
        // Feedforward supplies 0.64 power at the default 2880 RPM target.
        const val kF = 1.0 / ShooterSubsystem.MAX_MOTOR_RPM
        const val kP = 2.0 / ShooterSubsystem.MAX_MOTOR_RPM
        const val kI = 0.75 / ShooterSubsystem.MAX_MOTOR_RPM
        // D remains zero until encoder noise is measured on the robot.
    }

    private var integralPower = 0.0
    private var previousMeasuredRpm: Double? = null
    private data class Gains(val p: Double, val i: Double, val d: Double, val f: Double)
    private var previousGains: Gains? = null

    fun reset() {
        integralPower = 0.0
        previousMeasuredRpm = null
    }

    fun calculate(targetRpm: Double, measuredRpm: Double, dtSeconds: Double): Double {
        require(targetRpm.isFinite() && targetRpm > 0.0 && targetRpm <= ShooterSubsystem.MAX_MOTOR_RPM)
        require(measuredRpm.isFinite())
        require(dtSeconds.isFinite() && dtSeconds >= 0.0)

        fun validGain(value: Double, fallback: Double) =
            if (value.isFinite() && value >= 0.0) value else fallback
        val gains = Gains(
            validGain(ShooterPidfTuning.kP, kP),
            validGain(ShooterPidfTuning.kI, kI),
            validGain(ShooterPidfTuning.kD, 0.0),
            validGain(ShooterPidfTuning.kF, kF)
        )
        if (gains != previousGains) {
            reset()
            previousGains = gains
        }
        val errorRpm = targetRpm - measuredRpm
        val feedForward = gains.f * targetRpm
        val candidate = (integralPower + gains.i * errorRpm * dtSeconds.coerceAtMost(0.1))
            .coerceIn(-0.20, 0.20)
        val proportional = gains.p * errorRpm
        // Derivative on measurement avoids a power kick when the target changes.
        val previous = previousMeasuredRpm
        val derivative = if (previous != null && dtSeconds >= 0.0001 && gains.d > 0.0)
            -gains.d * (measuredRpm - previous) / dtSeconds else 0.0
        previousMeasuredRpm = measuredRpm
        val proposed = feedForward + proportional + candidate + derivative
        if (proposed in 0.0..1.0 || (proposed > 1.0 && errorRpm < 0.0) ||
            (proposed < 0.0 && errorRpm > 0.0)) {
            integralPower = candidate
        }
        return (feedForward + proportional + integralPower + derivative).coerceIn(0.0, 1.0)
    }
}
