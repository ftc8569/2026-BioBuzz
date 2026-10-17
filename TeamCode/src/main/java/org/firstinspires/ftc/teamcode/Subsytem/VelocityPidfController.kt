package org.firstinspires.ftc.teamcode.Subsytem

/** Software PIDF: target and feedback in motor RPM, time in seconds, output in power. */
internal class VelocityPidfController {
    private var integralPower = 0.0

    fun reset() { integralPower = 0.0 }

    fun calculate(targetRpm: Double, measuredRpm: Double, dtSeconds: Double): Double {
        require(targetRpm.isFinite() && targetRpm > 0.0 && targetRpm <= ShooterSubsystem.MAX_MOTOR_RPM)
        require(measuredRpm.isFinite())
        require(dtSeconds.isFinite() && dtSeconds >= 0.0)

        // Fixed gains across presets: adjusting the RPM target must not change tuning.
        val kF = 1.0 / ShooterSubsystem.MAX_MOTOR_RPM
        // Strong speed-error correction: full power during spin-up and large shot dips.
        val kP = 2.0 / ShooterSubsystem.MAX_MOTOR_RPM
        val kI = 0.75 / ShooterSubsystem.MAX_MOTOR_RPM
        val errorRpm = targetRpm - measuredRpm
        val feedForward = kF * targetRpm
        // D remains zero until encoder noise is measured on the robot.
        val candidate = (integralPower + kI * errorRpm * dtSeconds.coerceAtMost(0.1))
            .coerceIn(-0.20, 0.20)
        val proportional = kP * errorRpm
        val proposed = feedForward + proportional + candidate
        if (proposed in 0.0..1.0 || (proposed > 1.0 && errorRpm < 0.0) ||
            (proposed < 0.0 && errorRpm > 0.0)) {
            integralPower = candidate
        }
        return (feedForward + proportional + integralPower).coerceIn(0.0, 1.0)
    }
}
