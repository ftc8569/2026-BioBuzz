package org.firstinspires.ftc.teamcode.Subsytem

import com.bylazar.configurables.annotations.Configurable
import dev.frozenmilk.sinister.loading.Pinned

/** Panels > Configurables > ShooterLqrTuning. Initial model is ESTIMATED, not characterized. */
@Configurable
@Pinned
class ShooterLqrTuning {
    @Pinned
    companion object {
        // Seed steady-speed voltage from the previous 12 V normalized feedforward.
        const val DEFAULT_KV = 0.00240
        // Assumed 0.25 s first-order time constant; replace with fitted kA.
        const val DEFAULT_KA = 0.00015
        const val DEFAULT_RPM_TOLERANCE = 10.0
        const val DEFAULT_VOLTAGE_TOLERANCE = 3.0
        internal const val NOMINAL_PERIOD_SECONDS = 0.020

        @JvmField @Volatile var kV = DEFAULT_KV
        @JvmField @Volatile var kA = DEFAULT_KA
        // These choose LQR cost weights, not the feed-readiness threshold (still 1%).
        @JvmField @Volatile var rpmTolerance = DEFAULT_RPM_TOLERANCE
        @JvmField @Volatile var voltageTolerance = DEFAULT_VOLTAGE_TOLERANCE

        internal fun snapshot() = FlywheelLqrConfig(kV, kA, rpmTolerance, voltageTolerance)
    }
}
