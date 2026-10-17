package org.firstinspires.ftc.teamcode.Subsytem

import com.bylazar.configurables.annotations.Configurable
import dev.frozenmilk.sinister.loading.Pinned

/** Panels > Configurables > ShooterPidTuning. FTCLib PIDF gains in RPM units. */
@Configurable
@Pinned
class ShooterPidTuning {
    @Pinned
    companion object {
        const val DEFAULT_KP = 0.002
        const val DEFAULT_KI = 0.00007
        const val DEFAULT_KD = 0.0
        const val DEFAULT_KF = 0.000212

        // Shared with Panels even when the controller is loaded through Sloth.
        @JvmField @Volatile var kP = DEFAULT_KP
        @JvmField @Volatile var kI = DEFAULT_KI
        @JvmField @Volatile var kD = DEFAULT_KD
        @JvmField @Volatile var kF = DEFAULT_KF
    }
}
