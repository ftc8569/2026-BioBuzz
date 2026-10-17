package org.firstinspires.ftc.teamcode.Subsytem

import com.bylazar.configurables.annotations.Configurable

/** Panels > Configurables > ShooterPidfTuning. Changes apply on the next control loop. */
@Configurable
class ShooterPidfTuning {
    companion object {
        // Raw RPM gains; defaults preserve the existing aggressive controller.
        @JvmField @Volatile var kP = VelocityPidfController.kP
        @JvmField @Volatile var kI = VelocityPidfController.kI
        @JvmField @Volatile var kD = 0.0
        @JvmField @Volatile var kF = VelocityPidfController.kF
    }
}
