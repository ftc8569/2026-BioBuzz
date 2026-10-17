package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/** goBILDA Yellow Jacket with the confirmed 1:1 gearbox, not the SDK's 99.5:1 default. */
@MotorType(ticksPerRev = 28, gearing = 1, maxRPM = 6000, orientation = Rotation.CCW)
@DeviceProperties(xmlTag = "BioBuzzShooter1To1", name = "BioBuzz shooter goBILDA 1:1")
public interface BioBuzzShooterMotor {
}
