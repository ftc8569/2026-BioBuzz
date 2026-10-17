package org.firstinspires.ftc.teamcode.Subsytem

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.junit.Test
import org.mockito.Mockito.*

class RobotLocationSubsystemTest {
    @Test fun `location reader uses Pedro calibration and converts inches to millimeters`() {
        val config = Constants.localizerConfig
        val oldX = config.xPodOffset.get()
        val oldY = config.yPodOffset.get()
        val oldUnits = config.offsetUnits.get()
        val pinpoint = mock(GoBildaPinpointDriver::class.java)
        val map = mock(HardwareMap::class.java)
        `when`(map.get(GoBildaPinpointDriver::class.java, "pinpoint")).thenReturn(pinpoint)
        try {
            config.xPodOffset.set(2.0)
            config.yPodOffset.set(-3.0)
            config.offsetUnits.set(DistanceUnit.INCH)
            RobotLocationSubsystem(map)
            verify(pinpoint).setOffsets(50.8, -76.19999999999999, DistanceUnit.MM)
            verify(pinpoint).setEncoderDirections(config.xPodDirection.get(), config.yPodDirection.get())
            verify(pinpoint).setEncoderResolution(config.podType.get())
        } finally {
            config.xPodOffset.set(oldX)
            config.yPodOffset.set(oldY)
            config.offsetUnits.set(oldUnits)
        }
    }
}
