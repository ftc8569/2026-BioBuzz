package org.firstinspires.ftc.teamcode.Subsytem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.junit.Assert.assertEquals
import org.junit.Test
import org.mockito.Mockito.*

class DriveSubsystemTest {
    @Test fun `reported power uses scaled wheel commands and resets on stop`() {
        val map = mock(HardwareMap::class.java)
        for (name in listOf("driveFL", "driveFR", "driveBL", "driveBR")) {
            `when`(map.get(DcMotor::class.java, name)).thenReturn(mock(DcMotor::class.java))
        }
        val drive = DriveSubsystem(map)
        assertEquals(0.0, drive.commandedPowerMagnitude, 0.0)
        drive.drive(0.2, 0.0, 0.0)
        assertEquals(0.18, drive.commandedPowerMagnitude, 1e-9)
        drive.drive(0.0, 0.15, 0.15)
        assertEquals(0.27, drive.commandedPowerMagnitude, 1e-9)
        drive.drive(-1.0, -1.0, -1.0)
        assertEquals(0.9, drive.commandedPowerMagnitude, 1e-9)
        drive.stop()
        assertEquals(0.0, drive.commandedPowerMagnitude, 0.0)
    }
}
