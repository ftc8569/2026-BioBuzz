package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.junit.Test
import org.mockito.Mockito.*

class PhasedDriveTest {
 @Test fun `intake assist magnitude uses effective powers after Pedro threshold filtering`() {
  val map = mock(HardwareMap::class.java)
  val motors = listOf("driveFL", "driveFR", "driveBL", "driveBR").map { name ->
   mock(DcMotor::class.java).also { `when`(map.get(DcMotor::class.java, name)).thenReturn(it) }
  }
  val drive = DriveSubsystem(map)
  drive.enablePhasedIo()
  drive.requestWheelPowers(DoubleArray(4) { 0.499 }, 0.01)
  drive.commitOutputs()
  clearInvocations(*motors.toTypedArray())
  drive.requestWheelPowers(DoubleArray(4) { 0.501 }, 0.01)
  org.junit.Assert.assertEquals(0.499, drive.commandedPowerMagnitude, 0.0)
  drive.commitOutputs()
  verifyNoInteractions(*motors.toTypedArray())
  drive.requestWheelPowers(DoubleArray(4) { 0.52 }, 0.01)
  org.junit.Assert.assertEquals(0.52, drive.commandedPowerMagnitude, 0.0)
  drive.commitOutputs()
  motors.forEach { verify(it).power = 0.52 }
 }

}
