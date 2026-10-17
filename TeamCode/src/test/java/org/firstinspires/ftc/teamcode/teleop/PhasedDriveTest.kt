package org.firstinspires.ftc.teamcode.teleop
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.Subsytem.DriveSubsystem
import org.junit.Test
import org.mockito.Mockito.*
class PhasedDriveTest {
 @Test fun `compute buffers final drive output and commit suppresses duplicates`() {
  val map = mock(HardwareMap::class.java)
  val motors = listOf("driveFL", "driveFR", "driveBL", "driveBR").map { name ->
   mock(DcMotor::class.java).also { `when`(map.get(DcMotor::class.java, name)).thenReturn(it) }
  }
  val drive = DriveSubsystem(map)
  drive.enablePhasedIo()
  clearInvocations(*motors.toTypedArray())
  drive.drive(1.0,0.0,0.0)
  drive.stop()
  drive.drive(0.5,0.0,0.0)
  verifyNoInteractions(*motors.toTypedArray())
  drive.commitOutputs()
  motors.forEach { verify(it).power = 0.45 }
  clearInvocations(*motors.toTypedArray())
  drive.commitOutputs()
  verifyNoInteractions(*motors.toTypedArray())
  drive.forceStop()
  motors.forEach { verify(it).power = 0.0 }
 }
}
