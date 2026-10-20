package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.junit.Test
import org.mockito.Mockito.*

class IntakeSubsystemTest {
    @Test fun `intake clears inherited run mode and powers both motors`() {
        val map = mock(HardwareMap::class.java)
        val motor1 = mock(DcMotorEx::class.java)
        val motor2 = mock(DcMotorEx::class.java)
        `when`(map.get(DcMotorEx::class.java, "intake1")).thenReturn(motor1)
        `when`(map.get(DcMotorEx::class.java, "intake2")).thenReturn(motor2)
        val intake = IntakeSubsystem(map)
        for (motor in listOf(motor1, motor2)) {
            verify(motor).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            verify(motor).setPower(0.0)
        }
        verify(motor1).setDirection(DcMotorSimple.Direction.REVERSE)
        verify(motor2).setDirection(DcMotorSimple.Direction.FORWARD)
        intake.setPower(-0.8)
        for (motor in listOf(motor1, motor2)) verify(motor).setPower(-0.8)
        intake.setPower(0.66)
        for (motor in listOf(motor1, motor2)) verify(motor).setPower(0.66)
        intake.setPower(Double.NaN)
        for (motor in listOf(motor1, motor2)) verify(motor, times(2)).setPower(0.0)
    }
}
