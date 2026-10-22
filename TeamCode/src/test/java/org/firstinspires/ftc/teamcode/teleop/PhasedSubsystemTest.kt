package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.teamcode.subsystems.ShooterController
import org.firstinspires.ftc.teamcode.subsystems.*
import org.junit.Assert.*
import org.junit.Test
import org.mockito.Mockito.*

class PhasedSubsystemTest {
    private val map = mock(HardwareMap::class.java)
    private fun motor(name: String): DcMotorEx = mock(DcMotorEx::class.java).also {
        `when`(map.get(DcMotorEx::class.java, name)).thenReturn(it)
        `when`(map.get(DcMotor::class.java, name)).thenReturn(it)
        val type = mock(MotorConfigurationType::class.java)
        `when`(type.ticksPerRev).thenReturn(28.0)
        `when`(it.motorType).thenReturn(type)
    }

    @Test fun `drive commits only final request suppresses unchanged and force stops`() {
        val motors = listOf("driveFL", "driveFR", "driveBL", "driveBR").map(::motor)
        val drive = DriveSubsystem(map)
        drive.enablePhasedIo()
        clearInvocations(*motors.toTypedArray())
        drive.drive(1.0, 0.0, 0.0)
        drive.stop()
        drive.drive(0.5, 0.0, 0.0)
        verifyNoInteractions(*motors.toTypedArray())
        drive.commitOutputs()
        motors.forEach { verify(it).power = 0.45 }
        clearInvocations(*motors.toTypedArray())
        drive.drive(0.5, 0.0, 0.0)
        drive.commitOutputs()
        verifyNoInteractions(*motors.toTypedArray())
        drive.forceStop()
        motors.forEach { verify(it).power = 0.0 }
    }

    @Test fun `shooter reads once then computes and reports without hardware and commits final outputs`() {
        val m1 = motor("shooter1")
        val m2 = motor("shooter2")
        val servo = mock(Servo::class.java)
        `when`(map.get(Servo::class.java, "blocker")).thenReturn(servo)
        `when`(map.getAll(VoltageSensor::class.java)).thenReturn(emptyList())
        val shooter = ShooterSubsystem(map) { 0L }
        shooter.enablePhasedIo()
        clearInvocations(m1, m2, servo)
        shooter.beginFrame()
        verify(m1).currentPosition
        verify(m2).currentPosition
        verify(m1).velocity
        verify(m2).velocity
        clearInvocations(m1, m2, servo)
        shooter.setControlMode(ShooterController.ControlMode.MANUAL_POWER)
        shooter.updateShot(0.5, ShooterController.ControlMode.MANUAL_POWER)
        shooter.setManualBlockerOpen(true)
        assertEquals(0.5, shooter.flywheelPower, 0.0)
        assertEquals(0.65, shooter.control.blockerPosition, 0.0)
        verifyNoInteractions(m1, m2, servo)
        shooter.commitOutputs()
        verify(m1).power = 0.5
        verify(m2).power = 0.5
        verify(servo).position = 0.65
        verifyNoMoreInteractions(m1, m2, servo)
        clearInvocations(m1, m2, servo)
        shooter.commitOutputs()
        verifyNoInteractions(m1, m2, servo)
        shooter.forceStop()
        verify(m1).power = 0.0
        verify(m2).power = 0.0
        verify(servo).position = 0.78
    }

    @Test fun `intake buffers final output and suppresses unchanged writes`() {
        val motors = listOf(motor("intake1"), motor("intake2"))
        val intake = IntakeSubsystem(map)
        intake.enablePhasedIo()
        clearInvocations(*motors.toTypedArray())
        intake.setPower(1.0)
        intake.setPower(-0.2)
        verifyNoInteractions(*motors.toTypedArray())
        intake.commitOutputs()
        motors.forEach { verify(it).power = -0.2 }
        clearInvocations(*motors.toTypedArray())
        intake.commitOutputs()
        verifyNoInteractions(*motors.toTypedArray())
        intake.forceStop()
        motors.forEach { verify(it).power = 0.0 }
    }
    @Test fun `shooter keeps feedback and voltage stable throughout one frame`() {
        val m1 = motor("shooter1")
        val m2 = motor("shooter2")
        val servo = mock(Servo::class.java)
        val voltage = mock(VoltageSensor::class.java)
        `when`(map.get(Servo::class.java, "blocker")).thenReturn(servo)
        `when`(map.getAll(VoltageSensor::class.java)).thenReturn(listOf(voltage))
        `when`(voltage.voltage).thenReturn(12.0)
        `when`(m1.velocity).thenReturn(100.0)
        val shooter = ShooterSubsystem(map) { 0L }
        shooter.enablePhasedIo()
        shooter.batteryVoltage.pollOnce()
        shooter.beginFrame()
        `when`(voltage.voltage).thenReturn(10.0)
        `when`(m1.velocity).thenReturn(200.0)
        shooter.batteryVoltage.pollOnce() // Simulate the worker changing its cache mid-frame.
        clearInvocations(m1, m2, servo, voltage)
        shooter.updateShot(0.5, ShooterController.ControlMode.MANUAL_POWER)
        assertEquals(100.0, shooter.wheels[0].ticksPerSecond, 0.0)
        verifyNoInteractions(m1, m2, servo, voltage)
        shooter.commitOutputs()
        verify(m1).power = 0.5
        shooter.beginFrame()
        shooter.updateShot(0.5, ShooterController.ControlMode.MANUAL_POWER)
        assertEquals(200.0, shooter.wheels[0].ticksPerSecond, 0.0)
        shooter.commitOutputs()
        verify(m1).power = 0.6
    }

}
