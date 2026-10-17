package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import com.pedropathing.localization.Localizer
import com.pedropathing.localization.MotionState
import com.pedropathing.math.Pose
import com.pedropathing.math.Velocity
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Subsytem.*
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.junit.Assert.*
import org.junit.Test
import org.mockito.Mockito.*

class TeleOpHardwareTest {
    private class Rig {
        val map = mock(HardwareMap::class.java)
        val names = listOf("driveFL", "driveFR", "driveBL", "driveBR", "shooter1", "shooter2", "intake1", "intake2")
        val motors = names.map { name -> mock(DcMotorEx::class.java).also {
            `when`(map.get(DcMotor::class.java, name)).thenReturn(it)
            `when`(map.get(DcMotorEx::class.java, name)).thenReturn(it)
            val type = mock(MotorConfigurationType::class.java)
            `when`(type.ticksPerRev).thenReturn(28.0)
            `when`(it.motorType).thenReturn(type)
        } }
        val servo = mock(Servo::class.java)
        val hub = mock(LynxModule::class.java)
        val physical = mock(Localizer::class.java)
        val op = BioBuzzTeleOp()
        val drive: DriveSubsystem
        val shooter: ShooterSubsystem
        init {
            `when`(map.get(Servo::class.java, "blocker")).thenReturn(servo)
            `when`(map.getAll(VoltageSensor::class.java)).thenReturn(emptyList())
            `when`(physical.state()).thenReturn(MotionState.ofVelocity(Pose.zero(), Velocity.zero()))
            op.gamepad1 = Gamepad()
            op.gamepad2 = Gamepad()
            op.telemetry = mock(Telemetry::class.java)
            drive = DriveSubsystem(map).also { it.enablePhasedIo() }
            shooter = ShooterSubsystem(map) { 0L }.also { it.enablePhasedIo() }
            val intake = IntakeSubsystem(map).also { it.enablePhasedIo() }
            set("drive", drive); set("shooter", shooter); set("intake", intake)
            set("driveMotors", motors.take(4)); set("hubs", listOf(hub)); set("pinpoint", physical)
            val snapshot = field("snapshotLocalizer") as SnapshotLocalizer
            set("shootingDriveHold", ShootingDriveHold(Constants.create(snapshot, BufferedMecanum { p, t -> drive.requestWheelPowers(p,t) })))
            set("hardwareFrame", HardwareFrame({ call("readInputs") }, { call("writeOutputs") }))
            clearHardware()
        }
        fun set(name: String, value: Any) = BioBuzzTeleOp::class.java.getDeclaredField(name).apply { isAccessible = true }.set(op, value)
        fun field(name: String): Any = BioBuzzTeleOp::class.java.getDeclaredField(name).apply { isAccessible = true }.get(op)!!
        fun call(name: String) = BioBuzzTeleOp::class.java.getDeclaredMethod(name).apply { isAccessible = true }.invoke(op)
        fun read() = (field("hardwareFrame") as HardwareFrame).begin()
        fun compute() { call("updateDrive"); call("updateMechanisms") }
        fun clearHardware() = clearInvocations(*motors.toTypedArray(), servo, physical, hub)
        fun verifyNoHardware() = verifyNoInteractions(*motors.toTypedArray(), servo, physical, hub)
    }

    @Test fun `hold and hive save share one physical snapshot and computation and telemetry use caches`() {
        val r = Rig()
        r.op.gamepad1.y = true
        r.op.gamepad1.x = true
        r.op.gamepad1.b = true
        r.read()
        val order = inOrder(r.hub, r.motors[4], r.physical)
        order.verify(r.hub).clearBulkCache()
        order.verify(r.motors[4]).currentPosition
        order.verify(r.physical).update()
        verify(r.physical, times(1)).state()
        r.clearHardware()
        r.compute()
        r.verifyNoHardware()
        r.op.onUpdate()
        verify(r.motors[4]).power = org.mockito.AdditionalMatchers.eq(0.695, 1e-9)
        verify(r.servo).position = 0.65
        clearInvocations(*r.motors.toTypedArray(), r.servo)
        // Render again without a frame commit, to prove getters themselves are cache-only.
        r.set("hardwareFrame", HardwareFrame({}, {}))
        (r.field("hardwareFrame") as HardwareFrame).begin()
        r.op.onUpdate()
        r.verifyNoHardware()
    }

    @Test fun `manual hold manual handoff uses one output cache without an intermediate zero write`() {
        val r = Rig()
        r.op.gamepad1.left_stick_y = -0.5f
        r.read(); r.compute(); r.op.onUpdate()
        r.motors.take(4).forEach { verify(it).power = 0.45 }
        r.clearHardware()
        r.op.gamepad1.y = true
        r.read(); r.clearHardware(); r.compute(); r.verifyNoHardware(); r.op.onUpdate()
        r.motors.take(4).forEach { verify(it, times(1)).power = 0.0 }
        r.clearHardware()
        r.op.gamepad1.y = false
        r.read(); r.clearHardware(); r.compute(); r.op.onUpdate()
        r.motors.take(4).forEach { verify(it, times(1)).power = 0.45 }
    }

    @Test fun `stop attempts every actuator even when one motor rejects its stop`() {
        for (failedIndex in 0..7) {
            val r = Rig()
            doThrow(IllegalStateException("disconnected")).`when`(r.motors[failedIndex]).power = 0.0
            assertThrows(Exception::class.java) { r.op.onStop() }
            r.motors.filterIndexed { index, _ -> index != failedIndex }.forEach { verify(it).power = 0.0 }
            verify(r.servo).position = 0.78
        }
    }
}
