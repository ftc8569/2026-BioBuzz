package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.*
import org.firstinspires.ftc.teamcode.diagnostics.*
import org.firstinspires.ftc.teamcode.hardware.RobotHardware
import org.junit.Assert.*
import org.junit.Test
import org.mockito.Mockito.*

/** Constructor injection with real subsystems, real Pedro and fake physical hardware. */
internal class RobotTestRig {
    var now = 0L
    var displayNow = 0L
    val map = mock(HardwareMap::class.java)
    val motors = listOf("driveFL", "driveFR", "driveBL", "driveBR", "shooter1", "shooter2", "intake1", "intake2")
        .map { name -> mock(DcMotorEx::class.java).also { motor ->
            `when`(map.get(DcMotor::class.java, name)).thenReturn(motor)
            `when`(map.get(DcMotorEx::class.java, name)).thenReturn(motor)
            val type = mock(MotorConfigurationType::class.java)
            `when`(type.ticksPerRev).thenReturn(28.0)
            `when`(motor.motorType).thenReturn(type)
            val controller = mock(DcMotorController::class.java)
            `when`(controller.connectionInfo).thenReturn("USB module 1")
            `when`(motor.controller).thenReturn(controller)
        } }
    val servo = mock(Servo::class.java)
    val hub = mock(LynxModule::class.java)
    val physical = mock(GoBildaPinpointDriver::class.java)
    val ds = mock(Telemetry::class.java)
    val panels = mock(Telemetry::class.java)
    val g1 = Gamepad()
    val g2 = Gamepad()
    val profiler = LoopProfiler { now }
    val hardware: RobotHardware
    val display: RobotTelemetry
    val driver: DriverRobot
    init {
        `when`(map.get(Servo::class.java, "blocker")).thenReturn(servo)
        `when`(map.get(GoBildaPinpointDriver::class.java, "pinpoint")).thenReturn(physical)
        `when`(map.getAll(LynxModule::class.java)).thenReturn(listOf(hub))
        `when`(hub.bulkCachingMode).thenReturn(LynxModule.BulkCachingMode.AUTO)
        `when`(hub.connectionInfo).thenReturn("USB module 1")
        hardware = RobotHardware(map, profiler) { now }
        display = RobotTelemetry(ds, panels, profiler) { displayNow }
        driver = DriverRobot(hardware, display) { now }
        clearHardware()
        clearInvocations(ds, panels)
    }
    fun read(diagnostics: Boolean = true) {
        driver.captureControls(g1, g2)
        hardware.read(diagnostics)
    }
    fun compute() { driver.updateDrive(); driver.updateMechanisms() }
    fun commit() { hardware.write(); driver.updateTelemetry() }
    fun clearHardware() = clearInvocations(*motors.toTypedArray(), servo, physical, hub)
    fun verifyNoHardware() = verifyNoInteractions(*motors.toTypedArray(), servo, physical, hub)
}

class TeleOpHardwareTest {
    @Test fun `hold and hive save use one physical snapshot and telemetry uses only caches`() {
        val r = RobotTestRig()
        r.g1.y = true; r.g1.x = true; r.g1.b = true
        r.read()
        val order = inOrder(r.hub, r.motors[4], r.physical)
        order.verify(r.hub).bulkData
        order.verify(r.motors[4]).currentPosition
        order.verify(r.physical).update()
        verify(r.physical, times(1)).update()
        r.clearHardware()
        r.compute()
        r.verifyNoHardware()
        r.commit()
        verify(r.motors[4]).power = org.mockito.AdditionalMatchers.eq(0.72, 1e-9)
        verify(r.servo).position = 0.65
        r.clearHardware()
        r.displayNow += 50_000_000
        r.driver.updateTelemetry()
        r.verifyNoHardware()
        verify(r.ds, times(2)).update()
        verify(r.panels, times(2)).update()
    }

    @Test fun `manual hold manual handoff has no intermediate zero write`() {
        val r = RobotTestRig()
        r.g1.left_stick_y = -0.5f
        r.read(); r.compute(); r.commit()
        r.motors.take(4).forEach { verify(it).power = 0.45 }
        r.clearHardware()
        r.g1.y = true
        r.read(); r.clearHardware(); r.compute(); r.verifyNoHardware(); r.commit()
        r.motors.take(4).forEach { verify(it, times(1)).power = 0.0 }
        r.clearHardware()
        r.g1.y = false
        r.read(); r.clearHardware(); r.compute(); r.commit()
        r.motors.take(4).forEach { verify(it, times(1)).power = 0.45 }
    }

    @Test fun `stop attempts every actuator and restores caching after any motor rejects zero`() {
        for (failedIndex in 0..7) {
            val r = RobotTestRig()
            doThrow(IllegalStateException("disconnected")).`when`(r.motors[failedIndex]).power = 0.0
            assertThrows(Exception::class.java) { r.hardware.close() }
            r.motors.filterIndexed { index, _ -> index != failedIndex }.forEach { verify(it).power = 0.0 }
            verify(r.servo).position = 0.78
            verify(r.hub).bulkCachingMode = LynxModule.BulkCachingMode.AUTO
            verify(r.physical, never()).update()
        }
    }

    @Test fun `bulk transaction time is separate from feedback decoding`() {
        val r = RobotTestRig()
        `when`(r.hub.bulkData).thenAnswer { r.now += 7_000_000; null }
        r.read()
        r.now += 1_000_000_000
        val samples = r.profiler.snapshotIfDue()!!.associateBy { it.stage }
        assertEquals(7.0, samples.getValue(LoopStage.BULK_READ).averageMs, 0.0)
        assertEquals(0.0, samples.getValue(LoopStage.SHOOTER_ENCODERS).averageMs, 0.0)
    }
}

class RobotFailureTest {
    @org.junit.Test fun `initialization failure stops all acquired motors and blocker without starting Pedro`() {
        val op = Robot()
        val map = mock(HardwareMap::class.java)
        op.hardwareMap = map
        val blocker = mock(Servo::class.java)
        `when`(map.get(Servo::class.java, "blocker")).thenReturn(blocker)
        val motors = listOf("driveFL", "driveFR", "driveBL", "driveBR", "shooter1", "shooter2", "intake1", "intake2")
            .associateWith { mock(DcMotorEx::class.java) }
        for ((name, motor) in motors) `when`(map.get(DcMotorEx::class.java, name)).thenReturn(motor)
        `when`(map.get(DcMotorEx::class.java, "driveFR")).thenThrow(IllegalStateException("missing driveFR"))
        val failure = assertThrows(IllegalStateException::class.java) { op.init() }
        assertEquals("missing driveFR", failure.message)
        for ((name, motor) in motors) if (name != "driveFR") verify(motor, atLeastOnce()).power = 0.0
        verify(blocker, atLeastOnce()).position = 0.78
        op.loop()
        assertThrows(IllegalStateException::class.java) { op.start() }
    }

    @org.junit.Test fun `cache restoration tries every hub even when a motor and another hub fail`() {
        val safety = org.firstinspires.ftc.teamcode.hardware.SafeHardware()
        val map = mock(HardwareMap::class.java)
        val motor = mock(DcMotorEx::class.java)
        `when`(map.get(DcMotorEx::class.java, "driveFL")).thenReturn(motor)
        safety.motor(map, "driveFL")
        val hubs = List(2) { mock(LynxModule::class.java) }
        `when`(hubs[0].bulkCachingMode).thenReturn(LynxModule.BulkCachingMode.OFF)
        `when`(hubs[1].bulkCachingMode).thenReturn(LynxModule.BulkCachingMode.AUTO)
        safety.setCaching(hubs, LynxModule.BulkCachingMode.MANUAL)
        doThrow(IllegalStateException("motor failed")).`when`(motor).power = 0.0
        doThrow(IllegalStateException("hub failed")).`when`(hubs[0]).bulkCachingMode = LynxModule.BulkCachingMode.OFF
        val cause = IllegalStateException("original failure")
        assertSame(cause, safety.stop(cause))
        assertEquals(2, cause.suppressed.size)
        verify(hubs[1]).bulkCachingMode = LynxModule.BulkCachingMode.AUTO
    }
}
