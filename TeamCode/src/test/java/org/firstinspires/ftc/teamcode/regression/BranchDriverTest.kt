package org.firstinspires.ftc.teamcode.regression

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.Telemetry
import com.pedropathing.math.Pose
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.navigation.*
import org.firstinspires.ftc.teamcode.teleop.*
import org.junit.Assert.*
import org.junit.Test
import org.mockito.Mockito.*

class BranchDriverTest {
    @Test fun `pollen toggle runs collection and manual stick cancels in the same frame`() {
        val r = BranchDriverRig()
        try {
            r.g2.a = true
            r.driver.update(r.g1, r.g2)
            assertTrue(r.hardware.drive.commandedPowerMagnitude > 0.0)
            verify(r.motors[6]).power = 1.0
            verify(r.physical, times(1)).update()
            r.clearHardware()
            r.g1.left_stick_x = -1f
            r.driver.update(r.g1, r.g2)
            verify(r.motors[0]).power = -0.9
            verify(r.motors[1]).power = 0.9
            // Holding G2 A cannot retrigger the cancelled macro.
            r.g1.left_stick_x = 0f
            r.driver.update(r.g1, r.g2)
            assertEquals(0.0, r.hardware.drive.commandedPowerMagnitude, 0.0)
        } finally { r.hardware.close() }
    }

    @Test fun `shooting and intake stop both cancel pollen without stealing existing controls`() {
        for (shooting in listOf(true, false)) {
            val r = BranchDriverRig()
            try {
                r.g2.a = true
                r.driver.update(r.g1, r.g2)
                r.g1.right_bumper = shooting
                r.g1.a = !shooting
                r.driver.update(r.g1, r.g2)
                assertEquals(shooting, r.hardware.hold.active)
                assertEquals(0.0, r.hardware.drive.commandedPowerMagnitude, 0.0)
                r.g1.right_bumper = false; r.g1.a = false
                r.driver.update(r.g1, r.g2)
                assertEquals(0.0, r.hardware.drive.commandedPowerMagnitude, 0.0)
            } finally { r.hardware.close() }
        }
    }

    @Test fun `red and blue hive saves are independent and existing aim still works`() {
        val r = BranchDriverRig()
        fun pose(x: Double, y: Double) {
            `when`(r.physical.getPosX(DistanceUnit.INCH)).thenReturn(x)
            `when`(r.physical.getPosY(DistanceUnit.INCH)).thenReturn(y)
        }
        fun tick() { r.displayNow += 50_000_000L; r.driver.update(r.g1, r.g2) }
        try {
            pose(10.0, 0.0); r.g1.x = true; tick()
            r.g1.x = false; r.g2.dpad_right = true; tick()
            pose(0.0, 10.0); r.g1.x = true; tick()
            pose(0.0, 0.0); r.g1.x = false; r.g2.dpad_right = false; r.g2.y = true; tick()
            assertTrue(r.hardware.drive.commandedPowerMagnitude > 0.0)
            r.g2.y = false; r.g2.dpad_left = true; tick()
            r.g2.dpad_left = false; r.g2.y = true; tick()
            assertEquals(0.0, r.hardware.drive.commandedPowerMagnitude, 0.0)
            verify(r.ds, atLeastOnce()).addData("Selected hive (G2 D-pad left/right)", "Red" as Any)
            verify(r.ds, atLeastOnce()).addData("Selected hive (G2 D-pad left/right)", "Blue" as Any)
        } finally { r.hardware.close() }
    }

    @Test fun `approach needs a saved hive and uses the shared pose once`() {
        val r = BranchDriverRig()
        try {
            r.g2.x = true; r.driver.update(r.g1, r.g2)
            assertEquals(0.0, r.hardware.drive.commandedPowerMagnitude, 0.0)
            r.g2.x = false; r.g1.x = true; r.driver.update(r.g1, r.g2)
            r.g1.x = false; r.g2.x = true; r.clearHardware()
            r.driver.update(r.g1, r.g2)
            assertTrue(r.hardware.drive.commandedPowerMagnitude > 0.0)
            verify(r.physical, times(1)).update()
            r.g1.right_stick_x = 1f; r.driver.update(r.g1, r.g2)
            assertEquals(0.9, r.hardware.drive.commandedPowerMagnitude, 0.0)
        } finally { r.hardware.close() }
    }

    @Test fun `pollen and hive offsets reject bad inputs and finish with a zero drive request`() {
        assertThrows(IllegalArgumentException::class.java) { PollenMacro(Double.NaN) }
        assertThrows(IllegalArgumentException::class.java) { PollenMacro(10.0, -1) }
        val hive = hiveApproach(Pose(5.0, 7.0, Math.PI / 2))
        assertEquals(8.0, hive.x(), 1e-9)
        assertEquals(60.369, hive.y(), 1e-9)
        val r = BranchDriverRig()
        try {
            val macro = PollenMacro(10.0, 1)
            assertThrows(IllegalArgumentException::class.java) { macro.start(Pose(Double.NaN, 0.0, 0.0)) }
            macro.start(Pose(0.0, 0.0, 0.0))
            macro.update(Pose(10.0, 0.0, 0.0), r.hardware.drive)
            for (i in 1..16) macro.update(Pose(10 * kotlin.math.cos(Math.PI*i/16),
                10 * kotlin.math.sin(Math.PI*i/16), 0.0), r.hardware.drive)
            assertFalse(macro.active)
            assertFalse(macro.collecting)
            assertEquals(0.0, r.hardware.drive.commandedPowerMagnitude, 0.0)
        } finally { r.hardware.close() }
    }

    @Test fun `idle voltage sampling is reduced and shooting restores fast compensation`() {
        for ((active, expected) in listOf(false to 10, true to 40)) {
            val sensor = mock(VoltageSensor::class.java)
            `when`(sensor.voltage).thenReturn(12.0)
            var now = 0L
            BatteryVoltageMonitor(listOf(sensor)) { now }.use { monitor ->
                monitor.active = active
                repeat(40) { monitor.pollIfDue(); now += 250_000_000L }
                verify(sensor, times(expected)).voltage
                if (!active) {
                    monitor.pollIfDue()
                    monitor.active = true; now += 250_000_000L
                    monitor.pollIfDue()
                    verify(sensor, times(12)).voltage
                }
            }
        }
    }
}

internal class BranchDriverRig {
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
