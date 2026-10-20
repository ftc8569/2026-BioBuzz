package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.diagnostics.*
import org.firstinspires.ftc.teamcode.hardware.RobotHardware
import org.junit.Assert.*
import org.junit.Test
import org.mockito.Mockito.*

class TeleOpFrameLifecycleTest {
    private class Rig {
        var now = 0L
        val events = mutableListOf<String>()
        val profiler = LoopProfiler { now }
        val hardware = mock(RobotHardware::class.java, RETURNS_DEEP_STUBS)
        val ds = mock(Telemetry::class.java)
        val panels = mock(Telemetry::class.java)
        val driver: DriverRobot
        init {
            `when`(hardware.profiler).thenReturn(profiler)
            driver = spy(DriverRobot(hardware, RobotTelemetry(ds, panels, profiler) { 0L }) { now })
            doAnswer { events += "read"; now += 1_000_000; null }.`when`(hardware).read(true)
            doAnswer { events += "drive"; now += 2_000_000; null }.`when`(driver).updateDrive()
            doAnswer { events += "mechanisms"; now += 3_000_000; null }.`when`(driver).updateMechanisms()
            doAnswer { events += "write"; now += 4_000_000; null }.`when`(hardware).write()
            doAnswer { events += "telemetry"; now += 5_000_000; null }.`when`(driver).updateTelemetry()
        }
        fun update() = driver.update(Gamepad(), Gamepad())
    }

    @Test fun `real loop reads computes once commits and then publishes`() {
        val r = Rig()
        repeat(3) { r.update() }
        assertEquals(List(3) { listOf("read", "drive", "mechanisms", "write", "telemetry") }.flatten(), r.events)
        r.now += 1_000_000_000
        val samples = r.profiler.snapshotIfDue()!!.associateBy { it.stage }
        assertEquals(3L, samples.getValue(LoopStage.LOOP_WORK).calls)
        assertEquals(15.0, samples.getValue(LoopStage.LOOP_WORK).averageMs, 0.0)
        assertEquals(5.0, samples.getValue(LoopStage.COMPUTE).averageMs, 0.0)
    }

    @Test fun `failed read or compute never commits stale outputs and SDK entry point closes hardware`() {
        for (failRead in listOf(true, false)) {
            val r = Rig()
            if (failRead) doThrow(IllegalStateException("read failed")).`when`(r.hardware).read(true)
            else doThrow(IllegalStateException("compute failed")).`when`(r.driver).updateDrive()
            val op = Robot()
            op.gamepad1 = Gamepad(); op.gamepad2 = Gamepad()
            for ((name, value) in mapOf("running" to true, "hardware" to r.hardware, "driver" to r.driver))
                Robot::class.java.getDeclaredField(name).apply { isAccessible = true }.set(op, value)
            assertThrows(IllegalStateException::class.java) { op.loop() }
            op.loop()
            verify(r.hardware, never()).write()
            verify(r.hardware).close()
        }
    }

    @Test fun `throttled reporting does not throttle hardware or control loops`() {
        val r = RobotTestRig()
        repeat(3) { r.driver.update(r.g1, r.g2) }
        verify(r.physical, times(3)).update()
        verify(r.hub, times(3)).bulkData
        verify(r.ds, times(1)).update()
        verify(r.panels, times(1)).update()
    }
}
