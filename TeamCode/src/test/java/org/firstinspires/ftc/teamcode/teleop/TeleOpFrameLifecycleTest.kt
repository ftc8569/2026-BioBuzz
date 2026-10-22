package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.nextftc.robot.opmode.NextOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.diagnostics.*
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

    @Test fun `failed read or compute never commits stale outputs`() {
        for (failRead in listOf(true, false)) {
            val r = Rig()
            if (failRead) doThrow(IllegalStateException("read failed")).`when`(r.hardware).read(true)
            else doThrow(IllegalStateException("compute failed")).`when`(r.driver).updateDrive()
            assertThrows(IllegalStateException::class.java) { r.update() }
            verify(r.hardware, never()).write()
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

    /** Run the installed NextFTC wrapper and scheduler with fake physical devices. */
    private class NextRig {
        val devices = RobotTestRig().also { it.hardware.close(); it.clearHardware() }
        val session = RobotSession { devices.panels }
        val powers = DoubleArray(8)
        var lastActivePowers = DoubleArray(8)
        val events = mutableListOf<String>()
        private var active = false

        init {
            devices.motors.forEachIndexed { index, motor ->
                doAnswer {
                    val power = it.getArgument<Double>(0)
                    assertTrue(power.isFinite() && power in -1.0..1.0)
                    if (!active) assertEquals("INIT must stay stationary", 0.0, power, 0.0)
                    powers[index] = power
                    events += "motor $index"
                    null
                }.`when`(motor).setPower(anyDouble())
            }
            `when`(devices.hub.bulkData).thenAnswer { events += "cache"; null }
            doAnswer { events += "pose"; null }.`when`(devices.physical).update()
            doAnswer { events += "report"; null }.`when`(devices.ds)
                .addData(eq("Loop interval (ms)"), any<Any>())
        }

        fun run(frames: Int, beforeFrame: (Int) -> Unit = {}) {
            active = false
            val create: () -> NextOpMode = { Robot(session) }
            val type = Class.forName("dev.nextftc.robot.opmode.BoundNextOpMode")
            val constructor = type.getDeclaredConstructor(
                Function0::class.java, Boolean::class.javaPrimitiveType)
            val op = spy(constructor.newInstance(create, false) as LinearOpMode)
            op.hardwareMap = devices.map
            op.telemetry = devices.ds
            op.gamepad1 = devices.g1
            op.gamepad2 = devices.g2
            `when`(op.opModeInInit()).thenReturn(true, false)
            doNothing().`when`(op).waitForStart()
            var frame = 0
            doAnswer {
                if (frame < frames) {
                    active = true
                    events += "frame $frame"
                    beforeFrame(frame++)
                    true
                } else {
                    lastActivePowers = powers.copyOf()
                    active = false
                    false
                }
            }.`when`(op).opModeIsActive()
            op.runOpMode()
        }

        fun verifyStopped() {
            assertTrue(powers.all { it == 0.0 })
            assertTrue(session.mechanisms.isEmpty())
            verify(devices.servo, atLeastOnce()).position = .78
            verify(devices.hub, atLeastOnce()).bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
    }

    @Test fun `scanned NextRobot construction does not resolve hardware or Panels`() {
        val session = RobotSession { error("Panels must only be resolved during INIT") }
        assertTrue(session.mechanisms.isEmpty())
    }

    @Test fun `real NextFTC lifecycle runs one ordered frame per scheduler tick and stops`() {
        val r = NextRig()
        r.devices.g1.left_stick_x = 1f
        r.run(3)
        verify(r.devices.hub, times(3)).bulkData
        // Installed PinpointLocalizer samples once in its constructor, then once per active frame.
        verify(r.devices.physical, times(4)).update()
        assertArrayEquals(doubleArrayOf(.9, -.9, -.9, .9),
            r.lastActivePowers.copyOfRange(0, 4), 1e-9)
        val first = r.events.subList(r.events.indexOf("frame 0"), r.events.indexOf("frame 1"))
        assertTrue(first.indexOf("cache") < first.indexOf("pose"))
        assertTrue(first.indexOf("pose") < first.indexOf("motor 0"))
        assertTrue(first.indexOf("motor 0") < first.indexOf("report"))
        r.verifyStopped()
    }

    @Test fun `NextFTC teardown restores caching when INIT is stopped without an active loop`() {
        val r = NextRig()
        r.devices.g1.y = true
        r.devices.g1.right_trigger = 1f
        r.run(0)
        verify(r.devices.physical, times(1)).update() // Constructor only, no active frame.
        verify(r.devices.hub, never()).bulkData
        r.verifyStopped()
    }

    @Test fun `one scanned robot creates fresh driver state for each NextFTC run`() {
        val r = NextRig()
        var firstDriver: Any? = null
        r.devices.g1.y = true
        r.devices.g2.dpad_up = true
        r.run(1) { firstDriver = r.session.mechanisms.single() }
        assertEquals(.725, r.lastActivePowers[4], 1e-9)
        r.devices.g2.dpad_up = false
        r.run(1) { assertNotSame(firstDriver, r.session.mechanisms.single()) }
        assertEquals(.72, r.lastActivePowers[4], 1e-9)
        r.verifyStopped()
    }

    @Test fun `linear gamepad updates during reads cannot change the captured control frame`() {
        val r = NextRig()
        r.devices.g1.left_stick_x = 1f
        r.devices.g1.right_trigger = .5f
        `when`(r.devices.hub.bulkData).thenAnswer {
            r.devices.g1.left_stick_x = -1f
            r.devices.g1.right_trigger = 0f
            r.devices.g1.left_trigger = 1f
            null
        }
        r.run(1)
        assertArrayEquals(doubleArrayOf(.9, -.9, -.9, .9, 0.0, 0.0, .5, .5),
            r.lastActivePowers, 1e-9)
        r.verifyStopped()
    }

    @Test fun `NextFTC loop failure preserves its cause while failed shutdown stops other devices`() {
        val r = NextRig()
        val cause = IllegalStateException("Pinpoint disconnected")
        val stopFailure = IllegalStateException("driveFL cannot stop")
        r.devices.g1.left_stick_x = 1f
        val failure = assertThrows(IllegalStateException::class.java) {
            r.run(2) { frame ->
                if (frame == 1) {
                    doThrow(cause).`when`(r.devices.physical).update()
                    doThrow(stopFailure).`when`(r.devices.motors[0]).setPower(0.0)
                }
            }
        }
        assertSame(cause, failure)
        assertTrue(cause.suppressed.any { it === stopFailure })
        assertTrue(r.powers.drop(1).all { it == 0.0 })
        assertTrue(r.session.mechanisms.isEmpty())
        verify(r.devices.physical, times(3)).update() // Constructor + two frames, never during cleanup.
        verify(r.devices.servo, atLeastOnce()).position = .78
        verify(r.devices.hub).bulkCachingMode = LynxModule.BulkCachingMode.AUTO
    }

    @Test fun `NextFTC partial initialization stops acquired devices and leaves no active mechanism`() {
        val r = NextRig()
        val cause = IllegalArgumentException("missing driveFR")
        `when`(r.devices.map.get(DcMotorEx::class.java, "driveFR")).thenThrow(cause)
        val failure = assertThrows(IllegalArgumentException::class.java) { r.run(1) }
        assertSame(cause, failure)
        assertTrue(r.session.mechanisms.isEmpty())
        r.devices.motors.filterIndexed { index, _ -> index != 1 }.forEach {
            verify(it, atLeastOnce()).power = 0.0
        }
        verify(r.devices.servo, atLeastOnce()).position = .78
        verifyNoInteractions(r.devices.physical)
    }
}
