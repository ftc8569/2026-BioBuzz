package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.*
import org.junit.Assert.*
import org.junit.Test
import org.mockito.Mockito.*

/** Real installed Pedro, Foresight and LQR code; only hardware and Android logging are doubled. */
class AutoLifecycleIntegrationTest {
    private val autos = listOf<() -> AutoOpMode>(::BioBuzzAuto, { BasicBlueAuto() },
        ::BioBuzzRed1_5, ::BioBuzzAutoBlue, ::BackupAutoRedLeft, ::BackupAutoRedRight,
        ::BackupAutoBlueLeft, ::BackupAutoBlueRight)

    private class Rig(val auto: AutoOpMode) {
        val map = mock(HardwareMap::class.java)
        val pinpoint = mock(GoBildaPinpointDriver::class.java)
        val blocker = mock(Servo::class.java)
        val motors = listOf("driveFL", "driveFR", "driveBL", "driveBR", "shooter1", "shooter2", "intake1", "intake2")
            .associateWith { mock(DcMotorEx::class.java) }
        val powers = mutableMapOf<String, Double>()
        val hub = mock(LynxModule::class.java)
        init {
            val profile = mock(MotorConfigurationType::class.java)
            `when`(profile.ticksPerRev).thenReturn(28.0)
            for ((name, motor) in motors) {
                `when`(map.get(DcMotorEx::class.java, name)).thenReturn(motor)
                `when`(motor.motorType).thenReturn(profile)
                doAnswer {
                    val value = it.getArgument<Double>(0)
                    assertTrue(value.isFinite() && value in -1.0..1.0)
                    powers[name] = value
                    null
                }.`when`(motor).setPower(anyDouble())
            }
            `when`(map.get(Servo::class.java, "blocker")).thenReturn(blocker)
            `when`(map.get(GoBildaPinpointDriver::class.java, "pinpoint")).thenReturn(pinpoint)
            `when`(map.getAll(LynxModule::class.java)).thenReturn(listOf(hub))
            `when`(hub.bulkCachingMode).thenReturn(LynxModule.BulkCachingMode.OFF)
            var pose = Pose2D(DistanceUnit.INCH, 0.0, 0.0, AngleUnit.RADIANS, 0.0)
            doAnswer { pose = it.getArgument(0); null }.`when`(pinpoint).setPosition(any(Pose2D::class.java))
            `when`(pinpoint.getPosX(DistanceUnit.INCH)).thenAnswer { pose.getX(DistanceUnit.INCH) }
            `when`(pinpoint.getPosY(DistanceUnit.INCH)).thenAnswer { pose.getY(DistanceUnit.INCH) }
            `when`(pinpoint.getHeading(AngleUnit.RADIANS)).thenAnswer { pose.getHeading(AngleUnit.RADIANS) }
            auto.hardwareMap = map
            auto.telemetry = mock(Telemetry::class.java)
        }
    }

    @Test fun `every auto stays stationary before START and stops without reading localization`() {
        for (create in autos) {
            val r = Rig(create())
            r.auto.init()
            r.auto.init_loop()
            verify(r.auto.telemetry).addData(eq("Auto stage"), argThat<Any> { it?.toString() == "READY" })
            r.auto.loop()
            assertTrue(r.powers.values.all { it == 0.0 })
            r.auto.start()
            r.auto.loop()
            clearInvocations(r.pinpoint, r.hub)
            doThrow(IllegalStateException("Pinpoint disconnected")).`when`(r.pinpoint).update()
            r.auto.stop()
            assertTrue(r.powers.values.all { it == 0.0 })
            verify(r.pinpoint, never()).update()
            verify(r.hub).bulkCachingMode = LynxModule.BulkCachingMode.OFF
        }
    }

    @Test fun `real localizer failure plus failed motor stop preserves cause and stops remaining outputs`() {
        mockStatic(RobotLog::class.java).use { logs ->
            for (create in autos) {
                val r = Rig(create())
                r.auto.init()
                r.auto.start()
                r.auto.loop()
                val cause = IllegalStateException("Pinpoint disconnected")
                doThrow(cause).`when`(r.pinpoint).update()
                val failedStop = IllegalStateException("driveFL stop failed")
                doThrow(failedStop).`when`(r.motors.getValue("driveFL")).setPower(0.0)
                clearInvocations(r.hub)
                r.auto.loop()
                assertTrue(r.powers.filterKeys { it != "driveFL" }.values.all { it == 0.0 })
                assertTrue(cause.suppressed.isNotEmpty())
                verify(r.hub, times(1)).clearBulkCache()
                verify(r.hub).bulkCachingMode = LynxModule.BulkCachingMode.OFF
                verify(r.auto.telemetry, atLeastOnce()).addData(eq("Auto error"), contains("LOOP Pedro / Pinpoint") as Any)
                logs.verify { RobotLog.ee(eq("BioBuzzAuto"), same(cause), contains("Pinpoint disconnected")) }
                clearInvocations(r.pinpoint)
                r.auto.start()
                r.auto.loop()
                r.auto.stop()
                verify(r.pinpoint, never()).update()
            }
        }
    }

    @Test fun `partial initialization stops motors acquired after a missing device and closes blocker`() {
        mockStatic(RobotLog::class.java).use {
            for (create in autos.take(4)) {
                val r = Rig(create())
                `when`(r.map.get(DcMotorEx::class.java, "driveFR")).thenThrow(IllegalArgumentException("driveFR missing"))
                r.auto.init()
                r.auto.start()
                r.auto.loop()
                assertEquals(7, r.powers.size)
                assertTrue(r.powers.values.all { it == 0.0 })
                verify(r.blocker, atLeastOnce()).position = 0.78
                verifyNoInteractions(r.pinpoint)
                verify(r.auto.telemetry, atLeastOnce()).addData(eq("Auto error"), contains("driveFR missing") as Any)
                r.auto.stop()
            }
        }
    }
}
