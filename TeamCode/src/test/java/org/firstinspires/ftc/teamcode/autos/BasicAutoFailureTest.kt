package org.firstinspires.ftc.teamcode.autos

import com.pedropathing.drivetrain.Drivetrain
import com.pedropathing.follower.Follower
import com.pedropathing.paths.Path
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.junit.Assert.*
import org.junit.Test
import org.mockito.ArgumentMatchers.anyDouble
import org.mockito.Mockito.*

class BasicAutoFailureTest {
    private class Rig {
        val auto = BioBuzzAuto()
        val map = mock(HardwareMap::class.java)
        val follower = mock(Follower::class.java)
        val drivetrain = mock(Drivetrain::class.java)
        val hub = mock(LynxModule::class.java)
        val motors = listOf("driveFL", "driveFR", "driveBL", "driveBR", "shooter1", "shooter2", "intake1", "intake2")
            .associateWith { mock(DcMotorEx::class.java) }
        val powers = motors.mapValues { 0.5 }.toMutableMap()
        val blocker = mock(Servo::class.java)
        init {
            Follower::class.java.getField("drivetrain").apply { isAccessible = true }.set(follower, drivetrain)
            val profile = mock(MotorConfigurationType::class.java)
            `when`(profile.ticksPerRev).thenReturn(28.0)
            for ((name, motor) in motors) {
                `when`(map.get(DcMotorEx::class.java, name)).thenReturn(motor)
                `when`(motor.motorType).thenReturn(profile)
                doAnswer { powers[name] = it.getArgument(0); null }.`when`(motor).setPower(anyDouble())
            }
            `when`(map.get(Servo::class.java, "blocker")).thenReturn(blocker)
            `when`(map.getAll(LynxModule::class.java)).thenReturn(listOf(hub))
            auto.hardwareMap = map
            auto.telemetry = mock(Telemetry::class.java)
        }
        fun init() {
            mockStatic(Constants::class.java).use {
                it.`when`<Follower> { Constants.create(map) }.thenReturn(follower)
                auto.init()
            }
        }
        fun errorContains(text: String) = verify(auto.telemetry, atLeastOnce()).addData(eq("Auto error"), contains(text) as Any)
    }

    @Test fun `failed initialization is visible and START cannot run partially initialized auto`() {
        mockStatic(RobotLog::class.java).use {
            val rig = Rig()
            `when`(rig.map.get(DcMotorEx::class.java, "driveFR"))
                .thenThrow(IllegalArgumentException("No device named driveFR"))
            rig.init()
            rig.auto.init_loop()
            rig.auto.start()
            rig.auto.loop()
            rig.errorContains("INIT motor setup")
            rig.errorContains("driveFR")
            assertEquals(0.0, rig.powers.getValue("driveFL"), 0.0)
            verifyNoInteractions(rig.follower)
            rig.auto.stop()
        }
    }

    @Test fun `START failure stops every known output and reports the original error`() {
        mockStatic(RobotLog::class.java).use {
            val rig = Rig()
            rig.init()
            rig.powers.replaceAll { _, _ -> 0.5 }
            doThrow(IllegalStateException("drive stop failed")).`when`(rig.drivetrain).stop()
            rig.auto.start()
            for (power in rig.powers.values) assertEquals(0.0, power, 0.0)
            rig.errorContains("START")
            rig.errorContains("drive stop failed")
            verify(rig.follower, never()).follow(any(Path::class.java))
            verify(rig.hub).bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
    }

    @Test fun `Pinpoint failure and failed motor shutdown still stop other motors and preserve first cause`() {
        mockStatic(RobotLog::class.java).use {
            val rig = Rig()
            rig.init()
            rig.auto.start()
            rig.powers.replaceAll { _, _ -> 0.5 }
            doThrow(IllegalStateException("Pinpoint disconnected")).`when`(rig.follower).update()
            doThrow(IllegalStateException("shooter1 write failed")).`when`(rig.motors.getValue("shooter1")).setPower(0.0)
            rig.auto.loop()
            for ((name, power) in rig.powers) if (name != "shooter1") assertEquals(name, 0.0, power, 0.0)
            rig.errorContains("LOOP Pedro / Pinpoint")
            rig.errorContains("Pinpoint disconnected")
            verify(rig.blocker, atLeastOnce()).position = 0.78
            clearInvocations(rig.follower)
            rig.auto.loop()
            verifyNoInteractions(rig.follower)
        }
    }

    @Test fun `STOP does not require another localizer update`() {
        val rig = Rig()
        rig.init()
        rig.auto.start()
        clearInvocations(rig.follower)
        doThrow(IllegalStateException("cannot read Pinpoint")).`when`(rig.follower).update()
        rig.auto.stop()
        for (power in rig.powers.values) assertEquals(0.0, power, 0.0)
        verify(rig.follower, never()).update()
    }

    @Test fun `START before INIT reports incomplete initialization instead of null access`() {
        mockStatic(RobotLog::class.java).use {
            val auto = BioBuzzAuto()
            auto.telemetry = mock(Telemetry::class.java)
            auto.start()
            verify(auto.telemetry).addData(eq("Auto error"), contains("initialization is incomplete") as Any)
        }
    }
}
