package org.firstinspires.ftc.teamcode.regression

import com.pedropathing.drivetrain.Drivetrain
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import com.qualcomm.robotcore.util.RobotLog
import java.util.function.LongSupplier
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.*
import org.firstinspires.ftc.teamcode.autos.*
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.teleop.*
import org.junit.Assert.*
import org.junit.Test
import org.mockito.ArgumentCaptor
import org.mockito.Mockito.*

/** Only devices are doubled: Pedro's follower, paths, controller, and LQR are the installed code. */
private class BranchRig(create: (LongSupplier) -> AutoOpMode) : AutoCloseable {
    var now = 0L
    val auto = create(LongSupplier { now })
    val map = mock(HardwareMap::class.java)
    val pinpoint = mock(GoBildaPinpointDriver::class.java)
    val blocker = mock(Servo::class.java)
    val hub = mock(LynxModule::class.java)
    val motors = listOf("driveFL", "driveFR", "driveBL", "driveBR", "shooter1", "shooter2", "intake1", "intake2")
        .associateWith { mock(DcMotorEx::class.java) }
    val powers = mutableMapOf<String, Double>()
    var pose = Pose2D(DistanceUnit.INCH, 0.0, 0.0, AngleUnit.RADIANS, 0.0)
    init {
        val profile = mock(MotorConfigurationType::class.java)
        `when`(profile.ticksPerRev).thenReturn(28.0)
        for ((name, motor) in motors) {
            `when`(map.get(DcMotorEx::class.java, name)).thenReturn(motor)
            `when`(motor.motorType).thenReturn(profile)
            doAnswer {
                val power = it.getArgument<Double>(0)
                assertTrue(power.isFinite() && power in -1.0..1.0)
                powers[name] = power
                null
            }.`when`(motor).setPower(anyDouble())
        }
        `when`(map.get(Servo::class.java, "blocker")).thenReturn(blocker)
        `when`(map.get(GoBildaPinpointDriver::class.java, "pinpoint")).thenReturn(pinpoint)
        `when`(map.getAll(LynxModule::class.java)).thenReturn(listOf(hub))
        `when`(hub.bulkCachingMode).thenReturn(LynxModule.BulkCachingMode.AUTO)
        doAnswer { pose = it.getArgument(0); null }.`when`(pinpoint).setPosition(any(Pose2D::class.java))
        `when`(pinpoint.getPosX(DistanceUnit.INCH)).thenAnswer { pose.getX(DistanceUnit.INCH) }
        `when`(pinpoint.getPosY(DistanceUnit.INCH)).thenAnswer { pose.getY(DistanceUnit.INCH) }
        `when`(pinpoint.getHeading(AngleUnit.RADIANS)).thenAnswer { pose.getHeading(AngleUnit.RADIANS) }
        auto.hardwareMap = map
        auto.telemetry = mock(Telemetry::class.java)
    }
    fun ready() { auto.init(); auto.init_loop() }
    override fun close() = auto.stop()
}

class BranchIntegrationTest {
    private val mains = listOf<(LongSupplier) -> AutoOpMode>(::MainLeft020Red, ::MainLeft020Blue,
        ::MainLeft034Red, ::MainLeft034Blue, ::MainRight004Red, ::MainRight004Blue,
        ::MainRight022Red, ::MainRight022Blue)
    private val preloads = listOf<(LongSupplier) -> AutoOpMode>(::RedPreloadParkClose, ::BluePreloadParkClose,
        ::RedPreloadParkFar, ::BluePreloadParkFar)

    @Test fun `all new path autos require READY use real follower and stop after localization failure`() {
        mockStatic(RobotLog::class.java).use { logs ->
            for (create in mains + preloads) BranchRig(create).use { r ->
                r.ready()
                verify(r.auto.telemetry).addData(eq("Auto stage"), argThat<Any> { it?.toString() == "READY" })
                r.auto.loop()
                assertTrue(r.powers.values.all { it == 0.0 })
                r.auto.start(); r.auto.loop()
                val failure = IllegalStateException("Pinpoint 100% disconnected")
                doThrow(failure).`when`(r.pinpoint).update()
                doThrow(IllegalStateException("FL stop rejected")).`when`(r.motors.getValue("driveFL")).setPower(0.0)
                clearInvocations(r.hub)
                r.now += 50_000_000L
                r.auto.loop()
                assertTrue(r.powers.filterKeys { it != "driveFL" }.values.all { it == 0.0 })
                assertTrue(failure.suppressed.isNotEmpty())
                verify(r.hub, times(1)).clearBulkCache()
                verify(r.hub).bulkCachingMode = LynxModule.BulkCachingMode.AUTO
                verify(r.blocker, atLeastOnce()).position = ShooterController.BLOCKER_CLOSED
                logs.verify { RobotLog.ee(eq("BioBuzzAuto"), same(failure), contains("Pinpoint 100% disconnected")) }
                clearInvocations(r.pinpoint)
                r.auto.start(); r.auto.loop(); r.auto.stop()
                verifyNoInteractions(r.pinpoint)
            }
        }
    }

    @Test fun `main autos force park and become fully idle at thirty seconds`() {
        for (create in mains) BranchRig(create).use { r ->
            r.ready(); r.auto.start(); r.auto.loop()
            r.now = 29_000_000_000L
            r.auto.loop()
            verify(r.auto.telemetry).addData("forced park", true)
            assertEquals(0.0, r.powers.getValue("shooter1"), 0.0)
            r.now = 30_000_000_000L
            r.auto.loop()
            assertTrue(r.powers.values.all { it == 0.0 })
            clearInvocations(r.pinpoint, r.hub, *r.motors.values.toTypedArray())
            repeat(3) { r.auto.loop() }
            verifyNoInteractions(r.pinpoint, r.hub, *r.motors.values.toTypedArray())
        }
    }

    @Test fun `blue main starts are the exact field-center rotation`() {
        for (pair in mains.chunked(2)) {
            BranchRig(pair[0]).use { red -> BranchRig(pair[1]).use { blue ->
                red.ready(); blue.ready()
                assertEquals(141.5, red.pose.getX(DistanceUnit.INCH) + blue.pose.getX(DistanceUnit.INCH), 1e-9)
                assertEquals(141.5, red.pose.getY(DistanceUnit.INCH) + blue.pose.getY(DistanceUnit.INCH), 1e-9)
                val difference = blue.pose.getHeading(AngleUnit.RADIANS) - red.pose.getHeading(AngleUnit.RADIANS)
                assertEquals(-1.0, kotlin.math.cos(difference), 1e-9)
            } }
        }
    }

    @Test fun `FIFO feed waits for speed and blocker then counts at most three balls`() {
        BranchRig(::MainRight004Red).use { r ->
            r.ready(); r.auto.start(); r.auto.loop()
            r.now = 6_100_000_000L // First drive times out; shot must stop its drive request.
            r.auto.loop()
            assertTrue(r.powers.filterKeys { it.startsWith("drive") }.values.all { it == 0.0 })
            assertEquals(0.0, r.powers.getValue("intake1"), 0.0)
            r.now += 500_000_000L; r.auto.loop()
            assertEquals(0.0, r.powers.getValue("intake1"), 0.0)
            `when`(r.motors.getValue("shooter1").velocity).thenReturn(3240.0 * 28.0 / 60.0)
            r.now += 10_000_000L; r.auto.loop()
            assertEquals(0.0, r.powers.getValue("intake1"), 0.0)
            r.now += 100_000_000L; r.auto.loop()
            assertEquals(0.6, r.powers.getValue("intake1"), 0.0)
            r.now += 4_499_000_000L; r.auto.loop()
            assertEquals(0.6, r.powers.getValue("intake1"), 0.0)
            r.now += 1_000_000L; r.auto.loop()
            assertEquals(0.0, r.powers.getValue("intake1"), 0.0)
        }
    }

    @Test fun `preload curves preserve branch endpoints and two full seconds of feeding`() {
        for ((i, create) in preloads.withIndex()) BranchRig(create).use { r ->
            val follower = mock(Follower::class.java)
            Follower::class.java.getField("drivetrain").apply { isAccessible = true }
                .set(follower, mock(Drivetrain::class.java))
            `when`(follower.pose()).thenReturn(Pose(0.0, 0.0, 0.0))
            `when`(follower.isBusy).thenReturn(true)
            mockStatic(Constants::class.java).use { constants ->
                constants.`when`<Follower> { Constants.create(r.map) }.thenReturn(follower)
                r.ready()
            }
            r.auto.start(); r.auto.loop()
            `when`(r.motors.getValue("shooter1").velocity).thenReturn(3240.0 * 28.0 / 60.0)
            r.now = 10_000_000L; r.auto.loop()
            r.now += 100_000_000L; r.auto.loop()
            assertEquals(0.6, r.powers.getValue("intake1"), 0.0)
            r.now += 1_999_000_000L; r.auto.loop()
            verify(follower, never()).follow(any(Path::class.java))
            r.now += 1_000_000L; r.auto.loop()
            val path = ArgumentCaptor.forClass(Path::class.java)
            verify(follower).follow(path.capture())
            val blue = i % 2 == 1
            val close = i < 2
            fun axis(n: Double) = if (blue) 141.5 - n else n
            assertEquals(axis(58.4934), path.value.get(0.0).x(), 1e-9)
            assertEquals(axis(if (close) 6.6 else 9.0389), path.value.get(0.0).y(), 1e-9)
            assertEquals(axis(if (close) 11.3267 else 12.3656), path.value.endPose().x(), 1e-9)
            assertEquals(axis(if (close) 94.1791 else 117.243), path.value.endPose().y(), 1e-9)
            val expectedHeading = Math.toRadians((if (close) 90.0 else -90.0) + if (blue) 180.0 else 0.0)
            val error = path.value.heading(1.0) - expectedHeading
            assertEquals(0.0, kotlin.math.atan2(kotlin.math.sin(error), kotlin.math.cos(error)), 1e-9)
        }
    }

    @Test fun `timed diagonal requires no mechanisms or pinpoint and stops at exactly 250 ms`() {
        BranchRig(::MoveForwardLeft).use { r ->
            r.ready(); r.auto.loop()
            assertEquals(4, r.powers.size)
            assertTrue(r.powers.values.all { it == 0.0 })
            r.auto.start()
            assertEquals(listOf(0.0, 0.5, 0.5, 0.0), r.powers.values.toList())
            r.now = 249_999_999L; r.auto.loop()
            assertEquals(0.5, r.powers.getValue("driveFR"), 0.0)
            r.now++; r.auto.loop()
            assertTrue(r.powers.values.all { it == 0.0 })
            verifyNoInteractions(r.pinpoint, r.blocker)
        }
    }

    @Test fun `timed move failed setup prevents START and still zeros all acquired motors`() {
        mockStatic(RobotLog::class.java).use {
            BranchRig(::MoveForwardLeft).use { r ->
                `when`(r.map.get(DcMotorEx::class.java, "driveFR")).thenThrow(IllegalStateException("missing FR"))
                r.ready(); r.auto.start(); r.auto.loop()
                assertEquals(3, r.powers.size)
                assertTrue(r.powers.values.all { it == 0.0 })
                verify(r.auto.telemetry, atLeastOnce()).addData(eq("Auto error"), contains("missing FR") as Any)
            }
        }
    }

    @Test fun `timed move startup failure stops each independent output`() {
        mockStatic(RobotLog::class.java).use {
            BranchRig(::MoveForwardLeft).use { r ->
                r.ready()
                doThrow(IllegalStateException("driveBL failed")).`when`(r.motors.getValue("driveBL")).setPower(0.5)
                r.auto.start()
                assertTrue(r.powers.values.all { it == 0.0 })
                verify(r.auto.telemetry, atLeastOnce()).addData(eq("Auto error"), contains("START") as Any)
                r.auto.loop()
            }
        }
    }

    @Test fun `all twenty one match entries have distinct names and public no-arg constructors`() {
        val added = (mains + preloads + ::MoveForwardLeft).map { it(LongSupplier { 0L }).javaClass }
        val original = listOf(BioBuzzAuto::class.java, BasicBlueAuto::class.java, BioBuzzRed1_5::class.java,
            BioBuzzAutoBlue::class.java, BackupAutoRedLeft::class.java, BackupAutoRedRight::class.java,
            BackupAutoBlueLeft::class.java, BackupAutoBlueRight::class.java)
        val names = (original + added).map { type ->
            type.getConstructor().newInstance()
            requireNotNull(type.getAnnotation(Autonomous::class.java)).name
        }
        assertEquals(21, names.size)
        assertEquals(21, names.toSet().size)
    }
}
