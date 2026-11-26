package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.RobotLocationSubsystem
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.math.Pose
import com.pedropathing.revhub.localizers.PinpointLocalizer
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.*
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sign
import org.firstinspires.ftc.teamcode.pedro.Constants

/**
 * Match TeleOp. Each loop: copy gamepads → read sensors once → compute → write changed outputs.
 *
 * G1: left stick drives; right stick turns; RB shoots with LQR; Y shoots manually;
 * LB reverses the flywheels; triggers run/reverse intake; A stops intake;
 * X saves the hive location; left-stick click toggles collection assist;
 * right-stick click toggles the detailed status rows.
 * Either gamepad: D-pad up/down adjusts shooter preset; B opens the blocker in manual mode.
 * G2 Y aims at the saved hive. Turning the right stick cancels aiming.
 * G2 D-pad left/right selects blue/red hive; G2 X approaches its saved location;
 * G2 A toggles the pollen sweep. Driver sticks, shooting, or G1 A cancel either drive assist.
 *
 * The blocker servo is not moved during INIT (as on main); it closes when START is pressed.
 */
@TeleOp(name = "BioBuzz NextFTC Kotlin TeleOp") // Driver Station name kept from main.
class Robot : OpMode() {
    private var hardware: RobotHardware? = null
    private var driver: DriverRobot? = null

    override fun init() {
        val devices = RobotHardware(hardwareMap).also { hardware = it }
        driver = DriverRobot(devices, RobotTelemetry(telemetry, PanelsTelemetry.ftcTelemetry))
    }

    override fun start() = stopOnFailure { checkNotNull(driver).start() }
    override fun loop() = stopOnFailure { checkNotNull(driver).update(gamepad1, gamepad2) }

    override fun stop() {
        val devices = hardware
        hardware = null
        driver = null
        devices?.close()
    }

    /** Stop every device before the SDK reports the failure; keep cleanup errors attached. */
    private inline fun stopOnFailure(action: () -> Unit) {
        try { action() } catch (failure: Throwable) {
            try { stop() } catch (cleanup: Throwable) {
                if (cleanup !== failure) failure.addSuppressed(cleanup)
            }
            throw failure
        }
    }
}

/** Driver state and the read → compute → write → report loop. */
internal class DriverRobot(
    private val hardware: RobotHardware,
    val telemetry: RobotTelemetry,
    nanoTime: () -> Long = System::nanoTime
) {
    private val drive = hardware.drive
    private val shooter = hardware.shooter
    private val hold = hardware.hold
    private val location: Pose get() = hardware.pose
    private val g1 = Gamepad()
    private val g2 = Gamepad()
    private val saveButton = PressEdge()
    private val aimButton = PressEdge()
    private val loggingButton = PressEdge()
    private val approachButton = PressEdge()
    private val pollenButton = PressEdge()
    private val pollen = PollenMacro()
    private val navigation = hardware.location
    private var hiveAlliance = RobotLocationSubsystem.Alliance.BLUE
    private val hiveName: String get() = if (hiveAlliance == RobotLocationSubsystem.Alliance.BLUE) "Blue" else "Red"
    private val loopTiming = LoopTiming(nanoTime)
    private val intakeFeed = IntakeFeed(feedPower = 1.0, nanoTime) // main: TeleOp feeds at full power.
    private val driveIntakeControls = DriveIntakeControls()
    private val shooterControls = ShooterControls()
    private var detailedLogging = true // main: detailed rows start on.
    private val savedHive: Pose? get() = navigation.savedHive(hiveAlliance)
    private var targetHeading: Double? = null
    private var aiming = false
    private var headingError: Double? = null
    private var shooterPower = 0.0
    private var intakePower = 0.0
    private val up: Boolean get() = g1.dpad_up || g2.dpad_up
    private val down: Boolean get() = g1.dpad_down || g2.dpad_down
    private val manualBlocker: Boolean get() = g1.b || g2.b

    fun start() {
        loopTiming.reset()
        telemetry.reset()
        hardware.start()
    }

    fun update(gamepad1: Gamepad, gamepad2: Gamepad) {
        // The SDK updates gamepads asynchronously; one copy keeps this loop consistent.
        g1.copy(gamepad1)
        g2.copy(gamepad2)
        if (loggingButton.pressed(g1.right_stick_button)) {
            detailedLogging = !detailedLogging
            telemetry.clear()
        }
        hardware.read()
        updateDrive()
        updateMechanisms()
        hardware.write()
        updateTelemetry()
    }

    private fun updateDrive() {
        val selected = if (g2.dpad_left) RobotLocationSubsystem.Alliance.BLUE
            else if (g2.dpad_right) RobotLocationSubsystem.Alliance.RED else hiveAlliance
        if (selected != hiveAlliance) {
            hiveAlliance = selected
            aiming = false
            targetHeading = null
            pollen.cancel()
        }
        if (saveButton.pressed(g1.x)) {
            pollen.cancel()
            navigation.saveHive(hiveAlliance)
            targetHeading = null
            aiming = false
        }
        if (aimButton.pressed(g2.y)) {
            pollen.cancel()
            targetHeading = navigation.headingToHive(hiveAlliance)
            aiming = targetHeading != null
        }
        val approachPressed = approachButton.pressed(g2.x)
        val pollenPressed = pollenButton.pressed(g2.a)
        val manualDrive = abs(g1.left_stick_x) > 0.05 || abs(g1.left_stick_y) > 0.05 ||
            abs(g1.right_stick_x) > 0.05
        val cancelAssist = manualDrive || g1.right_bumper || g1.y || g1.left_bumper || g1.a
        if (cancelAssist) pollen.cancel()
        else if (pollenPressed) {
            aiming = false
            if (pollen.active) pollen.cancel() else pollen.start(location)
        } else if (approachPressed) savedHive?.let { aiming = false; pollen.approach(it) }
        val turn = aimedTurn(g1.right_stick_x.toDouble())
        val shooting = (g1.right_bumper || g1.y) && !g1.left_bumper
        if (shooting && !hold.active) aiming = false
        val holding = (shooting || hold.active) && hold.update(shooting, location)
        if (!holding && !pollen.update(location, drive))
            drive.drive(-g1.left_stick_y.toDouble(), g1.left_stick_x.toDouble(), turn)
    }

    /** Turn toward the hive at 0.8 × error, between 0.12 and 0.35 power, until within 2°. */
    private fun aimedTurn(manual: Double): Double {
        headingError = null
        if (abs(manual) > 0.05) aiming = false
        val target = targetHeading?.takeIf { aiming } ?: return manual
        val delta = target - location.heading()
        val error = atan2(sin(delta), cos(delta))
        headingError = error
        aiming = abs(error) > 0.035
        return if (aiming) -sign(error) * (0.8 * abs(error)).coerceIn(0.12, 0.35)
            else manual
    }

    private fun updateMechanisms() {
        driveIntakeControls.update(g1.left_stick_button)
        shooterControls.update(g1.y, up, down)
        shooterPower = shooterControls.requestedPower(g1.left_bumper, g1.right_bumper)
        shooter.setControlMode(shooterControls.mode)
        if (shooterPower > 0.0) shooter.updateShot(shooterPower, shooterControls.mode)
        else shooter.setFlywheelPower(shooterPower)
        shooter.setManualBlockerOpen(manualBlocker)
        val manualIntake = when {
            g1.right_trigger > 0.05 -> g1.right_trigger.toDouble()
            g1.left_trigger > 0.05 -> -g1.left_trigger.toDouble()
            else -> 0.0
        }
        intakePower = intakeFeed.power(
            shooter, shooterPower, stop = g1.a, manualPower = manualIntake,
            manualBlockerOpen = manualBlocker,
            minimumPower = if (pollen.collecting) 1.0
                else driveIntakeControls.minimumPower(drive.commandedPowerMagnitude)
        )
        hardware.intake.setPower(intakePower)
    }

    private fun updateTelemetry() {
        loopTiming.update()
        telemetry.publish {
            reportShooter(shooter, shooterControls)
            addData("Drive", when {
                hold.active -> "Holding shot position"
                pollen.collecting -> "Pollen sweep"
                pollen.active -> "Hive approach"
                aiming -> "Aiming"
                else -> "Manual"
            })
            addData("Hive", "$hiveName: ${if (savedHive == null) "not saved" else "saved"}")
            if (!detailedLogging) return@publish
            // main's detailed view, minus rows that need extra hardware reads each loop.
            addData("Pose (in / deg)", "%.1f, %.1f, %.1f", location.x(), location.y(),
                Math.toDegrees(location.heading()))
            addData("Hive heading (deg)", targetHeading?.let { "%.1f".format(Math.toDegrees(it)) }
                ?: "Press G1 X to save hive, then G2 Y to aim")
            headingError?.let { addData("Heading error (deg)", "%.1f", Math.toDegrees(it)) }
            addData("Intake", "%.2f; assist %s (G1 left-stick click)", intakePower,
                if (driveIntakeControls.enabled) "on" else "off")
            addData("Speed state", shooter.control.speedStatus)
            addData("Blocker", shooter.control.blockerStatus)
            addData("LQR commanded RPM", "%.0f", shooter.control.targetRpm)
            addData("Loop (ms / Hz)", "%.1f / %.0f", loopTiming.intervalMs,
                if (loopTiming.intervalMs > 0.0) 1000.0 / loopTiming.intervalMs else 0.0)
        }
    }
}

/** Construct once during INIT. Controllers use these objects for the entire match. */
internal class RobotHardware(map: HardwareMap, nanoTime: () -> Long = System::nanoTime) : AutoCloseable {
    private val safety = SafeHardware()
    val drive: DriveSubsystem
    val intake: IntakeSubsystem
    val shooter: ShooterSubsystem
    val hold: ShootingDriveHold
    private val pinpoint: PinpointLocalizer
    val location = RobotLocationSubsystem()
    private val feedbackHubs: List<LynxModule>
    val pose: Pose get() = location.pose

    init {
        try {
            safety.prepareMotors(map, true)
            drive = DriveSubsystem(map).apply { enablePhasedIo() }
            intake = IntakeSubsystem(map).apply { enablePhasedIo() }
            shooter = ShooterSubsystem(map, nanoTime).apply { enablePhasedIo() }
            safety.onStop { shooter.forceStop() }
            safety.onStop { shooter.stopVoltagePolling() }
            pinpoint = PinpointLocalizer(map, Constants.localizerConfig)
            val drivetrain = BufferedMecanum(drive::requestWheelPowers)
            hold = ShootingDriveHold(Constants.create(location.localizer, drivetrain))
            safety.onStop { hold.stop() }
            val hubs = map.getAll(LynxModule::class.java)
            safety.setCaching(hubs, LynxModule.BulkCachingMode.MANUAL)
            // Only the hub with the shooter encoders needs a bulk read each loop.
            val shooterHubs = listOf("shooter1", "shooter2")
                .map { map.get(DcMotor::class.java, it).controller.connectionInfo }.toSet()
            feedbackHubs = hubs.filter { it.connectionInfo in shooterHubs }
            shooter.startVoltagePolling()
        } catch (failure: Throwable) {
            throw safety.stop(failure)!!
        }
    }

    fun start() {
        shooter.startBlockerControl()
        pinpoint.setPose(Pose.zero())
        location.reset()
    }

    /** getBulkData refreshes the MANUAL cache once per loop, then sensors read cached values. */
    fun read() {
        feedbackHubs.forEach { it.bulkData }
        shooter.beginFrame()
        pinpoint.update()
        location.capture(pinpoint.state())
    }

    fun write() {
        drive.commitOutputs()
        shooter.commitOutputs()
        intake.commitOutputs()
    }

    override fun close() { safety.stop(null)?.let { throw it } }
}

/** Retain devices before setup writes, so even failed constructors can stop them. */
class SafeHardware {
    private val stops = mutableListOf<Runnable>()
    private val cacheModes = linkedMapOf<LynxModule, LynxModule.BulkCachingMode>()

    fun prepareMotors(map: HardwareMap, mechanisms: Boolean) {
        val names = if (mechanisms) DRIVE_NAMES +
            listOf("shooter1", "shooter2", "intake1", "intake2") else DRIVE_NAMES
        var failure: Throwable? = null
        for (name in names) failure = attempt(failure) {
            motor(map, name).apply { mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER; power = 0.0 }
        }
        rethrow(failure)
    }

    fun motor(map: HardwareMap, name: String): DcMotorEx =
        map.get(DcMotorEx::class.java, name).also {
            stops += Runnable { it.power = 0.0 }
        }

    fun onStop(action: Runnable) { stops.add(0, action) }

    fun setCaching(hubs: List<LynxModule>, mode: LynxModule.BulkCachingMode) {
        for (hub in hubs) {
            cacheModes.putIfAbsent(hub, hub.bulkCachingMode)
            hub.bulkCachingMode = mode
        }
    }

    fun stop(cause: Throwable?): Throwable? {
        var failure = cause
        for (action in stops) failure = attempt(failure, action)
        for ((hub, mode) in cacheModes) failure = attempt(failure) { hub.bulkCachingMode = mode }
        return failure
    }

    companion object {
        internal val DRIVE_NAMES = listOf("driveFL", "driveFR", "driveBL", "driveBR")
        @JvmStatic fun attempt(first: Throwable?, action: Runnable): Throwable? {
            try { action.run() } catch (next: RuntimeException) { return combine(first, next) }
            catch (next: LinkageError) { return combine(first, next) }
            return first
        }
        private fun combine(first: Throwable?, next: Throwable): Throwable {
            if (first == null) return next
            if (first !== next) first.addSuppressed(next)
            return first
        }
        @JvmStatic fun rethrow(failure: Throwable?) { failure?.let { throw it } }
    }
}

/** Normal commits fail fast; forced shutdown attempts every device before rethrowing. */
internal inline fun outputWrites(force: Boolean, count: Int, write: (Int) -> Unit) {
    var failure: Throwable? = null
    for (index in 0 until count) {
        try { write(index) } catch (t: Throwable) {
            if (!force) throw t
            if (failure == null) failure = t else if (failure !== t) failure.addSuppressed(t)
        }
    }
    failure?.let { throw it }
}
