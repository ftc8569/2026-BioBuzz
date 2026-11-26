package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ShooterController
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.configurables.annotations.Configurable
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.RobotLog
import java.util.Locale
import org.firstinspires.ftc.robotcore.external.Telemetry

// Telemetry and diagnostic OpModes. Called by the shared Robot loop.

/**
 * Driver Station / Panels refresh period. People cannot read numbers faster than this, and
 * every publish costs loop time (string formatting plus a network send), so control loops
 * run many times between refreshes. The SDK's own default is 250 ms.
 */
const val TELEMETRY_INTERVAL_MS = 100L

/** Interval between the same point in consecutive loops, including all loop work. */
internal class LoopTiming(private val nanoTime: () -> Long = System::nanoTime) {
    private var previousNanos: Long? = null
    var intervalMs = 0.0
        private set
    /** Smoothed loop rate, so the telemetry row is readable at 10 refreshes per second. */
    var averageHz = 0.0
        private set

    fun reset() {
        previousNanos = null
        intervalMs = 0.0
        averageHz = 0.0
    }

    fun update() {
        val now = nanoTime()
        intervalMs = previousNanos?.let { (now - it).coerceAtLeast(0L) / 1_000_000.0 } ?: 0.0
        previousNanos = now
        if (intervalMs > 0.0) {
            val hz = 1000.0 / intervalMs
            averageHz = if (averageHz == 0.0) hz else averageHz + 0.05 * (hz - averageHz)
        }
    }
}

/** Publish at most once per [TELEMETRY_INTERVAL_MS]; skipped loops do no formatting. */
class TelemetryRefresh @JvmOverloads constructor(
    private val nanoTime: () -> Long = System::nanoTime
) {
    private var lastRefresh: Long? = null

    fun reset() { lastRefresh = null }

    fun isDue(): Boolean {
        val now = nanoTime()
        val previous = lastRefresh
        if (previous != null && now - previous < TELEMETRY_INTERVAL_MS * 1_000_000L) return false
        lastRefresh = now
        return true
    }
}

/** One display clock and one formatting pass for both Driver Station and Panels. */
internal class RobotTelemetry(
    driverStation: Telemetry,
    panels: Telemetry,
    nanoTime: () -> Long = System::nanoTime
) {
    private val refresh = TelemetryRefresh(nanoTime)
    private val joined = JoinedTelemetry(driverStation, panels).apply { msTransmissionInterval = TELEMETRY_INTERVAL_MS.toInt() }

    fun addData(caption: String, value: Any?) { joined.addData(caption, value) }
    fun addData(caption: String, format: String, vararg values: Any?) {
        // Any selects the value overload, avoiding a second formatting pass per destination.
        val text: Any = if (values.isEmpty()) format else String.format(Locale.US, format, *values)
        joined.addData(caption, text)
    }
    fun publish(build: RobotTelemetry.() -> Unit) {
        if (!refresh.isDue()) return
        build()
        joined.update()
    }
    fun reset() { refresh.reset() }
    fun clear() { refresh.reset(); joined.clearAll() }
}

/** Display cached shooter state without reading hardware or changing controller state. */
internal fun RobotTelemetry.reportShooter(shooter: ShooterSubsystem, controls: ShooterControls) {
    val shot = shooter.control
    addData("Mode", if (controls.mode == ShooterController.ControlMode.MANUAL_POWER) "Manual" else "LQR")
    addData("Preset", "%.3f", controls.preset)
    val target = controls.velocityTargetRpm
    if (target == null) addData("Target RPM", "Manual power")
    else addData("Target RPM", "%.0f", target)
    addData("RPM 1 / 2", "%.0f / %.0f", shooter.wheels[0].rpm, shooter.wheels[1].rpm)
    addData("Motor power", "%.2f", shooter.flywheelPower)
    addData("Feed", shot.feedReadiness)
}

/** Tests the blocker servo without running the shooter or intake. Remove game pieces first. */
@TeleOp(name = "Blocker Diagnostic", group = "Test")
class BlockerDiagnosticTeleOp : OpMode() {
    private lateinit var blocker: Servo
    private var started = false

    // main: no servo movement during INIT; START closes the blocker.
    override fun init() { blocker = hardwareMap.get(Servo::class.java, "blocker") }
    override fun start() { started = true; blocker.position = CLOSED }

    override fun loop() {
        when {
            gamepad1.a -> blocker.position = CLOSED
            gamepad1.b -> blocker.position = OPEN
        }
        telemetry.addData("A: close / B: open", "%.2f", blocker.position)
        telemetry.update()
    }

    override fun stop() {
        if (started) blocker.position = CLOSED
        started = false
    }

    companion object {
        private const val CLOSED = ShooterController.BLOCKER_CLOSED
        private const val OPEN = ShooterController.BLOCKER_OPEN
    }
}

/** Direct motor test: no PID, intake, or automatic blocker movement. Clear the wheels first. */
@TeleOp(name = "Shooter Motor Diagnostic", group = "Test")
class ShooterMotorDiagnosticTeleOp : OpMode() {
    private lateinit var shooter1: DcMotorEx
    private lateinit var shooter2: DcMotorEx
    private lateinit var blocker: Servo
    private val safety = SafeHardware()
    private var initialPosition1 = 0
    private var initialPosition2 = 0
    private var started = false

    override fun init() = guarded {
        val hubs = hardwareMap.getAll(LynxModule::class.java)
        safety.setCaching(hubs, LynxModule.BulkCachingMode.OFF)
        hubs.forEach { it.clearBulkCache() }
        blocker = hardwareMap.get(Servo::class.java, "blocker") // Untouched until START.
        shooter1 = safety.motor(hardwareMap, "shooter1")
        shooter2 = safety.motor(hardwareMap, "shooter2")
        // Reverse both motor polarities while preserving logical forward/reverse controls.
        shooter1.direction = DcMotorSimple.Direction.FORWARD
        shooter2.direction = DcMotorSimple.Direction.REVERSE
        shooter1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        shooter2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        shooter1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        shooter2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        shooter1.power = 0.0
        shooter2.power = 0.0
        initialPosition1 = shooter1.currentPosition
        initialPosition2 = shooter2.currentPosition
    }

    override fun start() = guarded {
        started = true
        safety.onStop { blocker.position = ShooterController.BLOCKER_CLOSED }
        blocker.position = ShooterController.BLOCKER_CLOSED
    }

    override fun loop() = guarded {
        if (!started) return@guarded
        // Hold X or Y for one motor, B for both. Right trigger raises power to 0.80.
        val power = 0.25 + 0.55 * gamepad1.right_trigger.toDouble().coerceIn(0.0, 1.0)
        shooter1.power = if (gamepad1.x || gamepad1.b) power else 0.0
        shooter2.power = if (gamepad1.y || gamepad1.b) power else 0.0
        blocker.position = ShooterController.BLOCKER_CLOSED

        telemetry.addLine("Hold X: motor 1 | Y: motor 2 | B: both | RT: raise power")
        telemetry.addData("Power 1 / 2", "%.2f / %.2f", shooter1.power, shooter2.power)
        telemetry.addData("Raw encoder ticks/s 1 / 2", "%.0f / %.0f",
            shooter1.velocity, shooter2.velocity)
        val position1 = shooter1.currentPosition
        val position2 = shooter2.currentPosition
        telemetry.addData("Encoder position ticks 1 / 2", "%d / %d", position1, position2)
        telemetry.addData("Ticks moved since init 1 / 2", "%d / %d",
            position1.toLong() - initialPosition1, position2.toLong() - initialPosition2)
        telemetry.addData("Motor 1 / 2 connection", "%s / %s",
            shooter1.connectionInfo, shooter2.connectionInfo)
        telemetry.addData("Motor 1 / 2 profile", "%s / %s",
            shooter1.motorType.name, shooter2.motorType.name)
        telemetry.addLine("Bulk cache OFF: fresh reads. Spinning + zero counts: check encoder cables/ports.")
        telemetry.addData("Configured ticks/rev 1 / 2", "%.0f / %.0f",
            shooter1.motorType.ticksPerRev, shooter2.motorType.ticksPerRev)
        telemetry.addData("Configured max ticks/s 1 / 2", "%.0f / %.0f",
            shooter1.motorType.achieveableMaxTicksPerSecond,
            shooter2.motorType.achieveableMaxTicksPerSecond)
        telemetry.update()
    }

    private inline fun guarded(action: () -> Unit) {
        try { action() } catch (failure: Throwable) {
            throw safety.stop(failure)!!
        }
    }

    override fun stop() { safety.stop(null)?.let { throw it } }
}

/** Optional camera check. It starts only when this diagnostic is explicitly selected and started. */
@Configurable
@TeleOp(name = "Vision Diagnostic", group = "Test")
class VisionDiagnosticTeleOp : OpMode() {
    private var vision: VisionSubsystem? = null
    private val refresh = TelemetryRefresh()
    private var failure: String? = null

    override fun init() = guarded("INIT camera") {
        vision = VisionSubsystem.optional(hardwareMap)
        telemetry.msTransmissionInterval = TELEMETRY_INTERVAL_MS.toInt()
        telemetry.addData("Vision status", if (vision == null) "No configured limelight" else "Ready; START begins camera")
        telemetry.update()
    }
    override fun start() = guarded("START camera") { vision?.start(pipelineIndex) }
    override fun loop() = guarded("LOOP camera") {
        val camera = vision
        val frame = camera?.update()
        if (refresh.isDue()) {
            telemetry.addData("Vision status", (frame?.status ?: "No configured limelight") as Any)
            telemetry.addData("Visible tags", camera?.getVisibleTagIds() ?: emptyList<Int>())
            telemetry.addData("Frame age (ms)", frame?.ageMillis ?: 0)
            frame?.estimate?.let {
                telemetry.addData("Hive robot pose (m / rad)", it.robot)
                telemetry.addData("Hive fit residual (m)", it.residualMeters)
            }
            telemetry.update()
        }
    }
    override fun stop() {
        val camera = vision
        vision = null
        camera?.stop()
    }
    private inline fun guarded(phase: String, action: () -> Unit) {
        failure?.let {
            if (refresh.isDue()) { telemetry.addData("Vision error", it as Any); telemetry.update() }
            return
        }
        try { action() } catch (error: RuntimeException) {
            try { stop() } catch (cleanup: RuntimeException) { if (error !== cleanup) error.addSuppressed(cleanup) }
            failure = "$phase: ${error.message}"
            RobotLog.ee("BioBuzzVision", error, failure)
            telemetry.addData("Vision error", failure as Any?)
            telemetry.update()
        }
    }
    companion object { @JvmField var pipelineIndex = 0 }
}

/** Uses the match shooter and controls for tuning; only flywheels and blocker are required.
 * INIT stays stopped. No intake, drive, odometry or camera is created by this OpMode.
 */
@TeleOp(name = "Shooter Tuning", group = "Test")
class ShooterTuningTeleOp : OpMode() {
    private val nanoTime: () -> Long = System::nanoTime
    private val safety = SafeHardware()
    private val controls = ShooterControls()
    private val errorRefresh = TelemetryRefresh(nanoTime)
    private var hubs = emptyList<LynxModule>()
    private var shooter: ShooterSubsystem? = null
    private var display: RobotTelemetry? = null
    private var ready = false
    private var started = false
    private var failure: String? = null
    private var requestedPower = 0.0
    private val up: Boolean get() = gamepad1.dpad_up || gamepad2.dpad_up
    private val down: Boolean get() = gamepad1.dpad_down || gamepad2.dpad_down
    private val manualBlocker: Boolean get() = gamepad1.b || gamepad2.b

    override fun init() = guarded("INIT") {
        hubs = hardwareMap.getAll(LynxModule::class.java)
        safety.setCaching(hubs, LynxModule.BulkCachingMode.MANUAL)
        hubs.forEach { it.clearBulkCache() }
        // Acquire independent stop handles before the subsystem can partially initialize.
        var error: Throwable? = null
        for (name in listOf("shooter1", "shooter2")) error = SafeHardware.attempt(error) {
            safety.motor(hardwareMap, name).power = 0.0
        }
        SafeHardware.rethrow(error)
        shooter = ShooterSubsystem(hardwareMap, nanoTime).also {
            safety.onStop { it.forceStop() }
            safety.onStop { it.stopVoltagePolling() }
            it.enablePhasedIo()
        }
        display = RobotTelemetry(telemetry, com.bylazar.telemetry.PanelsTelemetry.ftcTelemetry, nanoTime)
        ready = true
        report()
    }

    override fun init_loop() = guarded("INIT") {
        controls.update(false, up, down) // Preset can be adjusted while outputs remain zero.
        report()
    }
    override fun start() = guarded("START") {
        if (!ready) return@guarded
        checkNotNull(shooter).apply { startBlockerControl(); startVoltagePolling() }
        started = true
    }
    override fun loop() = guarded("LOOP") {
        if (!ready || !started) return@guarded
        val device = checkNotNull(shooter)
        hubs.forEach { it.clearBulkCache() } // One owner and one invalidation per active loop.
        device.beginFrame()
        controls.update(gamepad1.y, up, down)
        requestedPower = controls.requestedPower(gamepad1.left_bumper, gamepad1.right_bumper)
        device.setControlMode(controls.mode)
        if (requestedPower > 0.0) device.updateShot(requestedPower, controls.mode)
        else device.setFlywheelPower(requestedPower)
        device.setManualBlockerOpen(manualBlocker)
        device.commitOutputs()
        report()
    }
    private fun report() {
        val device = shooter ?: return
        display?.publish {
            addData("Shooter tuning", if (started) "ACTIVE" else "READY: outputs stopped")
            addData("Controls", "Hold RB: LQR | Y: manual | LB: reverse | D-pad: preset | B: manual blocker")
            addData("Live LQR settings", "Panels > ShooterLqrTuning")
            reportShooter(device, controls)
        }
    }
    override fun stop() {
        ready = false
        started = false
        SafeHardware.rethrow(safety.stop(null))
    }
    private inline fun guarded(phase: String, action: () -> Unit) {
        if (failure == null) {
            try { action() } catch (error: Throwable) {
                ready = false
                started = false
                safety.stop(error)
                failure = "$phase: ${error.javaClass.simpleName}: ${error.message}"
                RobotLog.ee("BioBuzzShooterTuning", error, failure)
                errorRefresh.reset()
            }
        }
        failure?.let {
            if (errorRefresh.isDue()) { telemetry.addData("Shooter tuning error", it as Any); telemetry.update() }
        }
    }
}
