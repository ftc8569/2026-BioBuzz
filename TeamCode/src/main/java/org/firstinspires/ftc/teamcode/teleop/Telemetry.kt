package org.firstinspires.ftc.teamcode.teleop

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.configurables.annotations.Configurable
import org.firstinspires.ftc.teamcode.vision.VisionSubsystem
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

internal enum class LoopStage(val label: String) {
    LOOP_INTERVAL("Loop interval"),
    LOOP_WORK("Loop work total"),
    READ_INPUTS("Read inputs phase"),
    BULK_READ("Hub bulk read transactions"),
    COMPUTE("Control compute phase"),
    WRITE_OUTPUTS("Write outputs phase"),
    DRIVE_TOTAL("Drive total"),
    DRIVE_ENCODERS("Drive encoder reads"),
    PINPOINT("Pinpoint location read"),
    LOCATION_RESET("Pinpoint initial pose write"),
    HIVE_SAVE("Hive save (cached pose)"),
    DRIVE_HOLD("Pedro hold update"),
    DRIVE_OUTPUT("Drive motor output commit"),
    DRIVE_TELEMETRY("Drive telemetry build"),
    MECHANISMS_TOTAL("Mechanisms total"),
    SHOOTER_CONTROL("Shooter control total"),
    SHOOTER_ENCODERS("Shooter encoder and velocity reads"),
    SHOOTER_LQR("Shooter LQR calculation"),
    SHOOTER_MODES("Shooter motor mode writes"),
    SHOOTER_OUTPUT("Shooter motor output commit"),
    BLOCKER_OUTPUT("Blocker servo output commit"),
    INTAKE_OUTPUT("Intake motor writes"),
    TELEMETRY_BUILD("Main telemetry build and getters"),
    TELEMETRY_UPDATE("Telemetry update both outputs"),
    DS_TELEMETRY_UPDATE("Driver Station telemetry update"),
    PANELS_TELEMETRY_UPDATE("Panels telemetry update"),
    TIMING_REPORT("Timing report overhead"),
    VOLTAGE_POLL("Async voltage sensor poll (background)")
}

internal data class TimingSample(
    val stage: LoopStage,
    val calls: Long,
    val averageMs: Double,
    val maxMs: Double
)

/** Locks protect counters only, never hardware calls or report formatting. */
internal class LoopProfiler(@PublishedApi internal val nanoTime: () -> Long = System::nanoTime) {
    private val counts = LongArray(LoopStage.entries.size)
    private val totals = LongArray(counts.size)
    private val maxima = LongArray(counts.size)
    private var lastReportNanos = nanoTime()
    private var previousLoopStart: Long? = null
    private var loopStart: Long? = null

    inline fun <T> measure(stage: LoopStage, block: () -> T): T {
        val start = nanoTime()
        try {
            return block()
        } finally {
            record(stage, nanoTime() - start)
        }
    }

    @PublishedApi
    @Synchronized internal fun record(stage: LoopStage, elapsedNanos: Long) {
        val i = stage.ordinal
        val duration = elapsedNanos.coerceAtLeast(0L)
        counts[i]++
        totals[i] += duration
        maxima[i] = maxOf(maxima[i], duration)
    }

    fun beginLoop() {
        val now = nanoTime()
        previousLoopStart?.let { record(LoopStage.LOOP_INTERVAL, now - it) }
        previousLoopStart = now
        loopStart = now
    }

    fun endLoop() {
        loopStart?.let { record(LoopStage.LOOP_WORK, nanoTime() - it) }
        loopStart = null
    }

    @Synchronized fun reset() {
        counts.fill(0)
        totals.fill(0)
        maxima.fill(0)
        previousLoopStart = null
        loopStart = null
        lastReportNanos = nanoTime()
    }

    /** Drain once per second. Uncalled stages are absent, not reported as zero-cost calls. */
    @Synchronized fun snapshotIfDue(): List<TimingSample>? {
        val now = nanoTime()
        if (now - lastReportNanos < 1_000_000_000L) return null
        val samples = LoopStage.entries.mapNotNull { stage ->
            val i = stage.ordinal
            if (counts[i] == 0L) null else TimingSample(
                stage, counts[i], totals[i].toDouble() / counts[i] / 1_000_000.0,
                maxima[i] / 1_000_000.0
            )
        }
        counts.fill(0)
        totals.fill(0)
        maxima.fill(0)
        lastReportNanos = now
        return samples
    }
}

/** Interval between the same point in consecutive TeleOp loops, including all loop work. */
internal class LoopTiming(private val nanoTime: () -> Long = System::nanoTime) {
    private var previousNanos: Long? = null
    var intervalMs = 0.0
        private set
    val frequencyHz: Double get() = if (intervalMs > 0.0) 1000.0 / intervalMs else 0.0

    fun reset() {
        previousNanos = null
        intervalMs = 0.0
    }

    fun update() {
        val now = nanoTime()
        intervalMs = previousNanos?.let { (now - it).coerceAtLeast(0L) / 1_000_000.0 } ?: 0.0
        previousNanos = now
    }
}

/** Match the 50 ms display interval without formatting or publishing on every control tick. */
class TelemetryRefresh @JvmOverloads constructor(
    private val nanoTime: () -> Long = System::nanoTime
) {
    private var lastRefresh: Long? = null

    fun reset() { lastRefresh = null }

    fun isDue(): Boolean {
        val now = nanoTime()
        val previous = lastRefresh
        if (previous != null && now - previous < 50_000_000L) return false
        lastRefresh = now
        return true
    }
}

/** One display clock and one formatting pass for both Driver Station and Panels. */
internal class RobotTelemetry(
    driverStation: Telemetry,
    panels: Telemetry,
    private val profiler: LoopProfiler,
    nanoTime: () -> Long = System::nanoTime
) {
    private val refresh = TelemetryRefresh(nanoTime)
    private val joined = JoinedTelemetry(
        timed(driverStation, LoopStage.DS_TELEMETRY_UPDATE),
        timed(panels, LoopStage.PANELS_TELEMETRY_UPDATE)
    ).apply { msTransmissionInterval = 50 }
    private var timingRows: List<Pair<String, String>> = emptyList()

    fun addData(caption: String, value: Any?) { joined.addData(caption, value) }
    fun addData(caption: String, format: String, vararg values: Any?) {
        // Any selects the value overload, avoiding a second formatting pass per destination.
        val text: Any = if (values.isEmpty()) format else String.format(Locale.US, format, *values)
        joined.addData(caption, text)
    }
    fun publish(build: RobotTelemetry.() -> Unit) {
        if (!refresh.isDue()) return
        profiler.measure(LoopStage.TELEMETRY_BUILD) { build() }
        profiler.measure(LoopStage.TELEMETRY_UPDATE) { joined.update() }
    }
    fun reset() { refresh.reset(); timingRows = emptyList() }
    fun clear() { refresh.reset(); joined.clearAll() }

    fun reportTimings() = profiler.measure(LoopStage.TIMING_REPORT) {
        profiler.snapshotIfDue()?.let { samples ->
            val byStage = samples.associateBy { it.stage }
            timingRows = LoopStage.entries.map { stage ->
                val sample = byStage[stage]
                "Timing: ${stage.label}" to if (sample == null) "not called in this window" else
                    String.format(Locale.US, "avg=%.3f ms max=%.3f ms n=%d",
                        sample.averageMs, sample.maxMs, sample.calls)
            }
            val report = timingRows.joinToString("; ") { "${it.first}: ${it.second}" }
            RobotLog.ii("BioBuzzLoop", report)
        }
        for ((label, value) in timingRows) joined.addData(label, value as Any)
    }

    private fun timed(delegate: Telemetry, stage: LoopStage): Telemetry =
        object : Telemetry by delegate {
            override fun update(): Boolean = profiler.measure(stage) { delegate.update() }
        }
}

internal inline fun <T> LoopProfiler?.measureOrRun(stage: LoopStage, block: () -> T): T =
    if (this == null) block() else measure(stage, block)

/** Display cached shooter state without reading hardware or changing controller state. */
internal fun RobotTelemetry.reportShooter(
    shooter: ShooterSubsystem,
    controls: ShooterControls,
    requestedPower: Double,
    manualBlocker: Boolean,
    up: Boolean,
    down: Boolean
) {
    val shot = shooter.control
    val (first, second) = shooter.wheels
    addData("LQR interval (ms)", "%.2f", shot.lqrIntervalMs)
    addData("LQR frequency (Hz)", "%.1f", shot.lqrFrequencyHz)
    shooter.wheels.forEachIndexed { i, wheel ->
        addData("Shooter flywheel ${i + 1} RPM", "%.0f", wheel.rpm)
    }
    val selectedVelocityTarget = controls.velocityTargetRpm
    if (selectedVelocityTarget != null) {
        addData("LQR target RPM", "%.0f", selectedVelocityTarget)
    } else {
        addData("LQR target RPM", "Manual mode: no RPM target")
    }
    addData("LQR shooting active", selectedVelocityTarget != null && requestedPower > 0.0)
    addData("LQR commanded RPM", "%.0f", shot.targetRpm)
    addData("LQR model and cost tolerances", shot.lqrModelRead)
    addData("LQR feedback gain (V/RPM)", shot.lqrFeedbackGain)
    addData("LQR requested voltage (V)", shot.lqrRequestedVolts)
    addData("LQR feedback", "Flywheel 1 RPM (shared motor power)")
    addData("Shooter preset (G1/G2 up/down)", "%.3f", controls.preset)
    addData("Shooter mode (hold G1 Y for manual)", controls.mode)
    addData("Shooter speed state", shot.speedStatus)
    addData("D-pad seen (up/down)", "%s / %s", up, down)
    addData("Shooter requested fraction", "%.2f", requestedPower)
    addData("Shooter motor power 1 / 2", "%.2f / %.2f",
        shooter.flywheelPower, shooter.flywheelPower)
    addData("Manual blocker (G1/G2 B)", manualBlocker)
    addData("Blocker open", shot.blockerOpen)
    addData("Flywheel 1 within 1 percent of target", shot.atSpeed)
    addData("Automatic feed ready", shot.feedReady)
    addData("Feed readiness", shot.feedReadiness)
    addData("Blocker status", shot.blockerStatus)
    addData("Blocker commanded position", "%.2f", shot.blockerPosition)
    addData("Shooter preset reference RPM", "%.0f", ShooterController.MAX_MOTOR_RPM)
    addData("RPM configuration valid", first.valid)
    addData("Raw ticks/s 1 / 2", "%.0f / %.0f",
        first.ticksPerSecond, second.ticksPerSecond)
    addData("Encoder position ticks 1 / 2", "%d / %d",
        first.position, second.position)
    addData("LQR target ticks/s 1 / 2", "%.0f / %.0f",
        first.targetTicksPerSecond(shot.targetRpm), second.targetTicksPerSecond(shot.targetRpm))
    addData("Motor config ticks/rev 1 / 2", "%.0f / %.0f",
        first.ticksPerRev, second.ticksPerRev)
    addData("Motor config max ticks/s 1 / 2", "%.0f / %.0f",
        first.maxTicksPerSecond, second.maxTicksPerSecond)
}

/** Tests the blocker servo without running the shooter or intake. Remove game pieces first. */
@TeleOp(name = "Blocker Diagnostic", group = "Test")
class BlockerDiagnosticTeleOp : OpMode() {
    private lateinit var blocker: Servo

    override fun init() {
        blocker = hardwareMap.get(Servo::class.java, "blocker")
        blocker.position = CLOSED
    }

    override fun loop() {
        when {
            gamepad1.a -> blocker.position = CLOSED
            gamepad1.b -> blocker.position = OPEN
        }
        telemetry.addData("A: close / B: open", "%.2f", blocker.position)
        telemetry.update()
    }

    override fun stop() {
        if (::blocker.isInitialized) blocker.position = CLOSED
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

    override fun init() = guarded {
        val hubs = hardwareMap.getAll(LynxModule::class.java)
        safety.setCaching(hubs, LynxModule.BulkCachingMode.OFF)
        hubs.forEach { it.clearBulkCache() }
        blocker = safety.closeBlocker(hardwareMap, ShooterController.BLOCKER_CLOSED)
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
        blocker.position = ShooterController.BLOCKER_CLOSED
    }

    override fun loop() = guarded {
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
        telemetry.addData("Motor 1 connection", shooter1.connectionInfo)
        telemetry.addData("Motor 2 connection", shooter2.connectionInfo)
        telemetry.addData("Motor 1 profile", shooter1.motorType.name)
        telemetry.addData("Motor 2 profile", shooter2.motorType.name)
        telemetry.addLine("Bulk cache OFF: fresh reads. " +
            "Spinning + zero counts: check encoder cables/ports.")
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
        telemetry.msTransmissionInterval = 50
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
class ShooterTuningTeleOp internal constructor(
    private val nanoTime: () -> Long,
    private val panels: () -> Telemetry
) : OpMode() {
    constructor() : this(System::nanoTime, { com.bylazar.telemetry.PanelsTelemetry.ftcTelemetry })
    private val safety = SafeHardware()
    private val controls = ShooterControls()
    private val profiler = LoopProfiler(nanoTime)
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
        var error = SafeHardware.attempt(null) {
            safety.closeBlocker(hardwareMap, ShooterController.BLOCKER_CLOSED)
        }
        for (name in listOf("shooter1", "shooter2")) error = SafeHardware.attempt(error) {
            safety.motor(hardwareMap, name).power = 0.0
        }
        SafeHardware.rethrow(error)
        shooter = ShooterSubsystem(hardwareMap, nanoTime).also {
            safety.onStop { it.forceStop() }
            safety.onStop { it.stopVoltagePolling() }
            it.loopProfiler = profiler
            it.enablePhasedIo()
        }
        display = RobotTelemetry(telemetry, panels(), profiler, nanoTime)
        ready = true
        report()
    }

    override fun init_loop() = guarded("INIT") {
        controls.update(false, up, down) // Preset can be adjusted while outputs remain zero.
        report()
    }
    override fun start() = guarded("START") {
        if (!ready) return@guarded
        checkNotNull(shooter).startVoltagePolling()
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
            reportShooter(device, controls, requestedPower, manualBlocker, up, down)
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
