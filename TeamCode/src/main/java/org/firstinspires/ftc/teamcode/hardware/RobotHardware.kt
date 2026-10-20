package org.firstinspires.ftc.teamcode.hardware

import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.drivetrain.Drivetrain
import com.pedropathing.follower.Follower
import com.pedropathing.localization.Localizer
import com.pedropathing.localization.MotionState
import com.pedropathing.math.Pose
import com.pedropathing.revhub.localizers.PinpointLocalizer
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.*
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import org.firstinspires.ftc.teamcode.control.ShooterController
import org.firstinspires.ftc.teamcode.diagnostics.LoopProfiler
import org.firstinspires.ftc.teamcode.diagnostics.LoopStage
import org.firstinspires.ftc.teamcode.diagnostics.measureOrRun
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.subsystems.*

/** Construct once during INIT. Controllers use these objects for the entire match. */
internal class RobotHardware(
    map: HardwareMap,
    val profiler: LoopProfiler = LoopProfiler(),
    nanoTime: () -> Long = System::nanoTime
) : AutoCloseable {
    private val safety = SafeHardware()
    val drive: DriveSubsystem
    val intake: IntakeSubsystem
    val shooter: ShooterSubsystem
    val hold: ShootingDriveHold
    private val pinpoint: PinpointLocalizer
    private val snapshot = SnapshotLocalizer()
    private val driveMotors: List<DcMotor>
    private val inputs: BulkInputReader
    val pose: Pose get() = snapshot.pose()
    var encoderPositions: IntArray? = null
        private set

    init {
        try {
            safety.closeBlocker(map, ShooterController.BLOCKER_CLOSED)
            safety.prepareMotors(map, true)
            drive = DriveSubsystem(map).apply { enablePhasedIo() }
            intake = IntakeSubsystem(map).apply { enablePhasedIo() }
            shooter = ShooterSubsystem(map, nanoTime).apply {
                loopProfiler = profiler
                enablePhasedIo()
            }
            safety.onStop { shooter.stopVoltagePolling() }
            pinpoint = PinpointLocalizer(map, Constants.localizerConfig)
            val drivetrain = BufferedMecanum(drive::requestWheelPowers)
            hold = ShootingDriveHold(Constants.create(snapshot, drivetrain))
            safety.onStop { hold.stop() }
            driveMotors = SafeHardware.DRIVE_NAMES.map { map.get(DcMotor::class.java, it) }
            val hubs = map.getAll(LynxModule::class.java)
            safety.setCaching(hubs, LynxModule.BulkCachingMode.MANUAL)
            inputs = BulkInputReader(hubs,
                listOf("shooter1", "shooter2").map { map.get(DcMotor::class.java, it) },
                driveMotors)
            shooter.startVoltagePolling()
        } catch (failure: Throwable) {
            throw safety.stop(failure)!!
        }
    }

    fun start() {
        profiler.measure(LoopStage.LOCATION_RESET) { pinpoint.setPose(Pose.zero()) }
    }

    fun read(diagnostics: Boolean) = profiler.measure(LoopStage.READ_INPUTS) {
        profiler.measure(LoopStage.BULK_READ) { inputs.read(diagnostics) }
        shooter.beginFrame()
        encoderPositions = if (diagnostics) profiler.measure(LoopStage.DRIVE_ENCODERS) {
            driveMotors.map { it.currentPosition }.toIntArray()
        } else null
        profiler.measure(LoopStage.PINPOINT) {
            pinpoint.update()
            snapshot.capture(pinpoint.state())
        }
    }

    fun write() = profiler.measure(LoopStage.WRITE_OUTPUTS) {
        profiler.measure(LoopStage.DRIVE_OUTPUT) { drive.commitOutputs() }
        shooter.commitOutputs()
        profiler.measure(LoopStage.INTAKE_OUTPUT) { intake.commitOutputs() }
    }

    override fun close() { safety.stop(null)?.let { throw it } }
}

/** Resolve hub ownership once; getBulkData refreshes MANUAL caches once per active read. */
internal class BulkInputReader(
    hubs: List<LynxModule>, feedback: List<DcMotor>, diagnostics: List<DcMotor>
) {
    private val feedbackConnections = feedback.map { it.controller.connectionInfo }.toSet()
    private val diagnosticConnections = diagnostics.map { it.controller.connectionInfo }.toSet()
    private val feedbackHubs = hubs.filter { it.connectionInfo in feedbackConnections }
    private val diagnosticHubs = hubs.filter {
        it !in feedbackHubs && it.connectionInfo in diagnosticConnections
    }

    fun read(includeDiagnostics: Boolean) {
        feedbackHubs.forEach { it.bulkData }
        if (includeDiagnostics) diagnosticHubs.forEach { it.bulkData }
    }
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

    fun closeBlocker(map: HardwareMap, position: Double): Servo =
        map.get(Servo::class.java, "blocker").also {
            stops += Runnable { it.position = position }
            it.position = position
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
internal fun outputWrites(force: Boolean, count: Int, write: (Int) -> Unit) {
    var failure: Throwable? = null
    for (index in 0 until count) {
        try { write(index) } catch (t: Throwable) {
            if (!force) throw t
            if (failure == null) failure = t else if (failure !== t) failure.addSuppressed(t)
        }
    }
    failure?.let { throw it }
}

/** Pedro 3.0.1 Mecanum math, with hardware writes delegated to the shared output owner. */
internal class BufferedMecanum(private val submit: (DoubleArray, Double) -> Unit) : Drivetrain {
    private var lastPowers = DoubleArray(4)
    private fun wheels(p: DrivePowers) = doubleArrayOf(
        p.forward() - p.strafe() - p.turn(), p.forward() + p.strafe() + p.turn(),
        p.forward() + p.strafe() - p.turn(), p.forward() - p.strafe() + p.turn())

    override fun drive(powers: DrivePowers, manual: Boolean) {
        val raw = wheels(powers)
        val scale = maxOf(1.0, raw.maxOf { abs(it) })
        lastPowers = raw.map { it / scale }.toDoubleArray()
        submit(lastPowers, Constants.drivetrainConfig.powerThreshold.get())
    }
    override fun maxScaling(current: DrivePowers, delta: DrivePowers): Double {
        val a = wheels(current)
        val b = wheels(delta)
        var scale = 1.0
        for (i in a.indices) {
            if (abs(b[i]) < 1e-9) continue
            for (t in doubleArrayOf((1.0-a[i])/b[i], (-1.0-a[i])/b[i])) {
                if (t >= 0.0 && t < scale) scale = t
            }
        }
        return scale.coerceIn(0.0, 1.0)
    }
    override fun stop() = stop(false)
    override fun stop(brake: Boolean) {
        lastPowers = DoubleArray(4)
        submit(lastPowers, 0.0)
    }
    override fun debug(): Map<String, Any> = mapOf("wheelPowers" to lastPowers.toList())
    override fun interpolateVelocity(xRadius: Double, yRadius: Double, theta: Double): Double =
        1.0 / (abs(cos(theta))/xRadius + abs(sin(theta))/yRadius)
}

/** Pedro sees the same immutable pose/velocity sample as every other controller. */
internal class SnapshotLocalizer : Localizer {
    private var snapshot = MotionState.zero()
    fun capture(state: MotionState) { snapshot = state }
    override fun state(): MotionState = snapshot
    override fun update() = Unit
    override fun setPose(pose: Pose) { snapshot = snapshot.withPose(pose) }
    override fun reset() { snapshot = MotionState.zero() }
}

/** Capture once per shot; joystick drive resumes immediately when shooting stops. */
internal class ShootingDriveHold(private val follower: Follower) {
    var active = false
        private set

    fun update(shooting: Boolean, pose: Pose): Boolean {
        if (!shooting) {
            if (active) stop()
            return false
        }
        if (!active) {
            follower.hold(pose)
            active = true
        }
        follower.update()
        return true
    }

    fun stop() {
        follower.stop()
        active = false
    }
}

/** Hardware reads stay on one worker; control loops only read the volatile snapshot. */
internal class BatteryVoltageMonitor(private val sensors: List<VoltageSensor>) : AutoCloseable {
    @Volatile var voltage = 12.0
        private set
    @Volatile internal var profiler: LoopProfiler? = null
    private var executor: ScheduledExecutorService? = null
    private var closed = false

    @Synchronized fun start() {
        if (closed || executor != null || sensors.isEmpty()) return
        executor = Executors.newSingleThreadScheduledExecutor { task ->
            Thread(task, "BioBuzz battery voltage").apply { isDaemon = true }
        }.also {
            it.scheduleWithFixedDelay({ pollOnce() }, 0, 250, TimeUnit.MILLISECONDS)
        }
    }

    internal fun pollOnce() {
        profiler.measureOrRun(LoopStage.VOLTAGE_POLL) { readVoltage() }
    }

    private fun readVoltage() {
        var minimum = Double.POSITIVE_INFINITY
        for (sensor in sensors) {
            if (Thread.currentThread().isInterrupted) return
            val sample = try {
                sensor.voltage
            } catch (_: RuntimeException) {
                // A failed hub must not prevent future polls or reading the other hub.
                continue
            }
            if (sample.isFinite() && sample > 0.0) minimum = minOf(minimum, sample)
        }
        voltage = if (minimum.isFinite()) minimum else 12.0
    }

    @Synchronized override fun close() {
        closed = true
        executor?.shutdownNow()
        executor = null
    }
}

/** Cached motor feedback and its configured conversion; telemetry performs no hardware reads. */
class FlywheelFeedback(val ticksPerRev: Double, val maxTicksPerSecond: Double) {
    var position = 0
        private set
    var ticksPerSecond = 0.0
        private set
    val valid: Boolean get() = ticksPerRev.isFinite() && ticksPerRev > 0.0
    val rpm: Double get() = if (valid) ticksPerSecond * 60.0 / ticksPerRev else Double.NaN

    fun sample(position: Int, velocity: Double) {
        this.position = position
        ticksPerSecond = abs(velocity)
    }

    fun targetTicksPerSecond(rpm: Double): Double = if (valid) rpm * ticksPerRev / 60.0 else 0.0
}
