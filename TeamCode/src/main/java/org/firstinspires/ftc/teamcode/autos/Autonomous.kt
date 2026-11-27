package org.firstinspires.ftc.teamcode.autos

import com.pedropathing.api.Paths
import com.pedropathing.api.PoseFactory
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.ftc.NextFTCOpMode
import com.qualcomm.robotcore.util.RobotLog
import java.util.function.LongSupplier
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.teleop.TelemetryRefresh
import org.firstinspires.ftc.teamcode.teleop.TELEMETRY_INTERVAL_MS
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.Companion.DEFAULT_SHOOTER_PRESET
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.Companion.MAX_MOTOR_RPM
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.Mode
import org.firstinspires.ftc.teamcode.teleop.SafeHardware
import org.firstinspires.ftc.teamcode.teleop.IntakeFeed
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem

/** Original match autos and the shared NextFTC lifecycle. Additional routes use this same base. */

/** Routes are authored for red; blue rotates them 180 degrees about the field center. */
internal fun alliancePose(blue: Boolean, x: Double, y: Double, heading: Double): Pose =
    if (blue) PoseFactory.degrees().of(141.5 - x, 141.5 - y, heading + 180.0)
    else PoseFactory.degrees().of(x, y, heading)

// region Basic autos: shoot for two seconds, then follow the exported parking line

@Autonomous(name = "BasicRed Auto")
open class BioBuzzAuto(clock: LongSupplier = LongSupplier(System::nanoTime)) : AutoOpMode(clock) {
    // main: Basic and preload autos feed at the shooter preset, not full power.
    private val feed = IntakeFeed(DEFAULT_SHOOTER_PRESET, clock::getAsLong)
    private var feedingStarted: Long? = null
    private var driving = false
    private lateinit var path: Path

    override fun startingPose(): Pose =
        PoseFactory.degrees().of(57.01798561151079, 9.221582733812928, 90.0)

    /** main / basicred.pp: start at 90 degrees, drive the line holding a 270-degree heading. */
    protected open fun buildPath(start: Pose): Path =
        Paths.line(start, PoseFactory.degrees().of(8.96906474820144, 97.8935251798561, 270.0))
            .constant(Math.toRadians(270.0))

    override fun buildRoutine(start: Pose) { path = buildPath(start) }
    override fun startRoutine() {
        driving = false
        feedingStarted = null
        feed.reset()
    }
    override fun runningStage() = if (driving) "DRIVING" else "INITIAL_SHOT"

    override fun updateRoutine() {
        operation = "LOOP Pedro / Pinpoint"
        follower.update()
        if (driving) {
            if (!follower.isBusy) finish()
            return
        }
        operation = "LOOP shooter / feed"
        val preset = DEFAULT_SHOOTER_PRESET
        shooter.update(preset, Mode.LQR)
        val power = feed.power(shooter, preset)
        intake.setPower(power)
        if (power <= 0.0) { feedingStarted = null; return }
        val now = nanoTime.asLong
        val start = feedingStarted ?: now.also { feedingStarted = it }
        // Count actual feeding time, after the RPM gate and blocker opening delay.
        if (now - start < 2_000_000_000L) return
        shooter.update(0.0, Mode.LQR)
        intake.setPower(0.0)
        operation = "LOOP begin parking path"
        follower.follow(path)
        driving = true
    }

    override fun reportTelemetry() {
        telemetry.addData("Shooter target RPM", shooter.targetRpm)
        telemetry.addData("RPM 1 / 2", "%.0f / %.0f", shooter.rpm[0], shooter.rpm[1])
        telemetry.addData("Shooter", shooter.status)
        super.reportTelemetry()
    }
}

/** Same two-second shot as BasicRed; coordinates come from basicblue.pp. */
@Autonomous(name = "BasicBlue Auto")
class BasicBlueAuto : BioBuzzAuto() {
    override fun startingPose(): Pose =
        PoseFactory.degrees().of(83.07841726618705, 132.19424460431654, 90.0)
    override fun buildPath(start: Pose): Path =
        Paths.line(start, PoseFactory.degrees().of(132.9597122302158, 41.497122302158274, 90.0))
            .constant(Math.toRadians(90.0))
}

// endregion

// region Collection autos: shoot, collect, return, shoot, park

@Autonomous(name = "BioBuzzRed1.5")
open class BioBuzzRed1_5(private val blue: Boolean = false) : AutoOpMode() {
    private lateinit var path1: Path
    private lateinit var path2: Path
    private lateinit var path3: Path
    private var routine: Command? = null

    private fun pose(x: Double, y: Double, heading: Double) = alliancePose(blue, x, y, heading)

    override fun startingPose(): Pose = pose(58.4934, 9.0389, -90.0)

    override fun buildRoutine(start: Pose) {
        val path1End = pose(10.7034, 8.5727, -165.0)
        val path2Start = pose(10.7034, 8.5727, -115.0)
        val path2End = pose(58.1696, 132.804, 90.0)
        follower.holdEnd.set(false)
        path1 = Paths.curve(start, pose(34.4053, 12.9376, 0.0), path1End).linear(start, path1End)
        path2 = Paths.curve(path2Start, pose(64.9361, 49.5595, 0.0),
            pose(-0.2438, 135.2562, 0.0), pose(63.3781, 115.8686, 0.0), path2End)
            .linear(path2Start, path2End)
        path3 = Paths.curve(path2End, pose(57.986, 119.3084, 0.0),
            pose(12.3128, 115.8693, 4.3906)).reverseTangent()
    }

    override fun startRoutine() {
        // NextFTC SequentialGroup, driven directly by updateRoutine() (not CommandManager), so a
        // command failure stays inside AutoOpMode's guarded shutdown. Retained before start()
        // so a failed START can still stop it.
        routine = SequentialGroup(
            ShootCommand(follower, shooter, intake), followPath(path1, 1.0),
            followPath(path2, 0.0), ShootCommand(follower, shooter, intake), followPath(path3, 0.0)
        )
        checkNotNull(routine).start()
    }

    /** Intake collection lasts only as long as its path; cancellation stops it too. */
    private fun followPath(path: Path, intakePower: Double): Command = LambdaCommand("FollowPath")
        .requires(follower, intake)
        .setStart { intake.setPower(intakePower); follower.follow(path) }
        .setIsDone { !follower.following() }
        .setStop { interrupted ->
            intake.setPower(0.0)
            if (interrupted) follower.stop()
        }

    override fun updateRoutine() {
        operation = "LOOP routine"
        routine?.update()
        operation = "LOOP Pedro / Pinpoint"
        follower.update()
        // Once the whole sequence finishes, release outputs and stop background polling.
        routine?.let {
            if (it.isDone) {
                routine = null
                it.stop(false)
                finish()
            }
        }
    }

    override fun reportTelemetry() {
        val pose = follower.pose()
        telemetry.addData("Path state", follower.mode())
        telemetry.addData("Pose (in / deg)", "%.1f, %.1f, %.0f", pose.x(), pose.y(), Math.toDegrees(pose.heading()))
    }

    override fun stopRoutine() {
        val command = routine
        routine = null
        command?.stop(true)
    }
}

@Autonomous(name = "BioBuzz Auto Blue")
class BioBuzzAutoBlue : BioBuzzRed1_5(blue = true)

// endregion

// region Backup autos: drive one line to park, with no shooter initialization

open class BioBuzzBackupAuto(
    blue: Boolean, private val startX: Double, private val startY: Double,
    private val startHeadingDeg: Double, private val endHeadingDeg: Double,
    private val parkY: Double
) : AutoOpMode() {
    // Existing 18-inch robot length assumption; preserve it until measured on the robot.
    private val redParkX = 0.16 + 3.5 + 18.0 / 2.0
    private val parkX = if (blue) 141.5 - redParkX else redParkX
    private lateinit var toPark: Path

    override fun usesShooter() = false
    override fun startingPose(): Pose = PoseFactory.degrees().of(startX, startY, startHeadingDeg)
    override fun buildRoutine(start: Pose) {
        val park = PoseFactory.degrees().of(parkX, parkY, endHeadingDeg)
        // Preserve unwrapped headings (including 180 -> 360) and their turn direction.
        toPark = Paths.line(start, park)
            .linear(Math.toRadians(startHeadingDeg), Math.toRadians(endHeadingDeg))
    }
    override fun startRoutine() { follower.follow(toPark) }
    override fun updateRoutine() {
        operation = "LOOP Pedro / Pinpoint"
        // main: keep updating after arrival, so Pedro holds the park pose against pushes.
        follower.update()
    }
}

// endregion

// region Shared shooting command and guarded lifecycle

/** NextFTC command: hold position and shoot, finishing after one second of transfer. */
class ShootCommand(
    private val follower: Follower,
    private val shooter: ShooterSubsystem,
    private val intake: IntakeSubsystem,
    private val power: Double = DEFAULT_SHOOTER_PRESET,
    private val nanoTime: () -> Long = System::nanoTime
) : Command() {
    private var finished = false
    private var transferStarted: Long? = null
    private val feed = IntakeFeed(feedPower = 1.0, nanoTime) // main: ShootCommand feeds at full power.
    override val isDone: Boolean get() = finished

    init {
        require(power.isFinite() && power > 0.0 && power <= 1.0)
        requires(follower, shooter, intake)
    }

    override fun start() {
        finished = false
        transferStarted = null
        feed.reset()
        stopAll()
        shooter.startBlockerControl()
        follower.hold(follower.pose())
    }

    override fun update() {
        if (finished) return
        shooter.update(power, Mode.LQR)
        val transferPower = feed.power(shooter, power)
        val now = nanoTime()
        if (transferPower > 0.0) {
            val start = transferStarted ?: now.also { transferStarted = it }
            finished = now - start >= 1_000_000_000L
        } else {
            // A readiness interruption must not count as running the transfer.
            transferStarted = null
        }
        if (finished) stopAll() else intake.setPower(transferPower)
    }

    override fun stop(interrupted: Boolean) = stopAll()

    private fun stopAll() {
        var failure = SafeHardware.attempt(null) { intake.forceStop() }
        failure = SafeHardware.attempt(failure) { shooter.forceStop() }
        failure = SafeHardware.attempt(failure) { follower.stop() }
        SafeHardware.rethrow(failure)
    }
}

/**
 * NextFTC auto base. READY is required before motion; every exit attempts every acquired
 * output independently. Each lifecycle hook is guarded here because NextFTC skips onStop
 * after an exception. Routines call [finish] to end early; the OpMode then idles until STOP.
 */
abstract class AutoOpMode protected constructor(
    protected val nanoTime: LongSupplier = LongSupplier(System::nanoTime)
) : NextFTCOpMode() {
    private enum class State { UNINITIALIZED, READY, RUNNING, DONE, FAILED }
    private var state = State.UNINITIALIZED
    private var failureMessage: String? = null
    private var failure: Throwable? = null
    protected var operation = "Not initialized"
    protected lateinit var follower: Follower
    protected lateinit var intake: IntakeSubsystem
    protected lateinit var shooter: ShooterSubsystem
    private val safety = SafeHardware()
    private val telemetryRefresh = TelemetryRefresh(nanoTime::getAsLong)
    private var hubs = emptyList<LynxModule>()

    protected open fun usesShooter() = true
    protected open fun usesFollower() = true
    protected abstract fun startingPose(): Pose
    protected abstract fun buildRoutine(start: Pose)
    protected abstract fun startRoutine()
    protected abstract fun updateRoutine()
    protected open fun stopRoutine() {}
    protected open fun runningStage() = "RUNNING"

    final override fun onInit() = guarded {
        // The blocker servo is untouched until START, as on main.
        operation = "INIT motor setup"
        safety.prepareMotors(hardwareMap, usesShooter())
        if (usesShooter()) {
            operation = "INIT intake"
            intake = IntakeSubsystem(hardwareMap)
            operation = "INIT shooter"
            shooter = ShooterSubsystem(hardwareMap, nanoTime::getAsLong)
        }
        val start = startingPose()
        if (usesFollower()) {
            operation = "INIT Pedro / Pinpoint"
            follower = Constants.create(hardwareMap)
            follower.setPose(start)
        }
        operation = "INIT path construction"
        buildRoutine(start)
        operation = "INIT hub bulk cache"
        hubs = hardwareMap.getAll(LynxModule::class.java)
        safety.setCaching(hubs, LynxModule.BulkCachingMode.MANUAL)
        telemetry.msTransmissionInterval = TELEMETRY_INTERVAL_MS.toInt()
        operation = "INIT voltage polling"
        if (::shooter.isInitialized) shooter.startVoltagePolling()
        state = State.READY
    }

    final override fun onWaitForStart() {
        // LinearOpMode spins in INIT, so refresh at the telemetry interval only.
        if (!telemetryRefresh.isDue()) return
        if (failure != null) { showFailure(); return }
        guarded {
            operation = "INIT telemetry"
            telemetry.addData("Auto stage", state)
            if (::shooter.isInitialized) telemetry.addData("Shooter target RPM",
                DEFAULT_SHOOTER_PRESET * MAX_MOTOR_RPM)
            telemetry.update()
        }
    }

    final override fun onStartButtonPressed() {
        if (failure != null) { showFailure(); return }
        guarded {
            operation = "START"
            check(state == State.READY) { "Auto initialization is incomplete" }
            telemetryRefresh.reset()
            if (::follower.isInitialized) {
                follower.stop()
                follower.drivetrain.stop()
            }
            if (::shooter.isInitialized) shooter.startBlockerControl()
            startRoutine()
            state = State.RUNNING
        }
    }

    final override fun onUpdate() {
        if (failure != null) { if (telemetryRefresh.isDue()) showFailure(); return }
        if (state != State.RUNNING) return
        guarded {
            operation = "LOOP hub bulk cache"
            hubs.forEach { it.clearBulkCache() }
            updateRoutine()
            if (failure == null && telemetryRefresh.isDue()) {
                operation = "LOOP telemetry"
                telemetry.addData("Auto stage", if (state == State.RUNNING) runningStage() else state)
                reportTelemetry()
                telemetry.update()
            }
        }
    }

    protected open fun reportTelemetry() {
        if (!::follower.isInitialized) return
        val pose = follower.pose()
        telemetry.addData("Pose (in / deg)", "%.1f, %.1f, %.0f", pose.x(), pose.y(), Math.toDegrees(pose.heading()))
    }

    final override fun onStop() = finish()

    /** End the routine: stop every output and the voltage thread. Safe to call more than once. */
    protected fun finish() {
        state = if (failure == null) State.DONE else State.FAILED
        shutdown(failure)?.let { recordFailure(it, "STOP") }
    }

    private fun shutdown(cause: Throwable?): Throwable? {
        var error = SafeHardware.attempt(cause, ::stopRoutine)
        if (::shooter.isInitialized) error = SafeHardware.attempt(error, shooter::stopVoltagePolling)
        if (::follower.isInitialized) {
            error = SafeHardware.attempt(error, follower::stop)
            error = SafeHardware.attempt(error, follower.drivetrain::stop)
        }
        if (::intake.isInitialized) error = SafeHardware.attempt(error, intake::forceStop)
        if (::shooter.isInitialized) error = SafeHardware.attempt(error, shooter::forceStop)
        // Direct zero writes still happen after a localizer or controller failure.
        return safety.stop(error)
    }

    private inline fun guarded(action: () -> Unit) {
        try { action() }
        catch (error: RuntimeException) { recordFailure(shutdown(error)!!, operation) }
        catch (error: LinkageError) { recordFailure(shutdown(error)!!, operation) }
    }

    private fun recordFailure(error: Throwable, phase: String) {
        state = State.FAILED
        if (failure == null) {
            failure = error
            failureMessage = "$phase: ${error.javaClass.simpleName}: ${error.message}"
        }
        RobotLog.ee("BioBuzzAuto", failure, failureMessage)
        showFailure()
    }

    private fun showFailure() {
        try {
            telemetry.addData("Auto stage", State.FAILED)
            // Use the value overload: an exception message may contain literal '%' characters.
            telemetry.addData("Auto error", failureMessage as Any?)
            telemetry.update()
        } catch (error: RuntimeException) {
            RobotLog.ee("BioBuzzAuto", error, "Unable to display auto failure")
        } catch (error: LinkageError) {
            RobotLog.ee("BioBuzzAuto", error, "Unable to display auto failure")
        }
    }
}

// endregion

@Autonomous(name = "Backup Blue Left", group = "Backup")
class BackupAutoBlueLeft : BioBuzzBackupAuto(true, 112.1, 7.8, -90.0, 0.0, 35.5)

@Autonomous(name = "Backup Blue Right", group = "Backup")
class BackupAutoBlueRight : BioBuzzBackupAuto(true, 133.7, 74.7, 180.0, 360.0, 35.5)

@Autonomous(name = "Backup Red Left", group = "Backup")
class BackupAutoRedLeft : BioBuzzBackupAuto(false, 29.4, 133.7, 90.0, 180.0, 106.0)

@Autonomous(name = "Backup Red Right", group = "Backup")
class BackupAutoRedRight : BioBuzzBackupAuto(false, 7.8, 66.8, 0.0, 180.0, 106.0)
