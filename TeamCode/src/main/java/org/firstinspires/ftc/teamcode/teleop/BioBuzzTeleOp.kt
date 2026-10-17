package org.firstinspires.ftc.teamcode.teleop

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.robotcore.external.Telemetry
import java.util.Locale
import com.qualcomm.robotcore.util.RobotLog
import dev.nextftc.core.components.Component
import org.firstinspires.ftc.teamcode.diagnostics.LoopProfiler
import org.firstinspires.ftc.teamcode.diagnostics.LoopStage
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import com.qualcomm.hardware.lynx.LynxModule
import com.pedropathing.localization.Localizer
import com.pedropathing.revhub.localizers.PinpointLocalizer
import org.firstinspires.ftc.teamcode.Subsytem.DriveSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.FieldLocation
import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem

@TeleOp(name = "BioBuzz NextFTC Kotlin TeleOp")
class BioBuzzTeleOp : NextFTCOpMode() {
    private lateinit var shootingDriveHold: ShootingDriveHold
    private lateinit var drive: DriveSubsystem
    private lateinit var intake: IntakeSubsystem
    private lateinit var shooter: ShooterSubsystem
    private lateinit var pinpoint: Localizer
    private val snapshotLocalizer = SnapshotLocalizer()
    private var hubs: List<LynxModule> = emptyList()
    private var hardwareFrame: HardwareFrame? = null
    private var computeStartedNanos: Long? = null
    private var location = FieldLocation(0.0, 0.0, 0.0)
    private var hiveLocation: FieldLocation? = null
    private var g1 = ControlSnapshot()
    private var g2 = ControlSnapshot()
    private var driveEncoderPositions: IntArray? = null
    private lateinit var driveMotors: List<DcMotor>
    private var previousDriveEncoderPositions: IntArray? = null
    private var previousXButtonPressed = false
    private var previousAimButtonPressed = false
    private var blueHiveHeading: Double? = null
    private var rotatingToBlueHive = false
    private var detailedLogging = true
    private var previousLoggingButton = false
    private val profiler = LoopProfiler()
    private var timingRows: List<Pair<String, String>> = emptyList()
    private val loopTiming = LoopTiming()
    private val blockerFeedDelay = BlockerFeedDelay()
    private val driveIntakeControls = DriveIntakeControls()
    private val shooterControls = ShooterControls()
    private val shooterPreset: Double get() = shooterControls.preset
    private val shooterMode: ShooterSubsystem.ControlMode get() = shooterControls.mode
    private var currentShooterPower = 0.0
    private var currentIntakePower = 0.0

    init {
        // NextFTC 1.0.0 runs CommandManager explicitly before onUpdate AND through
        // its default component's postUpdate. Keep the explicit pass and preserve
        // all other manager lifecycle hooks through delegation.
        val defaultComponents = components.toList()
        (components as MutableSet<Component>).clear()
        addComponents(
            object : Component {
                override fun preUpdate() {
                    profiler.beginLoop()
                    hardwareFrame?.begin()
                    computeStartedNanos = profiler.nanoTime()
                }
                override fun postUpdate() = profiler.endLoop()
            },
            *defaultComponents.map { component ->
                if (component === CommandManager) object : Component by component {
                    override fun postUpdate() = Unit
                    // Default postStop runs commands once more before cancellation.
                    // Cancel directly so no controller executes outside a sampled frame.
                    override fun postStop() = CommandManager.cancelAll()
                } else component
            }.toTypedArray()
        )
        // The timer is first, so reverse postUpdate ordering ends it last.
    }

    override fun runOpMode() {
        var failure: Throwable? = null
        try {
            super.runOpMode()
        } catch (t: Throwable) {
            failure = t
            throw t
        } finally {
            try { safeStop() } catch (stopFailure: Exception) {
                if (failure == null) throw stopFailure else failure.addSuppressed(stopFailure)
            }
        }
    }

    override fun onInit() {
        telemetry = JoinedTelemetry(
            timedTelemetry(telemetry, LoopStage.DS_TELEMETRY_UPDATE),
            timedTelemetry(PanelsTelemetry.ftcTelemetry, LoopStage.PANELS_TELEMETRY_UPDATE)
        )
        telemetry.setMsTransmissionInterval(50)
        drive = DriveSubsystem(hardwareMap)
        driveMotors = listOf("driveFL", "driveFR", "driveBL", "driveBR")
            .map { hardwareMap.get(DcMotor::class.java, it) }
        intake = IntakeSubsystem(hardwareMap)
        shooter = ShooterSubsystem(hardwareMap)
        shooter.loopProfiler = profiler
        drive.enablePhasedIo()
        intake.enablePhasedIo()
        shooter.enablePhasedIo()
        // One physical localizer, using exactly the Pedro calibration used before this refactor.
        pinpoint = PinpointLocalizer(hardwareMap, Constants.localizerConfig)
        shootingDriveHold = ShootingDriveHold(Constants.create(
            snapshotLocalizer, BufferedMecanum { powers, threshold -> drive.requestWheelPowers(powers, threshold) }))
        hubs = hardwareMap.getAll(LynxModule::class.java)
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }
        hardwareFrame = HardwareFrame(::readInputs, ::writeOutputs)
        addComponents(SubsystemComponent(drive, intake, shooter))
        shooter.startVoltagePolling()
    }

    override fun onStartButtonPressed() {
        loopTiming.reset()
        profiler.reset()
        timingRows = emptyList()
        profiler.measure(LoopStage.LOCATION_RESET) { pinpoint.setPose(Pose.zero()) }
        LambdaCommand("Driver control")
            .setIsDone { false }
            .setUpdate { updateDrive() }
            .setStop { shootingDriveHold.stop(); drive.stop() }
            .requires(drive)
            .schedule()

        LambdaCommand("Shooter and intake control")
            .setIsDone { false }
            .setUpdate { updateMechanisms() }
            .setStop {
                shooter.setManualBlockerOpen(false)
                shooter.setFlywheelPower(0.0)
                intake.setPower(0.0)
            }
            .requires(shooter, intake)
            .schedule()
    }

    private fun updateDrive() {
        profiler.measure(LoopStage.DRIVE_TOTAL) {
            driveEncoderPositions?.let { positions ->
                val previousPositions = previousDriveEncoderPositions ?: positions
                if (detailedLogging) profiler.measure(LoopStage.DRIVE_TELEMETRY) {
                    telemetry.addData("Drive encoder delta (ticks)", "FL %d, FR %d, BL %d, BR %d",
                        positions[0] - previousPositions[0], positions[1] - previousPositions[1],
                        positions[2] - previousPositions[2], positions[3] - previousPositions[3])
                }
                previousDriveEncoderPositions = positions
            }
            if (g1.x && !previousXButtonPressed) {
                hiveLocation = location
                blueHiveHeading = null
                rotatingToBlueHive = false
            }
            previousXButtonPressed = g1.x
            // Gamepad 1 Y is reserved for hold-to-run manual shooting.
            if (g2.y && !previousAimButtonPressed) {
                blueHiveHeading = hiveLocation?.let { kotlin.math.atan2(it.yInches - location.yInches, it.xInches - location.xInches) }
                rotatingToBlueHive = blueHiveHeading != null
            }
            previousAimButtonPressed = g2.y
            val manualTurn = g1.right_stick_x.toDouble()
            var turn = manualTurn
            var headingError: Double? = null
            if (kotlin.math.abs(manualTurn) > 0.05) {
                rotatingToBlueHive = false
            } else if (rotatingToBlueHive) {
                val targetHeading = blueHiveHeading
                if (targetHeading == null) {
                    rotatingToBlueHive = false
                } else {
                    headingError = wrapAngle(targetHeading - location.headingRadians)
                    if (kotlin.math.abs(headingError) <= HEADING_TOLERANCE_RADIANS) {
                        rotatingToBlueHive = false
                    } else {
                        val correction = (-TURN_GAIN * headingError)
                            .coerceIn(-MAX_AUTO_TURN, MAX_AUTO_TURN)
                        turn = if (kotlin.math.abs(correction) < MIN_AUTO_TURN)
                            kotlin.math.sign(correction) * MIN_AUTO_TURN else correction
                    }
                }
            }
            val shooting = (g1.right_bumper || g1.y) && !g1.left_bumper
            if (shooting && !shootingDriveHold.active) {
                rotatingToBlueHive = false
            }
            val holdPose = Pose(location.xInches, location.yInches, location.headingRadians)
            val holding = if (shooting || shootingDriveHold.active) {
                profiler.measure(LoopStage.DRIVE_HOLD) { shootingDriveHold.update(shooting, holdPose) }
            } else {
                shootingDriveHold.update(shooting, holdPose)
            }
            if (!holding) {
                drive.drive(-g1.left_stick_y.toDouble(), g1.left_stick_x.toDouble(), turn)
            }
            if (detailedLogging) {
                profiler.measure(LoopStage.DRIVE_TELEMETRY) {
                    telemetry.addData("Robot X (in)", location.xInches)
                    telemetry.addData("Robot Y (in)", location.yInches)
                    telemetry.addData("Robot Heading (rad)", location.headingRadians)
                    telemetry.addData("Blue Hive Heading (rad)",
                        blueHiveHeading ?: "Press G1 X to save hive, then G2 Y to aim")
                    telemetry.addData("Turning to Blue Hive",
                        if (rotatingToBlueHive) "Yes (right stick cancels)" else "No")
                    headingError?.let { telemetry.addData("Heading Error (rad)", it) }
                }
            }
        }
    }

    private fun wrapAngle(angle: Double): Double {
        var wrapped = angle
        while (wrapped > Math.PI) wrapped -= 2.0 * Math.PI
        while (wrapped < -Math.PI) wrapped += 2.0 * Math.PI
        return wrapped
    }

    private companion object {
        const val HEADING_TOLERANCE_RADIANS = 0.035
        const val TURN_GAIN = 0.8
        const val MIN_AUTO_TURN = 0.12
        const val MAX_AUTO_TURN = 0.35
    }

    private fun updateMechanisms() {
        profiler.measure(LoopStage.MECHANISMS_TOTAL) {
            driveIntakeControls.update(g1.left_stick_button)
            val dpadUp = g1.dpad_up || g2.dpad_up
            val dpadDown = g1.dpad_down || g2.dpad_down
            shooterControls.update(g1.y, dpadUp, dpadDown)

            currentShooterPower = shooterControls.requestedPower(
                reverse = g1.left_bumper,
                pidShoot = g1.right_bumper
            )

            profiler.measure(LoopStage.SHOOTER_CONTROL) {
                shooter.setControlMode(shooterMode)
                if (currentShooterPower > 0.0) {
                    shooter.updateShot(currentShooterPower, shooterMode)
                } else {
                    shooter.setFlywheelPower(currentShooterPower)
                }
            }
            val manualBlockerOpen = g1.b || g2.b
            shooter.setManualBlockerOpen(manualBlockerOpen)

            val manualIntakePower = when {
                g1.right_trigger > 0.05 -> g1.right_trigger.toDouble()
                g1.left_trigger > 0.05 -> -g1.left_trigger.toDouble()
                else -> 0.0
            }

            currentIntakePower = FeedControls.intakePower(
                stop = g1.a,
                manualPower = manualIntakePower,
                shooterPower = currentShooterPower,
                preset = 1.0,
                mode = shooterMode,
                feedReady = shooter.feedReady,
                manualBlockerOpen = manualBlockerOpen,
                minimumPower = driveIntakeControls.minimumPower(drive.commandedPowerMagnitude)
            )

            currentIntakePower = blockerFeedDelay.apply(currentIntakePower, shooter.blockerOpen)
            intake.setPower(currentIntakePower)
        }
    }

    override fun onUpdate() {
        computeStartedNanos?.let { profiler.record(LoopStage.COMPUTE, profiler.nanoTime() - it) }
        computeStartedNanos = null
        hardwareFrame?.commit()
        loopTiming.update()
        if (!detailedLogging) {
            profiler.measure(LoopStage.TELEMETRY_BUILD) {
                telemetry.addData("Loop interval (ms)", "%.2f", loopTiming.intervalMs)
                reportTimings()
            }
            profiler.measure(LoopStage.TELEMETRY_UPDATE) { telemetry.update() }
            return
        }
        profiler.measure(LoopStage.TELEMETRY_BUILD) {
            telemetry.addData("Loop interval (ms)", "%.2f", loopTiming.intervalMs)
            telemetry.addData("Loop frequency (Hz)", "%.1f", loopTiming.frequencyHz)
            telemetry.addData("Detailed logging (G1 right-stick click)", "On")
            telemetry.addData("Shooting position hold", shootingDriveHold.active)
            telemetry.addData("Intake Power", currentIntakePower)
            telemetry.addData("Drive intake assist (G1 left-stick click)", driveIntakeControls.enabled)
            telemetry.addData("Shooter flywheel 1 RPM", "%.0f", shooter.measuredRpm1)
            telemetry.addData("Shooter flywheel 2 RPM", "%.0f", shooter.measuredRpm2)
            val selectedPidTarget = shooterControls.pidTargetRpm
            if (selectedPidTarget != null) {
                telemetry.addData("PID target RPM", "%.0f", selectedPidTarget)
            } else {
                telemetry.addData("PID target RPM", "Manual mode: no RPM target")
            }
            telemetry.addData("PID shooting active", selectedPidTarget != null && currentShooterPower > 0.0)
            telemetry.addData("PID commanded RPM", "%.0f", shooter.targetRpm)
            telemetry.addData("PIDF gains (P/I/D/F)", shooter.pidGainsRead)
            telemetry.addData("PID feedback", "Flywheel 1 RPM (shared motor power)")
            telemetry.addData("Shooter preset (G1/G2 up/down)", "%.3f", shooterPreset)
            telemetry.addData("Shooter mode (hold G1 Y for manual)", shooterMode)
            telemetry.addData("Shooter speed state", shooter.speedStatus)
            telemetry.addData("D-pad seen (up/down)", "%s / %s", g1.dpad_up || g2.dpad_up, g1.dpad_down || g2.dpad_down)
            telemetry.addData("Shooter requested fraction", "%.2f", currentShooterPower)
            telemetry.addData("Shooter motor power 1 / 2", "%.2f / %.2f", shooter.flywheelPower1, shooter.flywheelPower2)
            telemetry.addData("Manual blocker (G1/G2 B)", g1.b || g2.b)
            telemetry.addData("Blocker open", shooter.blockerOpen)
            telemetry.addData("Flywheel 1 within 5 percent of target", shooter.atSpeed)
            telemetry.addData("Automatic feed ready", shooter.feedReady)
            telemetry.addData("Feed readiness", shooter.feedReadiness)
            telemetry.addData("Blocker status", shooter.blockerStatus)
            telemetry.addData("Blocker commanded position", "%.2f", shooter.blockerCommandedPosition)
            telemetry.addData("Shooter preset reference RPM", "%.0f", ShooterSubsystem.MAX_MOTOR_RPM)
            telemetry.addData("RPM configuration valid", shooter.rpmConfigurationValid)
            telemetry.addData("Raw ticks/s 1 / 2", "%.0f / %.0f",
                shooter.measuredTicksPerSecond1, shooter.measuredTicksPerSecond2)
            telemetry.addData("Encoder position ticks 1 / 2", "%d / %d",
                shooter.encoderPosition1, shooter.encoderPosition2)
            telemetry.addData("PID target ticks/s 1 / 2", "%.0f / %.0f",
                shooter.targetTicksPerSecond1, shooter.targetTicksPerSecond2)
            telemetry.addData("Motor config ticks/rev 1 / 2", "%.0f / %.0f",
                shooter.configuredTicksPerRev1, shooter.configuredTicksPerRev2)
            telemetry.addData("Motor config max ticks/s 1 / 2", "%.0f / %.0f",
                shooter.configuredMaxTicksPerSecond1, shooter.configuredMaxTicksPerSecond2)
            reportTimings()
        }
        profiler.measure(LoopStage.TELEMETRY_UPDATE) { telemetry.update() }
    }

    private fun updateLoggingToggle() {
        val pressed = g1.right_stick_button
        if (pressed && !previousLoggingButton) {
            detailedLogging = !detailedLogging
            // Resume deltas from a fresh baseline after diagnostic reads were skipped.
            previousDriveEncoderPositions = null
            telemetry.clearAll()
        }
        previousLoggingButton = pressed
    }

    private fun timedTelemetry(delegate: Telemetry, stage: LoopStage): Telemetry =
        object : Telemetry by delegate {
            override fun update(): Boolean = profiler.measure(stage) { delegate.update() }
        }

    private fun reportTimings() {
        profiler.measure(LoopStage.TIMING_REPORT) {
            profiler.snapshotIfDue()?.let { samples ->
                val byStage = samples.associateBy { it.stage }
                timingRows = LoopStage.entries.map { stage ->
                    val sample = byStage[stage]
                    "Timing: ${stage.label}" to if (sample == null) "not called in this window" else
                        String.format(Locale.US, "avg=%.3f ms max=%.3f ms n=%d",
                            sample.averageMs, sample.maxMs, sample.calls)
                }
                RobotLog.ii("BioBuzzLoop", timingRows.joinToString("; ") { "${it.first}: ${it.second}" })
            }
            // Reuse formatted strings between reports. Totals include their child stages.
            for ((label, value) in timingRows) telemetry.addData(label, value)
        }
    }

    internal fun captureControls() {
        g1 = ControlSnapshot.capture(gamepad1)
        g2 = ControlSnapshot.capture(gamepad2)
        updateLoggingToggle()
    }

    private fun readInputs() = profiler.measure(LoopStage.READ_INPUTS) {
        captureControls()
        // Clearing is local bookkeeping. First motor read lazily requests one bulk response
        // per needed hub; subsequent encoder/velocity getters use that same response.
        profiler.measure(LoopStage.BULK_CACHE) { hubs.forEach { it.clearBulkCache() } }
        shooter.beginFrame()
        driveEncoderPositions = if (detailedLogging) profiler.measure(LoopStage.DRIVE_ENCODERS) {
            driveMotors.map { it.currentPosition }.toIntArray()
        } else null
        profiler.measure(LoopStage.PINPOINT) {
            pinpoint.update()
            snapshotLocalizer.capture(pinpoint.state())
            val pose = snapshotLocalizer.pose()
            location = FieldLocation(pose.x(), pose.y(), pose.heading())
        }
    }

    private fun writeOutputs() = profiler.measure(LoopStage.WRITE_OUTPUTS) {
        profiler.measure(LoopStage.DRIVE_OUTPUT) { drive.commitOutputs() }
        shooter.commitOutputs()
        profiler.measure(LoopStage.INTAKE_OUTPUT) { intake.commitOutputs() }
    }

    /** Try every safe output even if another device throws; never rely on cached equality. */
    private fun safeStop() {
        val actions = listOf<() -> Unit>(
            { CommandManager.cancelAll() },
            { if (::shooter.isInitialized) shooter.stopVoltagePolling() },
            { if (::shootingDriveHold.isInitialized) shootingDriveHold.stop() },
            { if (::drive.isInitialized) drive.forceStop() },
            { if (::intake.isInitialized) intake.forceStop() },
            { if (::shooter.isInitialized) shooter.forceStop() })
        var failure: Throwable? = null
        actions.forEach { action ->
            try { action() } catch (t: Exception) {
                if (failure == null) failure = t else failure.addSuppressed(t)
            }
        }
        failure?.let { throw it }
    }

    override fun onStop() = safeStop()
}
