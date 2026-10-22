package org.firstinspires.ftc.teamcode.teleop

import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.math.Pose
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.math.*
import org.firstinspires.ftc.teamcode.control.*
import org.firstinspires.ftc.teamcode.diagnostics.*
import org.firstinspires.ftc.teamcode.hardware.RobotHardware
import org.firstinspires.ftc.teamcode.subsystems.IntakeFeed

/** FTC supplies hardwareMap during INIT, so construct the robot there. */
@TeleOp(name = "BioBuzz NextFTC Kotlin TeleOp")
class Robot : OpMode() {
    private var hardware: RobotHardware? = null
    private var driver: DriverRobot? = null
    private var running = false

    override fun init() = guarded {
        val devices = RobotHardware(hardwareMap).also { hardware = it }
        val display = RobotTelemetry(telemetry, PanelsTelemetry.ftcTelemetry, devices.profiler)
        driver = DriverRobot(devices, display)
    }

    override fun start() = guarded {
        checkNotNull(driver) { "Robot initialization is incomplete" }.start()
        running = true
    }

    override fun loop() { if (running) guarded { driver!!.update(gamepad1, gamepad2) } }

    override fun stop() {
        running = false
        driver = null
        hardware?.close()
        hardware = null
    }

    private inline fun guarded(action: () -> Unit) {
        try { action() } catch (failure: Throwable) {
            try { stop() } catch (cleanup: Throwable) {
                if (cleanup !== failure) failure.addSuppressed(cleanup)
            }
            throw failure
        }
    }
}

/** One owner for driver state and the read → compute → write → report loop. */
internal class DriverRobot(
    private val hardware: RobotHardware,
    val telemetry: RobotTelemetry,
    nanoTime: () -> Long = System::nanoTime
) {
    private val drive = hardware.drive
    private val shooter = hardware.shooter
    private val hold = hardware.hold
    private val profiler = hardware.profiler
    private val location: Pose get() = hardware.pose
    private var g1 = Gamepad()
    private var g2 = Gamepad()
    private val saveButton = PressEdge()
    private val aimButton = PressEdge()
    private val loggingButton = PressEdge()
    private val loopTiming = LoopTiming(nanoTime)
    private val intakeFeed = IntakeFeed(nanoTime)
    private val driveIntakeControls = DriveIntakeControls()
    private val shooterControls = ShooterControls()
    private var detailedLogging = true
    private val previousEncoders = IntArray(4)
    private var hasEncoderBaseline = false
    private val encoderDeltas = IntArray(4)
    private var savedHive: Pose? = null
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
        profiler.reset()
        hardware.start()
    }

    fun update(gamepad1: Gamepad, gamepad2: Gamepad) {
        profiler.beginLoop()
        try {
            captureControls(gamepad1, gamepad2)
            hardware.read(detailedLogging)
            profiler.measure(LoopStage.COMPUTE) { updateDrive(); updateMechanisms() }
            hardware.write()
            updateTelemetry()
        } finally {
            profiler.endLoop()
        }
    }

    fun captureControls(gamepad1: Gamepad, gamepad2: Gamepad) {
        // FTC 12 iterative OpMode freezes these gamepads for each user callback.
        g1 = gamepad1
        g2 = gamepad2
        if (loggingButton.pressed(g1.right_stick_button)) {
            detailedLogging = !detailedLogging
            hasEncoderBaseline = false
            telemetry.clear()
        }
    }

    internal fun updateDrive() = profiler.measure(LoopStage.DRIVE_TOTAL) {
        hardware.encoderPositions?.let { positions ->
            for (i in encoderDeltas.indices) {
                encoderDeltas[i] =
                    if (hasEncoderBaseline) positions[i] - previousEncoders[i] else 0
                previousEncoders[i] = positions[i]
            }
            hasEncoderBaseline = true
        }
        if (saveButton.pressed(g1.x)) {
            savedHive = location
            targetHeading = null
            aiming = false
        }
        if (aimButton.pressed(g2.y)) {
            targetHeading = savedHive?.let {
                atan2(it.y() - location.y(), it.x() - location.x())
            }
            aiming = targetHeading != null
        }
        val turn = aimedTurn(g1.right_stick_x.toDouble())
        val shooting = (g1.right_bumper || g1.y) && !g1.left_bumper
        if (shooting && !hold.active) aiming = false
        val holding = (shooting || hold.active) && profiler.measure(LoopStage.DRIVE_HOLD) {
            hold.update(shooting, location)
        }
        if (!holding) drive.drive(-g1.left_stick_y.toDouble(), g1.left_stick_x.toDouble(), turn)
    }

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

    internal fun updateMechanisms() = profiler.measure(LoopStage.MECHANISMS_TOTAL) {
        driveIntakeControls.update(g1.left_stick_button)
        shooterControls.update(g1.y, up, down)
        shooterPower = shooterControls.requestedPower(g1.left_bumper, g1.right_bumper)
        profiler.measure(LoopStage.SHOOTER_CONTROL) {
            shooter.setControlMode(shooterControls.mode)
            if (shooterPower > 0.0) shooter.updateShot(shooterPower, shooterControls.mode)
            else shooter.setFlywheelPower(shooterPower)
        }
        shooter.setManualBlockerOpen(manualBlocker)
        val manualIntake = when {
            g1.right_trigger > 0.05 -> g1.right_trigger.toDouble()
            g1.left_trigger > 0.05 -> -g1.left_trigger.toDouble()
            else -> 0.0
        }
        intakePower = intakeFeed.power(
            shooter, shooterPower, stop = g1.a, manualPower = manualIntake,
            manualBlockerOpen = manualBlocker,
            minimumPower = driveIntakeControls.minimumPower(drive.commandedPowerMagnitude)
        )
        hardware.intake.setPower(intakePower)
    }

    internal fun updateTelemetry() {
        loopTiming.update()
        telemetry.publish {
            addData("Loop interval (ms)", "%.2f", loopTiming.intervalMs)
            if (detailedLogging) {
                reportDrive()
                addData("Loop frequency (Hz)", "%.1f", loopTiming.frequencyHz)
                addData("Detailed logging (G1 right-stick click)", "On")
                addData("Shooting position hold", hold.active)
                addData("Intake Power", intakePower)
                addData("Drive intake assist (G1 left-stick click)",
                    driveIntakeControls.enabled)
                reportShooter(shooter, shooterControls, shooterPower, manualBlocker, up, down)
            }
            reportTimings()
        }
    }

    private fun RobotTelemetry.reportDrive() = profiler.measure(LoopStage.DRIVE_TELEMETRY) {
        addData("Drive encoder delta (ticks)", "FL %d, FR %d, BL %d, BR %d",
            encoderDeltas[0], encoderDeltas[1], encoderDeltas[2], encoderDeltas[3])
        addData("Robot X (in)", location.x())
        addData("Robot Y (in)", location.y())
        addData("Robot Heading (rad)", location.heading())
        addData("Blue Hive Heading (rad)",
            targetHeading ?: "Press G1 X to save hive, then G2 Y to aim")
        addData("Turning to Blue Hive",
            if (aiming) "Yes (right stick cancels)" else "No")
        headingError?.let { addData("Heading Error (rad)", it) }
    }
}
