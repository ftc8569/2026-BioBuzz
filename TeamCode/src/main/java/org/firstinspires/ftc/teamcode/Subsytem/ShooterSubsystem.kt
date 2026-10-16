package org.firstinspires.ftc.teamcode.Subsytem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.core.subsystems.Subsystem
import kotlin.math.abs

class ShooterSubsystem @JvmOverloads constructor(
    hardwareMap: HardwareMap,
    private val nanoTime: () -> Long = System::nanoTime
) : Subsystem {
    enum class ControlMode { MANUAL_POWER, VELOCITY_PID }

    private val shooter1 = hardwareMap.get(DcMotorEx::class.java, "shooter1")
    private val shooter2 = hardwareMap.get(DcMotorEx::class.java, "shooter2")
    private val blocker = hardwareMap.get(Servo::class.java, "blocker")

    // These must describe encoder counts per revolution of the 4500-RPM shaft.
    val configuredTicksPerRev1 = shooter1.motorType.ticksPerRev
    val configuredTicksPerRev2 = shooter2.motorType.ticksPerRev
    val configuredMaxTicksPerSecond1 = shooter1.motorType.achieveableMaxTicksPerSecond
    val configuredMaxTicksPerSecond2 = shooter2.motorType.achieveableMaxTicksPerSecond
    val rpmConfigurationValid: Boolean get() =
        configuredTicksPerRev1.isFinite() && configuredTicksPerRev1 > 0.0 &&
        configuredTicksPerRev2.isFinite() && configuredTicksPerRev2 > 0.0

    var measuredTicksPerSecond1 = 0.0
        private set
    var measuredTicksPerSecond2 = 0.0
        private set
    var measuredRpm1 = 0.0
        private set
    var measuredRpm2 = 0.0
        private set
    var targetRpm = 0.0
        private set
    val targetTicksPerSecond1: Double get() =
        if (rpmConfigurationValid) targetRpm * configuredTicksPerRev1 / 60.0 else 0.0
    val targetTicksPerSecond2: Double get() =
        if (rpmConfigurationValid) targetRpm * configuredTicksPerRev2 / 60.0 else 0.0
    var encoderPosition1 = 0
        private set
    var encoderPosition2 = 0
        private set
    val flywheelPower1: Double get() = shooter1.power
    val flywheelPower2: Double get() = shooter2.power
    val blockerCommandedPosition: Double get() = blocker.position
    var blockerOpen = false
        private set
    var blockerStatus = "Shooter off"
        private set
    var atSpeed = false
        private set
    var feedReady = false
        private set
    var feedReadiness = "Shooter off"
        private set
    var encoderFault = false
        private set
    var controlMode = ControlMode.VELOCITY_PID
        private set
    var speedStatus = "Off"
        private set

    private var requestedPower = 0.0
    private var spinupStartedNanos = -1L
    private var lastControlNanos = -1L
    private var inToleranceSinceNanos = -1L
    private var feedbackEstablished = false
    private var manualBlockerOpen = false
    private val controller1 = VelocityPidfController()
    private val controller2 = VelocityPidfController()

    companion object {
        const val MAX_MOTOR_RPM = (3600.0 / 8.0) * 10.0
        private const val ENCODER_CHECK_NANOS = 3_000_000_000L
        private const val POWER_SPINUP_NANOS = 3_000_000_000L
        private const val READY_SPEED_FRACTION = 0.05
        private const val READY_STABLE_NANOS = 150_000_000L
        private const val BLOCKER_OPEN = 0.45
        private const val BLOCKER_CLOSED = 0.24
    }

    init {
        // Reverse both motor polarities while preserving logical forward/reverse controls.
        shooter1.direction = DcMotorSimple.Direction.FORWARD
        shooter2.direction = DcMotorSimple.Direction.REVERSE
        shooter1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        shooter2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        // Software PIDF reads encoders but commands power; avoid a second hub velocity loop.
        shooter1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        shooter2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        shooter1.power = 0.0
        shooter2.power = 0.0
        updateBlocker()
    }

    /** Also sync the mode while idle, so a stale manual override cannot bypass PID. */
    fun setControlMode(mode: ControlMode) {
        if (mode != controlMode) {
            resetSpeedHold()
            controlMode = mode
            shooter1.power = 0.0
            shooter2.power = 0.0
            updateBlocker()
        }
    }

    /** The selected fraction sets a fixed RPM target immediately, without learning a speed. */
    fun updateShot(power: Double, mode: ControlMode): Boolean {
        setControlMode(mode)
        val clipped = if (power.isFinite()) power.coerceIn(0.0, 1.0) else 0.0
        if (clipped == 0.0) {
            setFlywheelPower(0.0)
            return false
        }
        if (clipped != requestedPower) resetSpeedHold()
        requestedPower = clipped
        readSpeeds()
        val now = nanoTime()
        if (spinupStartedNanos == -1L) spinupStartedNanos = now

        if (mode == ControlMode.MANUAL_POWER) {
            shooter1.power = clipped
            shooter2.power = clipped
            feedReady = now - spinupStartedNanos >= POWER_SPINUP_NANOS
            feedReadiness = if (feedReady) "Timed feed ready - speed unverified"
                else "Timed spin-up: wait 3 seconds - speed unverified"
            speedStatus = "Raw power"
            updateBlocker()
            return false
        }

        targetRpm = clipped * MAX_MOTOR_RPM
        // A transient dropout must close feeding, but must not latch the intake off forever.
        encoderFault = !rpmConfigurationValid ||
            !measuredRpm1.isFinite() || !measuredRpm2.isFinite() ||
            ((feedbackEstablished || now - spinupStartedNanos >= ENCODER_CHECK_NANOS) &&
                (measuredTicksPerSecond1 < 1.0 || measuredTicksPerSecond2 < 1.0))
        if (encoderFault) {
            controller1.reset()
            controller2.reset()
            lastControlNanos = -1L
            inToleranceSinceNanos = -1L
            atSpeed = false
            feedReady = false
            // Preserve a bounded spin-up command for a missing encoder, never authorize feed.
            // Invalid calibration cannot be used for RPM control.
            shooter1.power = if (rpmConfigurationValid) clipped else 0.0
            shooter2.power = if (rpmConfigurationValid) clipped else 0.0
            speedStatus = if (rpmConfigurationValid) "Encoder unavailable: waiting for feedback"
                else "Invalid encoder ticks/rev: correct motor configuration"
            feedReadiness = "PID feedback fault - feeding blocked"
            updateBlocker()
            return false
        }

        if (measuredTicksPerSecond1 >= 1.0 && measuredTicksPerSecond2 >= 1.0) {
            feedbackEstablished = true
        }
        val dt = if (lastControlNanos == -1L) 0.0 else
            ((now - lastControlNanos) / 1e9).coerceIn(0.0, 0.1)
        lastControlNanos = now
        shooter1.power = controller1.calculate(targetRpm, measuredRpm1, dt)
        shooter2.power = controller2.calculate(targetRpm, measuredRpm2, dt)

        val bothReady = withinTolerance(measuredRpm1) && withinTolerance(measuredRpm2)
        if (bothReady) {
            if (inToleranceSinceNanos == -1L) inToleranceSinceNanos = now
        } else {
            inToleranceSinceNanos = -1L
        }
        atSpeed = bothReady && now - inToleranceSinceNanos >= READY_STABLE_NANOS
        feedReady = atSpeed
        speedStatus = if (atSpeed) "PID holding target RPM" else "PID reaching target RPM"
        feedReadiness = if (atSpeed) "Both motors at target RPM" else "Waiting for both motors' RPM"
        updateBlocker()
        return atSpeed
    }

    /** Manual mode only: PID never allows B to open the blocker before measured readiness. */
    fun setManualBlockerOpen(open: Boolean) {
        manualBlockerOpen = open
        updateBlocker()
    }

    fun setFlywheelPower(power: Double) {
        val clipped = if (power.isFinite()) power.coerceIn(-1.0, 0.0) else 0.0
        shooter1.power = clipped
        shooter2.power = clipped
        resetSpeedHold()
        readSpeeds()
        speedStatus = if (clipped < 0.0) "Reversing" else "Off"
        updateBlocker()
    }

    private fun readSpeeds() {
        encoderPosition1 = shooter1.currentPosition
        encoderPosition2 = shooter2.currentPosition
        measuredTicksPerSecond1 = abs(shooter1.velocity)
        measuredTicksPerSecond2 = abs(shooter2.velocity)
        measuredRpm1 = if (rpmConfigurationValid)
            measuredTicksPerSecond1 * 60.0 / configuredTicksPerRev1 else Double.NaN
        measuredRpm2 = if (rpmConfigurationValid)
            measuredTicksPerSecond2 * 60.0 / configuredTicksPerRev2 else Double.NaN
    }

    private fun withinTolerance(measuredRpm: Double): Boolean =
        targetRpm > 0.0 && measuredRpm.isFinite() &&
            abs(measuredRpm - targetRpm) <= targetRpm * READY_SPEED_FRACTION

    private fun resetSpeedHold() {
        encoderFault = false
        feedbackEstablished = false
        requestedPower = 0.0
        spinupStartedNanos = -1L
        targetRpm = 0.0
        lastControlNanos = -1L
        inToleranceSinceNanos = -1L
        controller1.reset()
        controller2.reset()
        atSpeed = false
        feedReady = false
        feedReadiness = "Shooter off"
    }

    private fun updateBlocker() {
        val manualOpenAllowed = manualBlockerOpen && controlMode == ControlMode.MANUAL_POWER
        blockerOpen = manualOpenAllowed || feedReady
        blockerStatus = when {
            manualOpenAllowed -> "Manual open"
            requestedPower > 0.0 -> feedReadiness
            manualBlockerOpen -> "Manual override disabled in PID"
            else -> "Shooter off"
        }
        blocker.position = if (blockerOpen) BLOCKER_OPEN else BLOCKER_CLOSED
    }
}
