package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.control.ShooterController.ControlMode
import org.firstinspires.ftc.teamcode.hardware.outputWrites

class IntakeSubsystem(hardwareMap: HardwareMap) {
    private val intake1 = hardwareMap.get(DcMotorEx::class.java, "intake1")
    private val intake2 = hardwareMap.get(DcMotorEx::class.java, "intake2")

    private val motors = listOf(intake1, intake2)
    private var phasedIo = false
    private var desiredPower = 0.0
    private val committed = DoubleArray(2) { Double.NaN }
    internal fun enablePhasedIo() { phasedIo = true; forceStop() }
    internal fun commitOutputs(force: Boolean = false) {
        outputWrites(force, motors.size) { i ->
            val motor = motors[i]
            if (force || desiredPower != committed[i]) {
                motor.power = desiredPower
                committed[i] = desiredPower
            }
        }
    }
    fun forceStop() { desiredPower = 0.0; commitOutputs(force = true) }

    init {
        motors.forEachIndexed { index, motor ->
            motor.direction = if (index == 0) DcMotorSimple.Direction.REVERSE
                else DcMotorSimple.Direction.FORWARD
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
        forceStop()
    }

    fun setPower(power: Double) {
        val clipped = if (power.isFinite()) power.coerceIn(-1.0, 1.0) else 0.0
        desiredPower = clipped
        if (!phasedIo) commitOutputs()
    }
}

/** Resolve feeding every loop; a trigger cannot bypass RPM readiness in LQR shooting. */
internal object FeedControls {
    fun intakePower(stop: Boolean, manualPower: Double, shooterPower: Double,
                    preset: Double, mode: ControlMode, feedReady: Boolean,
                    manualBlockerOpen: Boolean, minimumPower: Double = 0.0): Double {
        if (stop) return 0.0
        val manual = if (manualPower.isFinite()) manualPower.coerceIn(-1.0, 1.0) else 0.0
        // Reverse remains available to clear a jam.
        if (manual < 0.0) return manual
        val minimum = if (minimumPower.isFinite()) minimumPower.coerceIn(0.0, 1.0) else 0.0
        if (shooterPower > 0.0) {
            val canFeed = feedReady ||
                (mode == ControlMode.MANUAL_POWER && manualBlockerOpen)
            if (!canFeed) return 0.0
            return maxOf(minimum, if (manual > 0.0) manual else preset.coerceIn(0.0, 1.0))
        }
        // Ordinary collection is available with the shooter off.
        return maxOf(manual, minimum)
    }
}

/** Allow the blocker 100 ms to open before applying forward intake power. */
internal class BlockerFeedDelay(private val nanoTime: () -> Long = System::nanoTime) {
    private var openedAt: Long? = null

    fun apply(power: Double, blockerOpen: Boolean): Double {
        if (!blockerOpen) {
            openedAt = null
            return power
        }
        val now = nanoTime()
        val start = openedAt ?: now.also { openedAt = it }
        return if (power > 0.0 && now - start < 100_000_000L) 0.0 else power
    }
}

/** One feeding policy for TeleOp and every shooting auto. */
class IntakeFeed @JvmOverloads constructor(private val nanoTime: () -> Long = System::nanoTime) {
    private var delay = BlockerFeedDelay(nanoTime)

    fun reset() { delay = BlockerFeedDelay(nanoTime) }

    @JvmOverloads
    fun power(shooter: ShooterSubsystem, shooterPower: Double, stop: Boolean = false,
              manualPower: Double = 0.0, manualBlockerOpen: Boolean = false,
              minimumPower: Double = 0.0): Double {
        val requested = FeedControls.intakePower(stop, manualPower, shooterPower,
            FEED_POWER, shooter.control.controlMode, shooter.control.feedReady,
            manualBlockerOpen, minimumPower)
        return delay.apply(requested, shooter.control.blockerOpen)
    }

    companion object { const val FEED_POWER = 0.6 }
}
