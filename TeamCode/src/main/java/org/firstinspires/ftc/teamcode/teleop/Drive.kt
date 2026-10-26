package org.firstinspires.ftc.teamcode.teleop

import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.drivetrain.Drivetrain
import com.pedropathing.follower.Follower
import com.pedropathing.localization.Localizer
import com.pedropathing.localization.MotionState
import com.pedropathing.math.Pose
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import org.firstinspires.ftc.teamcode.pedro.Constants

// Drive and shooting position hold. Called by the shared Robot loop.

class DriveSubsystem(hardwareMap: HardwareMap) {
    private val motors = listOf("driveFL", "driveFR", "driveBL", "driveBR")
        .map { hardwareMap.get(DcMotor::class.java, it) }
    private var phasedIo = false
    private val desired = DoubleArray(4)
    private val committed = DoubleArray(4) { Double.NaN }
    private var threshold = 0.0

    internal fun enablePhasedIo() {
        phasedIo = true
        // Establish modes once; no recurring mode/direction traffic in the loop.
        motors.forEachIndexed { index, motor ->
            motor.direction = if (index == 0 || index == 2) DcMotorSimple.Direction.REVERSE
                else DcMotorSimple.Direction.FORWARD
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        }
        forceStop()
    }

    /** Both manual and Pedro requests go through this one output cache. */
    internal fun requestWheelPowers(powers: DoubleArray, powerThreshold: Double = 0.0) {
        require(powers.size == 4)
        for (i in desired.indices) {
            desired[i] = if (powers[i].isFinite()) powers[i].coerceIn(-1.0, 1.0) else 0.0
        }
        threshold = powerThreshold
        // Intake assist computes in this frame, so expose what the writer will actually
        // send/retain, including Pedro's deadband against the last successful command.
        commandedPowerMagnitude = desired.indices.maxOf { i ->
            abs(if (shouldWrite(i)) desired[i] else committed[i])
        }
        if (!phasedIo) commitOutputs(force = true)
    }

    private fun shouldWrite(i: Int): Boolean {
        val previous = committed[i]
        val value = desired[i]
        return previous.isNaN() || (value != previous &&
            (abs(value - previous) >= threshold ||
                kotlin.math.sign(value) != kotlin.math.sign(previous)))
    }

    internal fun commitOutputs(force: Boolean = false) {
        outputWrites(force, motors.size) { i ->
            val motor = motors[i]
            val value = desired[i]
            if (force || shouldWrite(i)) {
                motor.power = value
                committed[i] = value
            }
        }
        commandedPowerMagnitude = committed.maxOf { if (it.isFinite()) abs(it) else 0.0 }
    }

    internal fun forceStop() {
        desired.fill(0.0)
        commitOutputs(force = true)
    }

    var commandedPowerMagnitude = 0.0
        private set

    init {
        motors[0].direction = DcMotorSimple.Direction.REVERSE
        motors[2].direction = DcMotorSimple.Direction.REVERSE
    }

    fun drive(forward: Double, strafe: Double, turn: Double) {
        val denominator = maxOf(abs(forward) + abs(strafe) + abs(turn), 1.0)
        desired[0] = (forward + strafe + turn) / denominator * 0.9
        desired[1] = (forward - strafe - turn) / denominator * 0.9
        desired[2] = (forward - strafe + turn) / denominator * 0.9
        desired[3] = (forward + strafe - turn) / denominator * 0.9
        requestWheelPowers(desired)
    }

    fun stop() = drive(0.0, 0.0, 0.0)
}

/** Pedro 3.0.1 Mecanum math, with hardware writes delegated to the shared output owner. */
internal class BufferedMecanum(private val submit: (DoubleArray, Double) -> Unit) : Drivetrain {
    private val lastPowers = DoubleArray(4)
    private val currentWheels = DoubleArray(4)
    private val deltaWheels = DoubleArray(4)
    private fun wheels(p: DrivePowers, result: DoubleArray) {
        result[0] = p.forward() - p.strafe() - p.turn()
        result[1] = p.forward() + p.strafe() + p.turn()
        result[2] = p.forward() + p.strafe() - p.turn()
        result[3] = p.forward() - p.strafe() + p.turn()
    }

    override fun drive(powers: DrivePowers, manual: Boolean) {
        wheels(powers, lastPowers)
        val scale = maxOf(1.0, lastPowers.maxOf { abs(it) })
        for (i in lastPowers.indices) lastPowers[i] /= scale
        submit(lastPowers, Constants.drivetrainConfig.powerThreshold.get())
    }
    override fun maxScaling(current: DrivePowers, delta: DrivePowers): Double {
        wheels(current, currentWheels)
        wheels(delta, deltaWheels)
        var scale = 1.0
        for (i in currentWheels.indices) {
            val a = currentWheels[i]
            val b = deltaWheels[i]
            if (abs(b) < 1e-9) continue
            val positive = (1.0-a)/b
            val negative = (-1.0-a)/b
            if (positive >= 0.0) scale = minOf(scale, positive)
            if (negative >= 0.0) scale = minOf(scale, negative)
        }
        return scale.coerceIn(0.0, 1.0)
    }
    override fun stop() = stop(false)
    override fun stop(brake: Boolean) {
        lastPowers.fill(0.0)
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

/** Hive offset from shreyas-josyula: three inches right, then 53.369 inches forward. */
internal fun hiveApproach(hive: Pose): Pose = Pose(
    hive.x() + 3.0 * sin(hive.heading()) + 53.369 * cos(hive.heading()),
    hive.y() - 3.0 * cos(hive.heading()) + 53.369 * sin(hive.heading()), hive.heading()
)

/** Nonblocking pollen sweep / hive approach. Uses the loop's pose, never another hardware read. */
internal class PollenMacro(private val radius: Double = 10.0, private val loops: Int = 8) {
    init { require(radius.isFinite() && radius > 0.0); require(loops in 0..100) }
    private var points = emptyList<Pose>()
    private var index = 0
    var collecting = false
        private set
    val active: Boolean get() = index < points.size

    fun start(pose: Pose) {
        requireFinite(pose)
        var centerX = pose.x()
        val path = mutableListOf<Pose>()
        repeat(loops) {
            path += Pose(centerX + radius, pose.y(), pose.heading())
            for (point in 1..16) {
                val angle = Math.PI * point / 16.0
                path += Pose(centerX + radius * cos(angle), pose.y() + radius * sin(angle), pose.heading())
            }
            centerX -= radius
        }
        points = path
        index = 0
        collecting = active
    }

    fun approach(hive: Pose) {
        requireFinite(hive)
        points = listOf(hiveApproach(hive))
        index = 0
        collecting = false
    }

    fun cancel() { points = emptyList(); index = 0; collecting = false }

    /** Return true when this frame owns the drive request, including its final zero. */
    fun update(pose: Pose, drive: DriveSubsystem): Boolean {
        if (!active) return false
        requireFinite(pose)
        while (active) {
            val target = points[index]
            val dx = target.x() - pose.x()
            val dy = target.y() - pose.y()
            val distance = kotlin.math.hypot(dx, dy)
            val delta = target.heading() - pose.heading()
            val error = kotlin.math.atan2(sin(delta), cos(delta))
            if (distance <= 1.0 && abs(error) <= 0.035) { index++; continue }
            val speed = if (distance <= 1.0) 0.0 else (distance * 0.06).coerceIn(0.12, 0.35)
            val scale = if (distance > 0.0) speed / distance else 0.0
            val forward = (cos(pose.heading()) * dx + sin(pose.heading()) * dy) * scale
            val strafe = (sin(pose.heading()) * dx - cos(pose.heading()) * dy) * scale
            drive.drive(forward, strafe, (-0.8 * error).coerceIn(-0.35, 0.35))
            return true
        }
        cancel()
        drive.stop()
        return true
    }

    private fun requireFinite(pose: Pose) {
        require(pose.x().isFinite() && pose.y().isFinite() && pose.heading().isFinite()) {
            "Drive assist requires a finite pose"
        }
    }
}
