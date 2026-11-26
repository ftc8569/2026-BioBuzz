package org.firstinspires.ftc.teamcode.autos

import com.pedropathing.api.Paths
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple

/** Routes from updates/preload-park-blocker-timing; shared LQR and two-second feed are unchanged. */
open class PreloadParkAuto(private val blue: Boolean, private val close: Boolean) : BioBuzzAuto() {
    private fun pose(x: Double, y: Double, heading: Double) = alliancePose(blue, x, y, heading)
    override fun startingPose(): Pose = pose(58.4934, if (close) 6.6 else 9.0389, -90.0)
    override fun buildPath(start: Pose): Path {
        val control = if (close) pose(58.2467, 97.8414, 0.0) else pose(57.8311, 121.1131, 0.0)
        val end = if (close) pose(11.3267, 94.1791, 90.0) else pose(12.3656, 117.243, -90.0)
        return Paths.curve(start, pose(10.3025, 9.6131, 0.0), control, end).linear(start, end)
    }
}

/** Quarter-second diagonal move. Needs only four drive motors, with the same guarded cleanup. */
@Autonomous(name = "MoveForwardLeft")
class MoveForwardLeft : AutoOpMode() {
    private var motors = emptyList<DcMotorEx>()
    private var started = 0L
    override fun usesShooter() = false
    override fun usesFollower() = false
    override fun startingPose() = Pose(0.0, 0.0, 0.0)
    override fun buildRoutine(start: Pose) {
        // SafeHardware already retained each motor before any setup write can fail.
        motors = listOf("driveFL", "driveFR", "driveBL", "driveBR")
            .map { hardwareMap.get(DcMotorEx::class.java, it) }
        motors.forEachIndexed { i, motor ->
            motor.direction = if (i == 0 || i == 2) DcMotorSimple.Direction.REVERSE
                else DcMotorSimple.Direction.FORWARD
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }
    override fun startRoutine() {
        started = nanoTime.asLong
        motors.forEachIndexed { i, motor -> motor.power = if (i == 1 || i == 2) 0.5 else 0.0 }
    }
    override fun updateRoutine() {
        if (nanoTime.asLong - started >= 250_000_000L) stop()
    }
}

@Autonomous(name = "RedPreloadParkClose")
class RedPreloadParkClose : PreloadParkAuto(blue = false, close = true)

@Autonomous(name = "RedPreloadParkFar")
class RedPreloadParkFar : PreloadParkAuto(blue = false, close = false)

@Autonomous(name = "BluePreloadParkClose")
class BluePreloadParkClose : PreloadParkAuto(blue = true, close = true)

@Autonomous(name = "BluePreloadParkFar")
class BluePreloadParkFar : PreloadParkAuto(blue = true, close = false)
