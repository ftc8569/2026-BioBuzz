package org.firstinspires.ftc.teamcode.autos

import com.pedropathing.api.Paths
import com.pedropathing.api.PoseFactory
import com.pedropathing.follower.Follower
import com.pedropathing.ivy.CommandBuilder
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import java.util.function.LongSupplier
import org.firstinspires.ftc.teamcode.subsystems.ShooterController
import org.firstinspires.ftc.teamcode.subsystems.ShooterController.ControlMode
import org.firstinspires.ftc.teamcode.teleop.SafeHardware
import org.firstinspires.ftc.teamcode.subsystems.IntakeFeed
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem

/** Same two-second shot as BasicRed; coordinates come from basicblue.pp. */
@Autonomous(name = "BasicBlue Auto")
class BasicBlueAuto @JvmOverloads constructor(
    clock: LongSupplier = LongSupplier(System::nanoTime)
) : BioBuzzAuto(clock) {
    override fun startingPose(): Pose =
        PoseFactory.degrees().of(83.07841726618705, 132.19424460431654, 90.0)
    override fun buildPath(start: Pose): Path =
        Paths.line(start, PoseFactory.degrees().of(132.9597122302158, 41.497122302158274, 90.0))
            .constant(Math.toRadians(90.0))
}

@Autonomous(name = "BioBuzz Auto Blue")
class BioBuzzAutoBlue : BioBuzzRed1_5(true)

@Autonomous(name = "Backup Blue Left", group = "Backup")
class BackupAutoBlueLeft : BioBuzzBackupAuto(true, 112.1, 7.8, -90.0, 0.0, 35.5)

@Autonomous(name = "Backup Blue Right", group = "Backup")
class BackupAutoBlueRight : BioBuzzBackupAuto(true, 133.7, 74.7, 180.0, 360.0, 35.5)

@Autonomous(name = "Backup Red Left", group = "Backup")
class BackupAutoRedLeft : BioBuzzBackupAuto(false, 29.4, 133.7, 90.0, 180.0, 106.0)

@Autonomous(name = "Backup Red Right", group = "Backup")
class BackupAutoRedRight : BioBuzzBackupAuto(false, 7.8, 66.8, 0.0, 180.0, 106.0)

/** Hold position and perform right-bumper shooting, finishing after one second of transfer. */
class ShootCommand @JvmOverloads constructor(
    follower: Follower,
    shooter: ShooterSubsystem,
    intake: IntakeSubsystem,
    power: Double = ShooterController.DEFAULT_SHOOTER_PRESET,
    nanoTime: () -> Long = System::nanoTime
) : CommandBuilder() {
    init {
        require(power.isFinite() && power > 0.0 && power <= 1.0)
        var finished = false
        var transferStarted: Long? = null
        val feed = IntakeFeed(nanoTime)
        fun stopAll() {
            var failure = SafeHardware.attempt(null) { intake.forceStop() }
            failure = SafeHardware.attempt(failure) { shooter.forceStop() }
            failure = SafeHardware.attempt(failure) { follower.stop() }
            SafeHardware.rethrow(failure)
        }
        requiring(follower, shooter, intake)
        setStart {
            finished = false
            transferStarted = null
            feed.reset()
            stopAll()
            shooter.setControlMode(ControlMode.VELOCITY_LQR)
            follower.hold(follower.pose())
        }
        setExecute {
            if (!finished) {
                shooter.updateShot(power, ControlMode.VELOCITY_LQR)
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
        }
        setDone { finished }
        setEnd { stopAll() }
    }
}
