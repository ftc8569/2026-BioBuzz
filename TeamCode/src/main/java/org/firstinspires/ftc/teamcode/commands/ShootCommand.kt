package org.firstinspires.ftc.teamcode.commands

import com.pedropathing.follower.Follower
import com.pedropathing.ivy.CommandBuilder
import org.firstinspires.ftc.teamcode.Subsytem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem.ControlMode
import org.firstinspires.ftc.teamcode.teleop.BlockerFeedDelay
import org.firstinspires.ftc.teamcode.teleop.FeedControls

/** Hold position and perform right-bumper shooting, finishing after one second of transfer. */
class ShootCommand @JvmOverloads constructor(
    follower: Follower,
    shooter: ShooterSubsystem,
    intake: IntakeSubsystem,
    power: Double = ShooterSubsystem.DEFAULT_SHOOTER_PRESET,
    nanoTime: () -> Long = System::nanoTime
) : CommandBuilder() {
    init {
        require(power.isFinite() && power > 0.0 && power <= 1.0)
        var finished = false
        var transferStarted: Long? = null
        var blockerDelay = BlockerFeedDelay(nanoTime)
        fun stopAll() {
            intake.setPower(0.0)
            shooter.setManualBlockerOpen(false)
            shooter.setFlywheelPower(0.0)
            follower.stop()
        }
        requiring(follower, shooter, intake)
        setStart {
            finished = false
            transferStarted = null
            blockerDelay = BlockerFeedDelay(nanoTime)
            stopAll()
            shooter.setControlMode(ControlMode.VELOCITY_LQR)
            follower.hold(follower.pose())
        }
        setExecute {
            if (!finished) {
                shooter.updateShot(power, ControlMode.VELOCITY_LQR)
                val requested = FeedControls.intakePower(
                    stop = false, manualPower = 0.0, shooterPower = power,
                    preset = 1.0, mode = ControlMode.VELOCITY_LQR,
                    feedReady = shooter.feedReady, manualBlockerOpen = false
                )
                val transferPower = blockerDelay.apply(requested, shooter.blockerOpen)
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
