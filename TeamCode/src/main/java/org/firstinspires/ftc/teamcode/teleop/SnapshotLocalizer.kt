package org.firstinspires.ftc.teamcode.teleop

import com.pedropathing.localization.Localizer
import com.pedropathing.localization.MotionState
import com.pedropathing.math.Pose

/** Pedro sees the same immutable pose/velocity sample as every other controller. */
internal class SnapshotLocalizer : Localizer {
    private var snapshot = MotionState.zero()
    fun capture(state: MotionState) { snapshot = state }
    override fun state(): MotionState = snapshot
    override fun update() = Unit
    override fun setPose(pose: Pose) { snapshot = snapshot.withPose(pose) }
    override fun reset() { snapshot = MotionState.zero() }
}
