package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.robot.Mechanism
import com.pedropathing.localization.Localizer
import com.pedropathing.localization.MotionState
import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.teleop.hiveApproach

/** Pedro sees the same immutable pose/velocity sample as every other controller. */
internal class SnapshotLocalizer : Localizer {
    private var snapshot = MotionState.zero()
    fun capture(state: MotionState) { snapshot = state }
    override fun state(): MotionState = snapshot
    override fun update() = Unit
    override fun setPose(pose: Pose) { snapshot = snapshot.withPose(pose) }
    override fun reset() { snapshot = MotionState.zero() }
}

/** Location subsystem from the branch, adapted to the shared read phase and current Pedro units.
 * Getters, hive saves, aiming and the follower all use the same sample; none polls Pinpoint.
 */
class RobotLocationSubsystem : Mechanism {
    enum class Alliance { BLUE, RED }
    private val snapshot = SnapshotLocalizer()
    internal val localizer: Localizer get() = snapshot
    val pose: Pose get() = snapshot.pose()
    private val hives = arrayOfNulls<Pose>(Alliance.entries.size)

    internal fun capture(state: MotionState) {
        val pose = state.pose()
        require(pose.x().isFinite() && pose.y().isFinite() && pose.heading().isFinite()) {
            "Pinpoint returned a non-finite pose"
        }
        snapshot.capture(state)
    }

    fun savedHive(alliance: Alliance): Pose? = hives[alliance.ordinal]
    fun saveHive(alliance: Alliance) { hives[alliance.ordinal] = pose }
    fun approachTarget(alliance: Alliance): Pose? = savedHive(alliance)?.let(::hiveApproach)
    fun headingToHive(alliance: Alliance): Double? = savedHive(alliance)?.let {
        kotlin.math.atan2(it.y() - pose.y(), it.x() - pose.x())
    }

    internal fun reset() { snapshot.reset(); hives.fill(null) }
}
