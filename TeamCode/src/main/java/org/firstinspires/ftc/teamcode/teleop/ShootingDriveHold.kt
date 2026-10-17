package org.firstinspires.ftc.teamcode.teleop

import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose

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
