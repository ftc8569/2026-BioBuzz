package org.firstinspires.ftc.teamcode.teleop

import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.hardware.ShootingDriveHold
import org.junit.Assert.*
import org.junit.Test
import org.mockito.Mockito.*

class ShootingDriveHoldTest {
    @Test fun `holds entry pose until shooting ends and recaptures for next shot`() {
        val follower = mock(Follower::class.java)
        val hold = ShootingDriveHold(follower)
        val first = Pose(10.0, 20.0, 0.5)
        val drifted = Pose(11.0, 22.0, 0.6)
        assertFalse(hold.update(false, first))
        verifyNoInteractions(follower)
        assertTrue(hold.update(true, first))
        assertTrue(hold.update(true, drifted))
        verify(follower).hold(first)
        verify(follower, never()).hold(drifted)
        verify(follower, times(2)).update()
        assertFalse(hold.update(false, drifted))
        verify(follower).stop()
        assertTrue(hold.update(true, drifted))
        verify(follower).hold(drifted)
        hold.stop()
        assertFalse(hold.active)
        verify(follower, times(2)).stop()
    }
}
