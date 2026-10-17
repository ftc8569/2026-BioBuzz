package org.firstinspires.ftc.teamcode.commands

import com.pedropathing.ivy.behaviors.EndCondition
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.Subsytem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem
import org.firstinspires.ftc.teamcode.Subsytem.ShooterSubsystem.ControlMode
import org.junit.Assert.*
import org.junit.Test
import org.mockito.ArgumentMatchers.anyDouble
import org.mockito.Mockito.*

class ShootCommandTest {
    private class Rig {
        val follower = mock(Follower::class.java)
        val shooter = mock(ShooterSubsystem::class.java)
        val intake = mock(IntakeSubsystem::class.java)
        var now = 0L
        var ready = false
        var power = 0.0
        val command = ShootCommand(follower, shooter, intake, nanoTime = { now })
        init {
            `when`(follower.pose()).thenReturn(Pose(0.0, 0.0, 0.0))
            `when`(shooter.feedReady).thenAnswer { ready }
            `when`(shooter.blockerOpen).thenAnswer { ready }
            doAnswer { power = it.getArgument(0); null }.`when`(intake).setPower(anyDouble())
            command.start()
        }
        fun step(ms: Long) { now = ms * 1_000_000; command.execute() }
    }

    @Test fun `waits for speed and blocker then transfers for exactly one second`() {
        val rig = Rig()
        verify(rig.follower).hold(rig.follower.pose())
        rig.step(5000)
        assertEquals(0.0, rig.power, 0.0)
        assertFalse(rig.command.done())
        verify(rig.shooter).updateShot(0.72, ControlMode.VELOCITY_LQR)
        rig.ready = true
        rig.step(6000)
        rig.step(6099)
        assertEquals(0.0, rig.power, 0.0)
        rig.step(6100)
        assertEquals(1.0, rig.power, 0.0)
        rig.step(7099)
        assertEquals(1.0, rig.power, 0.0)
        assertFalse(rig.command.done())
        rig.step(7100)
        assertTrue(rig.command.done())
        rig.command.end(EndCondition.NATURALLY)
        assertEquals(0.0, rig.power, 0.0)
        verify(rig.shooter, atLeastOnce()).setFlywheelPower(0.0)
        verify(rig.shooter, atLeastOnce()).setManualBlockerOpen(false)
    }

    @Test fun `cancel stops transfer and command can be reused with fresh timers`() {
        val rig = Rig()
        rig.ready = true
        rig.step(0)
        rig.step(100)
        assertEquals(1.0, rig.power, 0.0)
        rig.command.end(EndCondition.INTERRUPTED)
        assertEquals(0.0, rig.power, 0.0)
        verify(rig.shooter, atLeastOnce()).setFlywheelPower(0.0)
        rig.command.start()
        rig.step(100)
        assertEquals(0.0, rig.power, 0.0)
        assertFalse(rig.command.done())
        rig.step(200)
        assertEquals(1.0, rig.power, 0.0)
        assertFalse(rig.command.done())
    }
    @Test fun `readiness loss stops transfer and restarts its timing after recovery`() {
        val rig = Rig()
        rig.ready = true
        rig.step(0)
        rig.step(100)
        rig.step(800)
        assertFalse(rig.command.done())
        rig.ready = false
        rig.step(900)
        assertEquals(0.0, rig.power, 0.0)
        rig.ready = true
        rig.step(1000)
        rig.step(1100)
        rig.step(2099)
        assertFalse(rig.command.done())
        rig.step(2100)
        assertTrue(rig.command.done())
    }
}
