package org.firstinspires.ftc.teamcode.teleop

import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.components.Component
import org.firstinspires.ftc.teamcode.diagnostics.LoopProfiler
import org.firstinspires.ftc.teamcode.diagnostics.LoopStage
import org.junit.Assert.*
import org.junit.Test
import org.mockito.Mockito.*

class TeleOpSchedulerTest {
    @Test fun `real NextFTC lifecycle updates each control once per loop and measures all loop work`() {
        var now = 0L
        val profiler = LoopProfiler { now }
        val opMode = mock(BioBuzzTeleOp::class.java, withSettings().useConstructor().defaultAnswer(CALLS_REAL_METHODS))
        BioBuzzTeleOp::class.java.getDeclaredField("profiler")
            .apply { isAccessible = true }.set(opMode, profiler)
        val counts = mutableListOf<Pair<Int, Int>>()
        var driveUpdates = 0
        var mechanismUpdates = 0
        var stoppedCommands = 0
        CommandManager.cancelAll()
        // Run the actual framework loop locally; replace only hardware-facing hooks.
        doNothing().`when`(opMode).onInit()
        doReturn(false).`when`(opMode).opModeInInit()
        doReturn(false).`when`(opMode).isStopRequested
        doReturn(true, true, true, false).`when`(opMode).opModeIsActive()
        doAnswer {
            LambdaCommand("Test drive")
                .setIsDone { driveUpdates >= 3 }
                .setUpdate { profiler.measure(LoopStage.DRIVE_TOTAL) { driveUpdates++; now += 2_000_000 } }
                .setStop { stoppedCommands++ }
                .schedule()
            LambdaCommand("Test mechanisms")
                .setIsDone { mechanismUpdates >= 3 }
                .setUpdate { profiler.measure(LoopStage.MECHANISMS_TOTAL) { mechanismUpdates++; now += 3_000_000 } }
                .setStop { stoppedCommands++ }
                .schedule()
            null
        }.`when`(opMode).onStartButtonPressed()
        doAnswer {
            counts += driveUpdates to mechanismUpdates
            now += 5_000_000
            null
        }.`when`(opMode).onUpdate()
        doNothing().`when`(opMode).onStop()
        opMode.addComponents(object : Component {
            override fun postUpdate() { now += 6_000_000 }
        })
        try {
            opMode.runOpMode()
            assertEquals(listOf(1 to 1, 2 to 2, 3 to 3), counts)
            assertEquals(2, stoppedCommands)
            assertFalse(CommandManager.hasCommands())
            now += 1_000_000_000
            val samples = profiler.snapshotIfDue()!!.associateBy { it.stage }
            val total = samples.getValue(LoopStage.LOOP_WORK)
            assertEquals(3L, total.calls)
            // 2 ms drive + 3 ms mechanisms + 5 ms telemetry + 6 ms post-update work.
            assertEquals(16.0, total.averageMs, 0.0)
            assertEquals(16.0, total.maxMs, 0.0)
            assertEquals(3L, samples.getValue(LoopStage.DRIVE_TOTAL).calls)
            assertEquals(3L, samples.getValue(LoopStage.MECHANISMS_TOTAL).calls)
        } finally {
            CommandManager.cancelAll()
        }
    }
    @Test fun `delegated shutdown still cancels a command that remains active`() {
        val opMode = mock(BioBuzzTeleOp::class.java, withSettings().useConstructor().defaultAnswer(CALLS_REAL_METHODS))
        var updates = 0
        var updatesAtStop = 0
        var stopped = false
        CommandManager.cancelAll()
        doNothing().`when`(opMode).onInit()
        doReturn(false).`when`(opMode).opModeInInit()
        doReturn(false).`when`(opMode).isStopRequested
        doReturn(true, true, true, false).`when`(opMode).opModeIsActive()
        doAnswer {
            LambdaCommand("Persistent control")
                .setIsDone { false }
                .setUpdate { updates++ }
                .setStop { stopped = true }
                .schedule()
            null
        }.`when`(opMode).onStartButtonPressed()
        doNothing().`when`(opMode).onUpdate()
        doAnswer { updatesAtStop = updates; null }.`when`(opMode).onStop()
        try {
            opMode.runOpMode()
            assertEquals(3, updatesAtStop)
            assertTrue(stopped)
            assertFalse(CommandManager.hasCommands())
        } finally {
            CommandManager.cancelAll()
        }
    }

}
