package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.hardware.Gamepad
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.utility.LambdaCommand
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.junit.Assert.*
import org.junit.Test
import org.mockito.Mockito.*

class TeleOpFrameLifecycleTest {
    @Test fun `real loop reads then computes then commits before telemetry`() {
        val events = mutableListOf<String>()
        val op = mock(BioBuzzTeleOp::class.java, withSettings().useConstructor().defaultAnswer(CALLS_REAL_METHODS))
        op.telemetry = mock(Telemetry::class.java)
        op.gamepad1 = Gamepad()
        op.gamepad2 = Gamepad()
        BioBuzzTeleOp::class.java.getDeclaredField("detailedLogging").apply { isAccessible = true }.set(op, false)
        BioBuzzTeleOp::class.java.getDeclaredField("hardwareFrame").apply { isAccessible = true }
            .set(op, HardwareFrame({ events += "read" }, { events += "write" }))
        doNothing().`when`(op).onInit()
        doReturn(false).`when`(op).opModeInInit()
        doReturn(false).`when`(op).isStopRequested
        doReturn(true, true, false).`when`(op).opModeIsActive()
        doNothing().`when`(op).onStop()
        doAnswer {
            LambdaCommand("Compute").setIsDone { false }.setUpdate { events += "compute" }.schedule()
            null
        }.`when`(op).onStartButtonPressed()
        doAnswer { events += "telemetry"; true }.`when`(op.telemetry).update()
        try {
            op.runOpMode()
            assertEquals(listOf("read", "compute", "write", "telemetry", "read", "compute", "write", "telemetry"), events)
        } finally { CommandManager.cancelAll() }
    }
}
