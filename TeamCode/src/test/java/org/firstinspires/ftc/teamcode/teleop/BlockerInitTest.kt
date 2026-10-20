package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.junit.Test
import org.mockito.ArgumentMatchers.anyDouble
import org.mockito.Mockito.*

class BlockerInitTest {
    private fun diagnostic(motorDiagnostic: Boolean): Pair<OpMode, Servo> {
        val map = mock(HardwareMap::class.java)
        val servo = mock(Servo::class.java)
        `when`(map.get(Servo::class.java, "blocker")).thenReturn(servo)
        `when`(map.getAll(LynxModule::class.java)).thenReturn(emptyList())
        for (name in listOf("shooter1", "shooter2"))
            `when`(map.get(DcMotorEx::class.java, name)).thenReturn(mock(DcMotorEx::class.java))
        val op: OpMode = if (motorDiagnostic) ShooterMotorDiagnosticTeleOp() else BlockerDiagnosticTeleOp()
        op.hardwareMap = map
        op.telemetry = mock(Telemetry::class.java)
        op.gamepad1 = Gamepad()
        return op to servo
    }

    @Test fun `diagnostics leave blocker untouched on INIT and STOP before Start`() {
        for (motorDiagnostic in listOf(false, true)) {
            val (op, servo) = diagnostic(motorDiagnostic)
            op.init()
            verify(servo, never()).setPosition(anyDouble())
            op.stop()
            verify(servo, never()).setPosition(anyDouble())
        }
    }

    @Test fun `diagnostics close blocker at Start and still close on normal STOP`() {
        for (motorDiagnostic in listOf(false, true)) {
            val (op, servo) = diagnostic(motorDiagnostic)
            op.init()
            verify(servo, never()).setPosition(anyDouble())
            op.start()
            verify(servo).position = 0.24
            if (!motorDiagnostic) {
                op.gamepad1.b = true
                op.loop()
                verify(servo).position = 0.45
            }
            op.stop()
            verify(servo, times(2)).position = 0.24
        }
    }
}
