package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import org.junit.Test
import org.mockito.Mockito.*

class BulkInputReaderTest {
    private fun motor(connection: String) = mock(DcMotor::class.java).also { motor ->
        val controller = mock(DcMotorController::class.java)
        `when`(controller.connectionInfo).thenReturn(connection)
        `when`(motor.controller).thenReturn(controller)
    }
    @Test fun `fetches each needed hub once and skips diagnostics-only and unused hubs when hidden`() {
        // Hubs may share a USB serial; connection info includes the distinct module address.
        val hubs = listOf("USB module 1", "USB module 2", "USB module 3").map { connection ->
            mock(LynxModule::class.java).also { `when`(it.connectionInfo).thenReturn(connection) }
        }
        val reader = BulkInputReader(hubs,
            listOf(motor("USB module 1"), motor("USB module 1")),
            listOf(motor("USB module 1"), motor("USB module 2")))
        clearInvocations(*hubs.toTypedArray())
        reader.read(false)
        verify(hubs[0]).bulkData
        verifyNoMoreInteractions(*hubs.toTypedArray())
        clearInvocations(*hubs.toTypedArray())
        reader.read(true)
        verify(hubs[0]).bulkData
        verify(hubs[1]).bulkData
        verifyNoMoreInteractions(*hubs.toTypedArray())
    }
}
