package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotor

/** Resolve ownership during init; no controller metadata getters in the active loop. */
internal class BulkInputReader(
    hubs: List<LynxModule>, feedback: List<DcMotor>, diagnostics: List<DcMotor>
) {
    // SDK LynxController.getConnectionInfo delegates directly to its owning module.
    // Unlike USB serial alone, this distinguishes hubs on the same RS485 chain.
    private val feedbackConnections = feedback.map { it.controller.connectionInfo }.toSet()
    private val diagnosticConnections = diagnostics.map { it.controller.connectionInfo }.toSet()
    private val feedbackHubs = hubs.filter { it.connectionInfo in feedbackConnections }
    private val diagnosticHubs = hubs.filter { it !in feedbackHubs && it.connectionInfo in diagnosticConnections }

    fun read(includeDiagnostics: Boolean) {
        // SDK getBulkData clears and populates its cache. MANUAL motor getters then
        // decode this response without initiating another hub transaction.
        feedbackHubs.forEach { it.bulkData }
        if (includeDiagnostics) diagnosticHubs.forEach { it.bulkData }
    }
}
