package org.firstinspires.ftc.teamcode.teleop

import org.junit.Assert.*
import org.junit.Test

class DriveIntakeControlsTest {
    @Test fun `enabled by default and only toggles on a new press`() {
        val controls = DriveIntakeControls()
        assertTrue(controls.enabled)
        controls.update(true)
        assertFalse(controls.enabled)
        controls.update(true)
        assertFalse(controls.enabled)
        controls.update(false)
        controls.update(true)
        assertTrue(controls.enabled)
    }

    @Test fun `minimum requires drive magnitude strictly above threshold and enabled toggle`() {
        val controls = DriveIntakeControls()
        for (power in listOf(0.0, 0.19, 0.2, -0.2)) {
            assertEquals(0.0, controls.minimumPower(power), 0.0)
        }
        assertEquals(0.2, controls.minimumPower(0.201), 0.0)
        assertEquals(0.2, controls.minimumPower(-0.8), 0.0)
        controls.update(true)
        assertEquals(0.0, controls.minimumPower(0.9), 0.0)
    }
}
