package org.firstinspires.ftc.teamcode.control

import kotlin.math.*
import org.firstinspires.ftc.teamcode.control.FlywheelLqr
import org.firstinspires.ftc.teamcode.control.FlywheelLqrConfig
import org.junit.Assert.*
import org.junit.Test

class FlywheelLqrTest {
    private val config = FlywheelLqrConfig(0.002436, 0.000609, 150.0, 3.0)

    @Test fun `feedback gain matches an independent iterative Riccati solution`() {
        val controller = FlywheelLqr()
        val dt = 0.02
        controller.calculate(2000.0, 3000.0, dt, config)!!
        val a = exp(-config.kV / config.kA * dt)
        val b = (1-a) / config.kV
        val q = 1.0 / config.rpmTolerance.pow(2)
        val r = 1.0 / config.voltageTolerance.pow(2)
        var p = q
        repeat(10000) { p = q + a*a*p - (a*b*p).pow(2)/(r+b*b*p) }
        val expected = b*p*a/(r+b*b*p)
        assertEquals(expected, controller.feedbackGain, 1e-12)
        assertTrue(abs(a-b*controller.feedbackGain) < 1.0)
    }

    @Test fun `equilibrium voltage matches model and feedback corrects speed error`() {
        val controller = FlywheelLqr()
        val equilibrium = config.kV * 3000.0
        assertEquals(equilibrium, controller.calculate(3000.0, 3000.0, 0.02, config)!!, 1e-9)
        assertTrue(controller.calculate(2900.0, 3000.0, 0.02, config)!! > equilibrium)
        assertTrue(controller.calculate(3100.0, 3000.0, 0.02, config)!! < equilibrium)
    }

    @Test fun `simulated flywheel reaches target and recovers a shot across varying loop times and battery voltages`() {
        for (voltage in listOf(10.0, 12.0, 14.0)) {
            val controller = FlywheelLqr()
            var rpm = 0.0
            var maxRpm = 0.0
            for (i in 0..499) {
                if (i == 250) rpm -= 700.0
                val dt = listOf(0.017, 0.02, 0.03, 0.066)[i % 4]
                val volts = controller.calculate(rpm, 3000.0, dt, config)!!.coerceIn(0.0, voltage)
                val a = exp(-config.kV/config.kA * dt)
                rpm = a*rpm + (1-a)/config.kV*volts
                maxRpm = max(maxRpm, rpm)
                assertTrue(rpm.isFinite())
            }
            assertEquals(3000.0, rpm, 1.0)
            assertTrue(maxRpm < 3150.0)
        }
    }

    @Test fun `live tuning changes feedback and invalid models or samples produce no valid command`() {
        val controller = FlywheelLqr()
        val first = controller.calculate(2900.0, 3000.0, 0.02, config)!!
        val tighter = controller.calculate(2900.0, 3000.0, 0.02, config.copy(rpmTolerance = 50.0))!!
        assertTrue(tighter > first)
        for (bad in listOf(0.0, -1.0, Double.NaN, Double.POSITIVE_INFINITY)) {
            assertNull(controller.calculate(0.0, 3000.0, 0.02, config.copy(kV=bad)))
            assertNull(controller.calculate(0.0, 3000.0, 0.02, config.copy(kA=bad)))
            assertNull(controller.calculate(0.0, 3000.0, 0.02, config.copy(rpmTolerance=bad)))
            assertNull(controller.calculate(0.0, 3000.0, 0.02, config.copy(voltageTolerance=bad)))
            assertNull(controller.calculate(0.0, 3000.0, bad, config))
        }
        assertNull(controller.calculate(Double.NaN, 3000.0, 0.02, config))
        assertNull(controller.calculate(0.0, Double.POSITIVE_INFINITY, 0.02, config))
    }
}
