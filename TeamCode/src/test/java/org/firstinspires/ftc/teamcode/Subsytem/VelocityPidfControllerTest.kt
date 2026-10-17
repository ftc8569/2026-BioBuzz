package org.firstinspires.ftc.teamcode.Subsytem

import org.junit.Assert.*
import org.junit.Test
import org.junit.After

class VelocityPidfControllerTest {
    @After fun restoreLiveGains() {
        ShooterPidfTuning.kP = VelocityPidfController.kP
        ShooterPidfTuning.kI = VelocityPidfController.kI
        ShooterPidfTuning.kD = 0.0
        ShooterPidfTuning.kF = VelocityPidfController.kF
    }

    @Test fun `Panels gain changes affect an existing controller immediately`() {
        val controller = VelocityPidfController()
        assertEquals(0.64, controller.calculate(2880.0, 2880.0, 0.02), 1e-12)
        ShooterPidfTuning.kP = 0.0
        ShooterPidfTuning.kI = 0.0
        ShooterPidfTuning.kF = 0.5 / 4500.0
        assertEquals(0.32, controller.calculate(2880.0, 2000.0, 0.02), 1e-12)
    }

    @Test fun `live derivative damps measured acceleration without target kick`() {
        ShooterPidfTuning.kP = 0.0
        ShooterPidfTuning.kI = 0.0
        ShooterPidfTuning.kD = 0.00001
        val controller = VelocityPidfController()
        assertEquals(0.64, controller.calculate(2880.0, 2500.0, 0.02), 1e-12)
        assertEquals(0.59, controller.calculate(2880.0, 2600.0, 0.02), 1e-12)
        assertEquals(0.66, controller.calculate(2970.0, 2600.0, 0.02), 1e-12)
        controller.reset()
        assertEquals(0.64, controller.calculate(2880.0, 2700.0, 0.0), 1e-12)
    }

    @Test fun `invalid Panels gains fall back to finite defaults`() {
        ShooterPidfTuning.kP = Double.NaN
        ShooterPidfTuning.kI = -1.0
        ShooterPidfTuning.kD = Double.POSITIVE_INFINITY
        ShooterPidfTuning.kF = Double.NaN
        assertEquals(0.64, VelocityPidfController().calculate(2880.0, 2880.0, 0.02), 1e-12)
    }

    @Test fun `64 percent target commands full spinup and strong shot recovery`() {
        val controller = VelocityPidfController()
        val target = 2880.0
        assertEquals(1.0, controller.calculate(target, 0.0, 0.0), 0.0)
        assertEquals(1.0, controller.calculate(target, target - 900.0, 0.0), 0.0)
        assertEquals(0.64, controller.calculate(target, target, 0.0), 1e-12)
        assertEquals(0.24, controller.calculate(target, target + 900.0, 0.0), 1e-12)
    }

    @Test fun `RPM feedforward holds target and corrects speed error in the right direction`() {
        val controller = VelocityPidfController()
        assertEquals(0.66, controller.calculate(2970.0, 2970.0, 0.0), 1e-12)
        assertTrue(controller.calculate(2970.0, 2500.0, 0.0) > 0.66)
        assertTrue(controller.calculate(2970.0, 3400.0, 0.0) < 0.66)
        assertEquals(1.0, controller.calculate(4500.0, 4500.0, 0.0), 1e-12)
    }

    @Test fun `saturation does not wind up integral and reset clears accumulated correction`() {
        val controller = VelocityPidfController()
        repeat(1000) { assertEquals(1.0, controller.calculate(4050.0, 0.0, 0.02), 0.0) }
        assertEquals(0.9, controller.calculate(4050.0, 4050.0, 0.0), 1e-12)
        repeat(100) { controller.calculate(2250.0, 2025.0, 0.02) }
        assertTrue(controller.calculate(2250.0, 2250.0, 0.0) > 0.5)
        controller.reset()
        assertEquals(0.5, controller.calculate(2250.0, 2250.0, 0.0), 1e-12)
        assertEquals(0.0, controller.calculate(2250.0, 9000.0, 0.0), 0.0)
    }

    @Test fun `simulated flywheel reaches RPM and recovers after battery sag and ball impact`() {
        val controller = VelocityPidfController()
        val target = 2970.0
        var rpm = 0.0
        var power = 0.0
        repeat(3000) { step ->
            if (step == 1500) rpm -= 750.0
            val freeRpm = 4500.0 * if (step >= 500) 0.85 else 1.0
            power = controller.calculate(target, rpm, 0.02)
            assertTrue(power.isFinite() && power in 0.0..1.0)
            rpm += (power * freeRpm - rpm) * 0.02 / 0.3
        }
        assertEquals(target, rpm, target * 0.01)
        assertTrue(power > 0.66)
    }

    @Test(expected = IllegalArgumentException::class)
    fun `invalid feedback never returns NaN motor power`() {
        VelocityPidfController().calculate(2970.0, Double.NaN, 0.02)
    }
}
