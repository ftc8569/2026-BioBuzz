package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.junit.Assert.*
import org.junit.Test
import org.mockito.ArgumentMatchers.anyDouble
import org.mockito.Mockito.*

class MoveForwardLeftTest {
    private class Rig {
        var now = 0L
        val powers = DoubleArray(4) { Double.NaN }
        val motors = List(4) { mock(DcMotor::class.java) }
        val auto = MoveForwardLeft { now }
        init {
            val map = mock(HardwareMap::class.java)
            for ((i, name) in listOf("driveFL", "driveFR", "driveBL", "driveBR").withIndex()) {
                `when`(map.get(DcMotor::class.java, name)).thenReturn(motors[i])
                doAnswer { powers[i] = it.getArgument(0); null }.`when`(motors[i]).setPower(anyDouble())
            }
            auto.hardwareMap = map
            auto.init()
        }
    }

    @Test fun `starts only on Start drives diagonally at half power and stops at 250 milliseconds`() {
        val rig = Rig()
        assertArrayEquals(doubleArrayOf(0.0, 0.0, 0.0, 0.0), rig.powers, 0.0)
        for ((i, motor) in rig.motors.withIndex()) {
            verify(motor).direction = if (i == 0 || i == 2)
                DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
            verify(motor).mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            verify(motor).zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
        rig.now = 10_000_000_000L // Waiting in INIT cannot consume the motion duration.
        rig.auto.start()
        assertArrayEquals(doubleArrayOf(0.0, 0.5, 0.5, 0.0), rig.powers, 0.0)
        rig.now += 249_999_999L
        rig.auto.loop()
        assertArrayEquals(doubleArrayOf(0.0, 0.5, 0.5, 0.0), rig.powers, 0.0)
        rig.now += 1L
        rig.auto.loop()
        assertArrayEquals(doubleArrayOf(0.0, 0.0, 0.0, 0.0), rig.powers, 0.0)
        clearInvocations(*rig.motors.toTypedArray())
        rig.now += 1_000_000_000L
        rig.auto.loop()
        verifyNoInteractions(*rig.motors.toTypedArray())
    }

    @Test fun `early STOP zeros all wheels and cannot resume motion`() {
        val rig = Rig()
        rig.auto.start()
        rig.now = 100_000_000L
        rig.auto.stop()
        assertArrayEquals(doubleArrayOf(0.0, 0.0, 0.0, 0.0), rig.powers, 0.0)
        clearInvocations(*rig.motors.toTypedArray())
        rig.auto.loop()
        verifyNoInteractions(*rig.motors.toTypedArray())
    }
}
