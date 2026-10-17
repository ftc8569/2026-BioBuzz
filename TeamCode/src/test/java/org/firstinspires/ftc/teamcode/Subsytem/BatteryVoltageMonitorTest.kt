package org.firstinspires.ftc.teamcode.Subsytem

import com.qualcomm.robotcore.hardware.VoltageSensor
import org.junit.Assert.*
import org.junit.Test
import org.mockito.Mockito.*
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

class BatteryVoltageMonitorTest {
    @Test fun `publishes lowest valid voltage and falls back to nominal`() {
        val first = mock(VoltageSensor::class.java)
        val second = mock(VoltageSensor::class.java)
        val monitor = BatteryVoltageMonitor(listOf(first, second))
        assertEquals(12.0, monitor.voltage, 0.0)
        `when`(first.voltage).thenReturn(14.0)
        `when`(second.voltage).thenReturn(10.0)
        monitor.pollOnce()
        assertEquals(10.0, monitor.voltage, 0.0)
        for (invalid in listOf(0.0, -1.0, Double.NaN, Double.POSITIVE_INFINITY)) {
            `when`(first.voltage).thenReturn(invalid)
            `when`(second.voltage).thenReturn(invalid)
            monitor.pollOnce()
            assertEquals(12.0, monitor.voltage, 0.0)
        }
    }

    @Test fun `failed sensor does not prevent sampling other sensors`() {
        val first = mock(VoltageSensor::class.java)
        val second = mock(VoltageSensor::class.java)
        `when`(first.voltage).thenThrow(IllegalStateException("unavailable"))
        `when`(second.voltage).thenReturn(11.0)
        val monitor = BatteryVoltageMonitor(listOf(first, second))
        monitor.pollOnce()
        assertEquals(11.0, monitor.voltage, 0.0)
    }

    @Test fun `worker repeatedly publishes voltage without main loop polling`() {
        val sensor = mock(VoltageSensor::class.java)
        val samples = CountDownLatch(2)
        `when`(sensor.voltage).thenAnswer {
            samples.countDown()
            11.0
        }
        val monitor = BatteryVoltageMonitor(listOf(sensor))
        try {
            monitor.start()
            assertTrue(samples.await(2, TimeUnit.SECONDS))
            // The first sample is published before the second poll starts.
            assertEquals(11.0, monitor.voltage, 0.0)
        } finally {
            monitor.close()
        }
    }

    @Test fun `blocked sensor runs off thread and close interrupts polling`() {
        val sensor = mock(VoltageSensor::class.java)
        val entered = CountDownLatch(1)
        val stopped = CountDownLatch(1)
        `when`(sensor.voltage).thenAnswer {
            entered.countDown()
            try {
                CountDownLatch(1).await()
                10.0
            } finally {
                stopped.countDown()
            }
        }
        val monitor = BatteryVoltageMonitor(listOf(sensor))
        try {
            monitor.start()
            monitor.start() // Idempotent: no second worker.
            assertTrue(entered.await(2, TimeUnit.SECONDS))
            assertEquals(12.0, monitor.voltage, 0.0)
        } finally {
            monitor.close()
        }
        assertTrue(stopped.await(2, TimeUnit.SECONDS))
        monitor.close()
        monitor.start() // A closed monitor cannot start another worker.
        verify(sensor, times(1)).getVoltage()
    }
}
