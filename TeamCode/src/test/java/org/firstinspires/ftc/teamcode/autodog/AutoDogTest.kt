package org.firstinspires.ftc.teamcode.autodog

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Subsytem.DriveSubsystem
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test
import org.mockito.Mockito.mock
import org.mockito.Mockito.verify
import org.mockito.Mockito.`when`
import org.mockito.ArgumentCaptor
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetSocketAddress
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.PI
import kotlin.math.abs

class AutoDogTest {

    @Test
    fun `ball dead ahead drives straight with no turn`() {
        val p = ChaseController().compute(1.0, 1.0, 0.0, 2.0, 1.0)
        assertEquals(0.0, p.turn, 1e-9)
        assertEquals(0.0, p.left, 1e-9)
        assertEquals(0.5, p.forward, 1e-9) // capped at maxForward
    }

    @Test
    fun `ball behind turns in place`() {
        val p = ChaseController().compute(1.0, 1.0, 0.0, 0.0, 1.05)
        assertEquals(0.0, p.forward, 1e-9)
        assertTrue(abs(p.turn) == 0.5) // saturated
    }

    @Test
    fun `turn direction follows counterclockwise convention`() {
        // robot faces +y; ball to its left (-x) needs a CCW (positive) turn
        val p = ChaseController().compute(1.0, 1.0, PI / 2, 0.5, 1.5)
        assertTrue(p.turn > 0)
        // and a ball to its right needs a negative turn
        assertTrue(ChaseController().compute(1.0, 1.0, PI / 2, 1.5, 1.5).turn < 0)
    }

    @Test
    fun `slows down as the ball reaches the intake`() {
        val c = ChaseController()
        val far = c.compute(0.0, 0.0, 0.0, 1.5, 0.0).forward
        val near = c.compute(0.0, 0.0, 0.0, 0.3, 0.0).forward
        assertTrue(near < far && near > 0)
    }

    @Test
    fun `strafes toward a close ball off to the side`() {
        val p = ChaseController().compute(0.0, 0.0, 0.0, 0.4, 0.1)
        assertTrue(p.left > 0)
        assertEquals(0.0, ChaseController().compute(0.0, 0.0, 0.0, 2.0, 0.5).left, 1e-9)
    }

    @Test
    fun `field frame conversion round trips`() {
        val (x, y) = FieldFrame.toTracker(-1.580126, 1.598982) // blue goal
        assertEquals(0.2486, x, 1e-3)
        assertEquals(3.4278, y, 1e-3)
        val (bx, by) = FieldFrame.fromTracker(x, y)
        assertEquals(-1.580126, bx, 1e-12)
        assertEquals(1.598982, by, 1e-12)
    }

    @Test
    fun `team inches convert to tracker meters`() {
        val (x, y) = FieldFrame.inchesToTracker(0.0, 0.0) // field center
        assertEquals(72 * 0.0254, x, 1e-12)
        assertEquals(72 * 0.0254, y, 1e-12)
        val (cx, cy) = FieldFrame.inchesToTracker(-72.0, -72.0)
        assertEquals(0.0, cx, 1e-12)
        assertEquals(0.0, cy, 1e-12)
    }

    @Test
    fun `ball on the left turns the real drivetrain counterclockwise`() {
        val map = mock(HardwareMap::class.java)
        val motors = listOf("driveFL", "driveFR", "driveBL", "driveBR").associateWith {
            mock(DcMotor::class.java).also { m -> `when`(map.get(DcMotor::class.java, it)).thenReturn(m) }
        }
        val drive = DriveSubsystem(map)
        // robot at origin facing +x, ball far off to the left: pure CCW turn
        val p = ChaseController().compute(0.0, 0.0, 0.0, 0.0, 2.0)
        assertTrue(p.turn > 0)
        drive.driveChase(p)
        fun last(name: String): Double {
            val c = ArgumentCaptor.forClass(Double::class.java)
            verify(motors.getValue(name), org.mockito.Mockito.atLeastOnce()).power = c.capture()
            return c.value
        }
        // CCW in place: left wheels backward, right wheels forward (logical power)
        assertTrue(last("driveFL") < 0 && last("driveBL") < 0)
        assertTrue(last("driveFR") > 0 && last("driveBR") > 0)
    }

    @Test
    fun `link answers pings, streams poses and receives targets`() {
        val fakePi = DatagramSocket(0).apply { soTimeout = 1000 }
        val link = AutoDogLink(piHost = null, piPort = fakePi.localPort, localPort = 0)
        try {
            val rc = InetSocketAddress("127.0.0.1", link.localPort)

            // ping -> pong, and the link learns where the Pi is
            val ping = ByteBuffer.allocate(13).order(ByteOrder.LITTLE_ENDIAN)
                .put('S'.code.toByte()).putInt(7).putDouble(123.5).array()
            fakePi.send(DatagramPacket(ping, ping.size, rc))
            val pong = receive(fakePi)
            assertEquals(29, pong.remaining() + 1)
            assertEquals(7, pong.int)
            assertEquals(123.5, pong.double, 0.0)
            val t1 = pong.double
            val t2 = pong.double
            assertTrue(t2 >= t1)

            // poses now go to the Pi
            link.sendPose(1.0, 2.0, 0.5, tRc = 42.0)
            val pose = receive(fakePi, expectType = 'P')
            assertEquals(42.0, pose.double, 0.0)
            assertEquals(1.0, pose.double, 0.0)
            assertEquals(2.0, pose.double, 0.0)
            assertEquals(0.5, pose.double, 0.0)

            // target in
            val tgt = ByteBuffer.allocate(47).order(ByteOrder.LITTLE_ENDIAN)
                .put('T'.code.toByte()).putDouble(10.0).put(1).putInt(9)
                .putDouble(1.5).putDouble(2.5).putDouble(0.2).putDouble(-0.4)
                .put('G'.code.toByte()).array()
            fakePi.send(DatagramPacket(tgt, tgt.size, rc))
            val deadline = System.nanoTime() + 1_000_000_000L
            while (link.latestTarget == null && System.nanoTime() < deadline) Thread.sleep(2)
            val t = link.latestTarget
            assertNotNull(t)
            t!!
            assertTrue(t.hasTarget)
            assertEquals(9, t.trackId)
            assertEquals('G', t.color)
            val (px, py) = t.predictedTo(10.5)
            assertEquals(1.6, px, 1e-12)
            assertEquals(2.3, py, 1e-12)
            assertEquals(1, link.pingsAnswered)
        } finally {
            link.close()
            fakePi.close()
        }
    }

    private fun receive(sock: DatagramSocket, expectType: Char? = null): ByteBuffer {
        val buf = ByteArray(256)
        val pkt = DatagramPacket(buf, buf.size)
        sock.receive(pkt)
        val bb = ByteBuffer.wrap(buf, 0, pkt.length).order(ByteOrder.LITTLE_ENDIAN)
        val type = bb.get().toInt().toChar()
        if (expectType != null) assertEquals(expectType, type)
        return bb
    }
}
