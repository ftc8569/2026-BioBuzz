package org.firstinspires.ftc.teamcode.autodog

import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetSocketAddress
import java.net.SocketAddress
import java.net.SocketTimeoutException
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Robot-controller side of the auto dog UDP link (design doc 2.8).
 *
 * "Pi" below means whatever runs python/autodog_run.py: a Pi on the robot,
 * or a laptop on the robot's Wi-Fi reading the Limelight stream.
 *
 * Mirrors python/autodog/link.py. Little-endian, no padding, first byte is
 * the message type:
 *   'P' pose    RC -> Pi   t_rc, x, y, theta (doubles)               33 bytes
 *   'S' ping    Pi -> RC   seq (u32), t0                             13 bytes
 *   'R' pong    RC -> Pi   seq (u32), t0, t1, t2                     29 bytes
 *   'T' target  Pi -> RC   t_rc, has (u8), id (u32), x, y, vx, vy,
 *                          color (1 ASCII char)                      47 bytes
 *
 * Poses and targets are in the tracker's field frame: origin at a field
 * corner, meters, heading counterclockwise from +x. Use [FieldFrame] to
 * convert from the team's center-origin odometry.
 *
 * Time is this process's monotonic clock in seconds ([clock]); the Pi
 * estimates the offset to its own clock with the ping/pong handshake.
 *
 * @param piHost Pi address. When null the link learns it from the first ping.
 */
class AutoDogLink(
    piHost: String?,
    private val piPort: Int = DEFAULT_PI_PORT,
    localPort: Int = DEFAULT_RC_PORT,
    val clock: () -> Double = { System.nanoTime() / 1e9 },
) : AutoCloseable {
    private val socket = DatagramSocket(localPort).apply { soTimeout = 50 }

    @Volatile
    private var pi: SocketAddress? = piHost?.let { InetSocketAddress(it, piPort) }

    /** Most recent target message, or null before the first one arrives. */
    @Volatile
    var latestTarget: AutoDogTarget? = null
        private set

    @Volatile
    var pingsAnswered: Int = 0
        private set

    @Volatile
    var badPackets: Int = 0
        private set

    @Volatile
    private var running = true

    val localPort: Int get() = socket.localPort

    private val receiver = Thread({ receiveLoop() }, "AutoDogLink").apply {
        isDaemon = true
        start()
    }

    /** Stream one odometry sample. Call every loop (the doc assumes ~50 Hz). */
    fun sendPose(x: Double, y: Double, theta: Double, tRc: Double = clock()) {
        val dest = pi ?: return
        val buf = ByteBuffer.allocate(POSE_SIZE).order(ByteOrder.LITTLE_ENDIAN)
        buf.put('P'.code.toByte()).putDouble(tRc).putDouble(x).putDouble(y).putDouble(theta)
        send(buf.array(), dest)
    }

    private fun send(bytes: ByteArray, dest: SocketAddress) {
        try {
            socket.send(DatagramPacket(bytes, bytes.size, dest))
        } catch (_: Exception) {
            // UDP is fire and forget; a dropped pose only thins the buffer.
        }
    }

    private fun receiveLoop() {
        val data = ByteArray(256)
        val packet = DatagramPacket(data, data.size)
        while (running) {
            try {
                packet.setData(data)
                socket.receive(packet)
            } catch (_: SocketTimeoutException) {
                continue
            } catch (_: Exception) {
                if (!running) return
                continue
            }
            val t1 = clock()
            handle(packet, t1)
        }
    }

    private fun handle(packet: DatagramPacket, t1: Double) {
        val len = packet.length
        val buf = ByteBuffer.wrap(packet.data, 0, len).order(ByteOrder.LITTLE_ENDIAN)
        when (buf.get().toInt().toChar()) {
            'S' -> {
                if (len != PING_SIZE) return run { badPackets++ }
                val seq = buf.int
                val t0 = buf.double
                if (pi == null) {
                    // Learn the Pi's address; poses go to its listening port.
                    val from = packet.socketAddress as InetSocketAddress
                    pi = InetSocketAddress(from.address, piPort)
                }
                val out = ByteBuffer.allocate(PONG_SIZE).order(ByteOrder.LITTLE_ENDIAN)
                out.put('R'.code.toByte()).putInt(seq).putDouble(t0).putDouble(t1).putDouble(clock())
                send(out.array(), packet.socketAddress)
                pingsAnswered++
            }
            'T' -> {
                if (len != TARGET_SIZE) return run { badPackets++ }
                val tRc = buf.double
                val has = buf.get().toInt() != 0
                val id = buf.int
                val x = buf.double
                val y = buf.double
                val vx = buf.double
                val vy = buf.double
                val color = buf.get().toInt().toChar()
                latestTarget = AutoDogTarget(tRc, has, id, x, y, vx, vy, color, receivedAt = t1)
            }
            else -> badPackets++
        }
    }

    override fun close() {
        running = false
        socket.close()
        receiver.join(500)
    }

    companion object {
        const val DEFAULT_PI_PORT = 47800
        const val DEFAULT_RC_PORT = 47801
        const val POSE_SIZE = 33
        const val PING_SIZE = 13
        const val PONG_SIZE = 29
        const val TARGET_SIZE = 47
    }
}

/** A ball the Pi wants us to chase, as of [tRc] (RC clock, seconds). */
data class AutoDogTarget(
    val tRc: Double,
    val hasTarget: Boolean,
    val trackId: Int,
    val x: Double,
    val y: Double,
    val vx: Double,
    val vy: Double,
    val color: Char,
    val receivedAt: Double,
) {
    /** Constant-velocity prediction to time [t] (doc 2.10: never act on stale state). */
    fun predictedTo(t: Double): Pair<Double, Double> {
        val dt = (t - tRc).coerceIn(0.0, 0.5)
        return Pair(x + vx * dt, y + vy * dt)
    }

    fun age(now: Double): Double = now - tRc
}

/**
 * Converts between the team's field frame (origin at field center, as in
 * docs/hive-vision.md) and the tracker's frame (origin at a field corner).
 * Both have x/y in the floor plane and counterclockwise heading; the team
 * frame is in inches on the robot (RobotLocationSubsystem), the tracker's in meters.
 */
object FieldFrame {
    const val INCH = 0.0254
    const val FIELD_SIZE_M = 144 * INCH
    const val HALF = FIELD_SIZE_M / 2

    /** Team center-origin inches -> tracker corner-origin meters. */
    fun inchesToTracker(xInches: Double, yInches: Double): Pair<Double, Double> =
        toTracker(xInches * INCH, yInches * INCH)

    fun toTracker(xCenter: Double, yCenter: Double): Pair<Double, Double> =
        Pair(xCenter + HALF, yCenter + HALF)

    fun fromTracker(xCorner: Double, yCorner: Double): Pair<Double, Double> =
        Pair(xCorner - HALF, yCorner - HALF)
}
