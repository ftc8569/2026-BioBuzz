package org.firstinspires.ftc.teamcode.autos

import com.pedropathing.api.Paths
import com.pedropathing.api.PoseFactory
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import java.util.function.LongSupplier
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import org.firstinspires.ftc.teamcode.teleop.IntakeFeed
import org.firstinspires.ftc.teamcode.subsystems.ShooterController
import org.firstinspires.ftc.teamcode.subsystems.ShooterController.ControlMode

/** The ri5w-autos FIFO route runner, using the current controller and guarded hardware lifecycle. */
abstract class BioBuzzMainAuto(
    private val route: MainRoutine, private val blue: Boolean,
    clock: LongSupplier = LongSupplier(System::nanoTime)
) : AutoOpMode(clock) {
    private val feed = IntakeFeed(clock::getAsLong)
    private var index = -1
    private var started = 0L
    private var entered = 0L
    private var previousFrame = 0L
    private var feedNanos = 0L
    private var feeding = false
    private var forcedPark = false
    private val step: MainStep get() = route.steps[index]

    private fun pose(x: Double, y: Double, heading: Double): Pose = PoseFactory.degrees().of(
        if (blue) 141.5 - x else x, if (blue) 141.5 - y else y,
        heading + if (blue) 180.0 else 0.0
    )
    override fun startingPose(): Pose = pose(route.x, route.y, route.heading)
    override fun buildRoutine(start: Pose) { require(route.steps.isNotEmpty()) }
    override fun startRoutine() {
        started = nanoTime.asLong
        forcedPark = false
        enter(0)
    }

    private fun enter(next: Int) {
        if (next >= route.steps.size) { stop(); return }
        index = next
        entered = nanoTime.asLong
        previousFrame = entered
        feedNanos = 0L
        feeding = false
        feed.reset()
        intake.setPower(0.0)
        // A timed-out path must not continue driving during the next shot or wait.
        follower.stop()
        follower.drivetrain.stop()
        if (step.type == StepType.DRIVE || step.type == StepType.PUSH) {
            pathTo(step)?.let { follower.follow(it) }
        } else follower.hold(follower.pose())
        updateMechanisms()
    }

    override fun updateRoutine() {
        val now = nanoTime.asLong
        // Stop at the match limit even when a route never settles or the RPM gate never opens.
        if (now - started >= 30_000_000_000L) { stop(); return }
        operation = "LOOP Pedro / Pinpoint"
        follower.update()
        if (!forcedPark && index < route.steps.lastIndex &&
            (now - started) / 1e9 >= 30.0 - route.parkSeconds) {
            forcedPark = true
            enter(route.steps.lastIndex)
            return
        }
        operation = "LOOP shooter / feed"
        if (feeding) feedNanos += (now - previousFrame).coerceAtLeast(0L)
        previousFrame = now
        updateMechanisms()
        val elapsed = (now - entered) / 1e9
        val done = when (step.type) {
            StepType.DRIVE, StepType.PUSH -> !follower.isBusy || elapsed >= step.timeout * 1.5
            StepType.SHOOT -> feedNanos / 1e9 >= step.count.coerceIn(0, 3) * 1.5 || elapsed >= 7.0
            StepType.WAIT, StepType.WAIT_TIP -> elapsed >= step.timeout
            // The source branch has no camera/tip/hopper sensor implementation. Keep its bounded fallback.
            StepType.WAIT_CELL_AUDIENCE, StepType.WAIT_CELL_REAR -> elapsed >= minOf(step.timeout, 2.5)
        }
        if (done) enter(index + 1)
    }

    private fun updateMechanisms() {
        val shooting = step.type == StepType.SHOOT
        val collecting = step.intake && !shooting
        val parking = index == route.steps.lastIndex
        val preset = ShooterController.DEFAULT_SHOOTER_PRESET
        if (collecting || parking) shooter.setFlywheelPower(0.0)
        else shooter.updateShot(preset, ControlMode.VELOCITY_LQR)
        val power = when {
            shooting -> feed.power(shooter, preset)
            collecting -> 1.0
            else -> 0.0
        }
        intake.setPower(power)
        feeding = shooting && power > 0.0
    }

    private fun pathTo(s: MainStep): Path? {
        var from = follower.pose()
        var heading = from.heading()
        val targets = s.via.map { pose(it[0], it[1], it[2]) } + pose(s.x, s.y, s.heading)
        val parts = mutableListOf<Path>()
        for (to in targets) {
            if (hypot(to.x() - from.x(), to.y() - from.y()) < 0.5) continue
            val turn = atan2(sin(to.heading() - heading), cos(to.heading() - heading))
            parts += Paths.line(from, to).linear(heading, heading + turn)
            heading += turn
            from = to
        }
        return when (parts.size) { 0 -> null; 1 -> parts[0]; else -> Paths.path(*parts.toTypedArray()) }
    }

    override fun reportTelemetry() {
        telemetry.addData("routine", "${route.name} ${if (blue) "BLUE" else "RED"}")
        telemetry.addData("step", "${index + 1}/${route.steps.size} ${step.label} ${step.type}")
        telemetry.addData("time", "%.1f s (step %.1f s)",
            (nanoTime.asLong - started) / 1e9, (nanoTime.asLong - entered) / 1e9)
        telemetry.addData("forced park", forcedPark)
        telemetry.addData("mechanisms", if (feeding) "feeding" else if (step.intake) "collecting" else "waiting / spin-up")
        telemetry.addData("sensor waits", "Timed fallback: camera, tip and hopper sensors unavailable")
        super.reportTelemetry()
    }
}

enum class StepType { DRIVE, PUSH, SHOOT, WAIT, WAIT_TIP, WAIT_CELL_AUDIENCE, WAIT_CELL_REAR }
data class MainStep(val type: StepType, val label: String, val x: Double, val y: Double,
    val heading: Double, val via: List<DoubleArray>, val count: Int, val timeout: Double, val intake: Boolean)
data class MainRoutine(val name: String, val x: Double, val y: Double, val heading: Double,
    val parkSeconds: Double, val steps: List<MainStep>)

/** Exact red-alliance route data from ri5w-autos d98058d. Blue rotates about the field center. */
object MainRoutes {
    val RIGHT_004 = MainRoutine("RIGHT_004", 7.8, 66.8, 0.0, 4.0, listOf(
        MainStep(StepType.DRIVE, "s1 driveTo", 60.6, 17.6, -90.0, emptyList(), 0, 4.0, false),
        MainStep(StepType.SHOOT, "s2 shoot", 60.6, 17.6, -90.0, emptyList(), 4, 2.5, false),
        MainStep(StepType.DRIVE, "s3 driveTo", 21.8, 47.9, 180.0, emptyList(), 0, 3.5, false),
        MainStep(StepType.DRIVE, "s4 driveTo", 14.1, 47.4, 180.0, emptyList(), 0, 1.5, true),
        MainStep(StepType.PUSH, "s5 push", 13.5, 47.3, 180.0, emptyList(), 0, 2.2, true),
        MainStep(StepType.DRIVE, "s6 driveTo", 61.1, 17.4, -90.0, emptyList(), 0, 4.0, false),
        MainStep(StepType.WAIT_CELL_AUDIENCE, "s7 waitUntil", 61.1, 17.4, -90.0, emptyList(), 0, 14.0, false),
        MainStep(StepType.SHOOT, "s8 shoot", 61.1, 17.4, -90.0, emptyList(), 4, 2.5, false),
        MainStep(StepType.DRIVE, "s9 driveTo", 60.3, 10.7, -147.0, emptyList(), 0, 2.5, true),
        MainStep(StepType.DRIVE, "s10 driveTo", 18.2, 10.4, -150.0, emptyList(), 0, 3.0, true),
        MainStep(StepType.DRIVE, "s11 driveTo", 60.7, 18.1, -90.0, emptyList(), 0, 3.5, false),
        MainStep(StepType.SHOOT, "s12 shoot", 60.7, 18.1, -90.0, emptyList(), 4, 2.5, false),
        MainStep(StepType.DRIVE, "s13 driveTo", 57.9, 46.7, -90.0, emptyList(), 0, 2.0, false),
        MainStep(StepType.WAIT, "s14 wait", 57.9, 46.7, -90.0, emptyList(), 0, 2.0, false),
        MainStep(StepType.DRIVE, "s15 driveTo", 60.2, 10.4, -150.0, emptyList(), 0, 2.5, true),
        MainStep(StepType.DRIVE, "s16 driveTo", 15.4, 10.5, -150.0, emptyList(), 0, 3.0, true),
        MainStep(StepType.DRIVE, "s17 driveTo", 58.6, 123.1, 90.0, listOf(doubleArrayOf(12.3, 15.8, 170.4), doubleArrayOf(31.5, 88.5, 90.0)), 0, 5.0, false),
        MainStep(StepType.SHOOT, "s18 shoot", 58.6, 123.1, 90.0, emptyList(), 4, 3.0, false),
        MainStep(StepType.DRIVE, "s19 driveTo", 59.3, 131.3, 92.0, emptyList(), 0, 1.5, true),
        MainStep(StepType.PUSH, "s20 push", 59.6, 131.9, 93.0, emptyList(), 0, 0.6, true),
        MainStep(StepType.DRIVE, "s21 driveTo", 58.6, 123.1, 90.0, emptyList(), 0, 2.0, false),
        MainStep(StepType.SHOOT, "s22 shoot", 58.6, 123.1, 90.0, emptyList(), 4, 1.5, false),
        MainStep(StepType.DRIVE, "s23 driveTo", 10.8, 92.5, 180.0, listOf(doubleArrayOf(59.6, 131.9, 93.3), doubleArrayOf(55.7, 123.6, 159.0)), 0, 6.0, true)
    ))

    val LEFT_034 = MainRoutine("LEFT_034", 29.4, 133.7, 90.0, 3.0, listOf(
        MainStep(StepType.DRIVE, "s1 driveTo", 36.3, 120.5, 126.0, emptyList(), 0, 3.0, false),
        MainStep(StepType.WAIT_CELL_REAR, "s2 waitUntil", 36.3, 120.5, 126.0, emptyList(), 0, 8.0, false),
        MainStep(StepType.SHOOT, "s3 shoot", 36.3, 120.5, 126.0, emptyList(), 4, 2.5, false),
        MainStep(StepType.DRIVE, "s4 driveTo", 47.9, 119.3, 91.0, emptyList(), 0, 3.0, false),
        MainStep(StepType.DRIVE, "s5 driveTo", 47.4, 127.4, 90.0, emptyList(), 0, 1.5, true),
        MainStep(StepType.PUSH, "s6 push", 47.3, 128.0, 90.0, emptyList(), 0, 2.2, true),
        MainStep(StepType.DRIVE, "s7 driveTo", 35.4, 120.9, 126.0, emptyList(), 0, 3.0, false),
        MainStep(StepType.SHOOT, "s8 shoot", 35.4, 120.9, 126.0, emptyList(), 4, 2.5, false),
        MainStep(StepType.DRIVE, "s9 driveTo", 35.0, 120.7, 127.0, emptyList(), 0, 2.5, false),
        MainStep(StepType.WAIT_TIP, "s10 waitTip", 35.0, 120.7, 127.0, emptyList(), 0, 4.0, false),
        MainStep(StepType.WAIT, "s11 wait", 35.0, 120.7, 127.0, emptyList(), 0, 2.8, false),
        MainStep(StepType.DRIVE, "s12 driveTo", 59.9, 121.2, 90.0, emptyList(), 0, 2.5, true),
        MainStep(StepType.DRIVE, "s13 driveTo", 59.2, 132.1, 88.0, emptyList(), 0, 2.0, true),
        MainStep(StepType.PUSH, "s14 push", 59.3, 133.3, 86.0, emptyList(), 0, 0.8, true),
        MainStep(StepType.DRIVE, "s15 driveTo", 35.2, 121.0, 127.0, listOf(doubleArrayOf(56.4, 124.6, 121.4)), 0, 3.0, false),
        MainStep(StepType.SHOOT, "s16 shoot", 35.2, 121.0, 127.0, emptyList(), 4, 12.0, false),
        MainStep(StepType.DRIVE, "s17 driveTo", 33.0, 131.0, 149.0, emptyList(), 0, 2.5, true),
        MainStep(StepType.DRIVE, "s18 driveTo", 11.9, 130.8, 150.0, emptyList(), 0, 3.0, true),
        MainStep(StepType.DRIVE, "s19 driveTo", 10.9, 125.9, -123.0, emptyList(), 0, 2.0, true),
        MainStep(StepType.DRIVE, "s20 driveTo", 10.8, 87.9, -120.0, emptyList(), 0, 3.0, true),
        MainStep(StepType.DRIVE, "s21 driveTo", 36.4, 121.7, 127.0, emptyList(), 0, 3.0, false),
        MainStep(StepType.SHOOT, "s22 shoot", 36.4, 121.7, 127.0, emptyList(), 4, 8.0, false),
        MainStep(StepType.DRIVE, "s23 driveTo", 10.5, 126.1, -121.0, emptyList(), 0, 2.0, true),
        MainStep(StepType.DRIVE, "s24 driveTo", 10.8, 87.9, -120.0, emptyList(), 0, 3.0, true),
        MainStep(StepType.DRIVE, "s25 driveTo", 11.2, 119.7, 180.0, emptyList(), 0, 6.0, true)
    ))

    val RIGHT_022 = MainRoutine("RIGHT_022", 7.8, 66.8, 0.0, 2.0, listOf(
        MainStep(StepType.DRIVE, "s1 driveTo", 55.0, 12.1, -90.0, emptyList(), 0, 4.0, false),
        MainStep(StepType.SHOOT, "s2 shoot", 55.0, 12.1, -90.0, emptyList(), 4, 2.5, false),
        MainStep(StepType.DRIVE, "s3 driveTo", 21.8, 48.0, 180.0, emptyList(), 0, 3.5, false),
        MainStep(StepType.DRIVE, "s4 driveTo", 14.1, 47.4, 180.0, emptyList(), 0, 1.5, true),
        MainStep(StepType.PUSH, "s5 push", 13.5, 47.3, 180.0, emptyList(), 0, 2.2, true),
        MainStep(StepType.DRIVE, "s6 driveTo", 55.5, 11.9, -90.0, emptyList(), 0, 4.0, false),
        MainStep(StepType.WAIT_CELL_AUDIENCE, "s7 waitUntil", 55.5, 11.9, -90.0, emptyList(), 0, 14.0, false),
        MainStep(StepType.SHOOT, "s8 shoot", 55.5, 11.9, -90.0, emptyList(), 4, 2.5, false),
        MainStep(StepType.DRIVE, "s9 driveTo", 60.1, 10.7, -147.0, emptyList(), 0, 2.5, true),
        MainStep(StepType.DRIVE, "s10 driveTo", 15.4, 10.6, -150.0, emptyList(), 0, 3.0, true),
        MainStep(StepType.DRIVE, "s11 driveTo", 55.2, 12.5, -90.0, emptyList(), 0, 3.5, false),
        MainStep(StepType.SHOOT, "s12 shoot", 55.2, 12.5, -90.0, emptyList(), 4, 2.5, false),
        MainStep(StepType.DRIVE, "s13 driveTo", 58.1, 46.7, -90.0, emptyList(), 0, 2.0, false),
        MainStep(StepType.WAIT, "s14 wait", 58.1, 46.7, -90.0, emptyList(), 0, 2.9, false),
        MainStep(StepType.DRIVE, "s15 driveTo", 60.0, 11.6, -151.0, emptyList(), 0, 2.5, true),
        MainStep(StepType.DRIVE, "s16 driveTo", 43.9, 10.7, -150.0, emptyList(), 0, 3.0, true),
        MainStep(StepType.DRIVE, "s17 driveTo", 58.6, 123.1, 90.0, listOf(doubleArrayOf(37.9, 12.8, -168.4), doubleArrayOf(34.4, 25.8, 115.4), doubleArrayOf(31.3, 79.0, 90.0), doubleArrayOf(38.4, 100.3, 90.0)), 0, 5.0, false),
        MainStep(StepType.SHOOT, "s18 shoot", 58.6, 123.1, 90.0, emptyList(), 4, 3.0, false),
        MainStep(StepType.DRIVE, "s19 driveTo", 59.2, 132.1, 92.0, emptyList(), 0, 1.5, true),
        MainStep(StepType.PUSH, "s20 push", 59.0, 133.3, 95.0, emptyList(), 0, 0.6, true),
        MainStep(StepType.DRIVE, "s21 driveTo", 58.0, 122.5, 90.0, emptyList(), 0, 2.0, false),
        MainStep(StepType.SHOOT, "s22 shoot", 58.0, 122.5, 90.0, emptyList(), 4, 1.5, false),
        MainStep(StepType.DRIVE, "s23 driveTo", 10.8, 92.5, 180.0, emptyList(), 0, 6.0, true)
    ))

    val LEFT_020 = MainRoutine("LEFT_020", 29.4, 133.7, 90.0, 4.0, listOf(
        MainStep(StepType.DRIVE, "s1 driveTo", 34.9, 117.6, 127.0, emptyList(), 0, 3.0, false),
        MainStep(StepType.WAIT_CELL_REAR, "s2 waitUntil", 34.9, 117.6, 127.0, emptyList(), 0, 8.0, false),
        MainStep(StepType.SHOOT, "s3 shoot", 34.9, 117.6, 127.0, emptyList(), 4, 2.5, false),
        MainStep(StepType.DRIVE, "s4 driveTo", 48.0, 119.4, 91.0, emptyList(), 0, 3.0, false),
        MainStep(StepType.DRIVE, "s5 driveTo", 47.4, 127.4, 90.0, emptyList(), 0, 1.5, true),
        MainStep(StepType.PUSH, "s6 push", 47.3, 128.0, 90.0, emptyList(), 0, 2.2, true),
        MainStep(StepType.DRIVE, "s7 driveTo", 34.3, 118.1, 127.0, emptyList(), 0, 3.0, false),
        MainStep(StepType.SHOOT, "s8 shoot", 34.3, 118.1, 127.0, emptyList(), 4, 2.5, false),
        MainStep(StepType.DRIVE, "s9 driveTo", 33.8, 117.8, 127.0, emptyList(), 0, 2.5, false),
        MainStep(StepType.WAIT_TIP, "s10 waitTip", 33.8, 117.8, 127.0, emptyList(), 0, 4.0, false),
        MainStep(StepType.WAIT, "s11 wait", 33.8, 117.8, 127.0, emptyList(), 0, 2.3, false),
        MainStep(StepType.DRIVE, "s12 driveTo", 59.9, 121.2, 90.0, emptyList(), 0, 2.5, true),
        MainStep(StepType.DRIVE, "s13 driveTo", 60.1, 128.5, 90.0, emptyList(), 0, 2.0, true),
        MainStep(StepType.PUSH, "s14 push", 60.1, 128.5, 90.0, emptyList(), 0, 0.8, true),
        MainStep(StepType.DRIVE, "s15 driveTo", 34.0, 118.2, 127.0, emptyList(), 0, 3.0, false),
        MainStep(StepType.SHOOT, "s16 shoot", 34.0, 118.2, 127.0, emptyList(), 4, 12.0, false),
        MainStep(StepType.DRIVE, "s17 driveTo", 33.0, 131.1, 150.0, emptyList(), 0, 2.5, true),
        MainStep(StepType.DRIVE, "s18 driveTo", 11.9, 130.7, 150.0, emptyList(), 0, 3.0, true),
        MainStep(StepType.DRIVE, "s19 driveTo", 10.9, 125.9, -123.0, emptyList(), 0, 2.0, true),
        MainStep(StepType.DRIVE, "s20 driveTo", 10.8, 87.9, -120.0, emptyList(), 0, 3.0, true),
        MainStep(StepType.DRIVE, "s25 driveTo", 11.2, 119.7, 180.0, emptyList(), 0, 6.0, true)
    ))

}

@Autonomous(name = "MAIN left-020 (red)", group = "BioBuzz Main")
class MainLeft020Red @JvmOverloads constructor(clock: LongSupplier = LongSupplier(System::nanoTime)) :
    BioBuzzMainAuto(MainRoutes.LEFT_020, false, clock)

@Autonomous(name = "MAIN left-020 (blue)", group = "BioBuzz Main")
class MainLeft020Blue @JvmOverloads constructor(clock: LongSupplier = LongSupplier(System::nanoTime)) :
    BioBuzzMainAuto(MainRoutes.LEFT_020, true, clock)

@Autonomous(name = "MAIN left-034 (red)", group = "BioBuzz Main")
class MainLeft034Red @JvmOverloads constructor(clock: LongSupplier = LongSupplier(System::nanoTime)) :
    BioBuzzMainAuto(MainRoutes.LEFT_034, false, clock)

@Autonomous(name = "MAIN left-034 (blue)", group = "BioBuzz Main")
class MainLeft034Blue @JvmOverloads constructor(clock: LongSupplier = LongSupplier(System::nanoTime)) :
    BioBuzzMainAuto(MainRoutes.LEFT_034, true, clock)

@Autonomous(name = "MAIN right-004 (red)", group = "BioBuzz Main")
class MainRight004Red @JvmOverloads constructor(clock: LongSupplier = LongSupplier(System::nanoTime)) :
    BioBuzzMainAuto(MainRoutes.RIGHT_004, false, clock)

@Autonomous(name = "MAIN right-004 (blue)", group = "BioBuzz Main")
class MainRight004Blue @JvmOverloads constructor(clock: LongSupplier = LongSupplier(System::nanoTime)) :
    BioBuzzMainAuto(MainRoutes.RIGHT_004, true, clock)

@Autonomous(name = "MAIN right-022 (red)", group = "BioBuzz Main")
class MainRight022Red @JvmOverloads constructor(clock: LongSupplier = LongSupplier(System::nanoTime)) :
    BioBuzzMainAuto(MainRoutes.RIGHT_022, false, clock)

@Autonomous(name = "MAIN right-022 (blue)", group = "BioBuzz Main")
class MainRight022Blue @JvmOverloads constructor(clock: LongSupplier = LongSupplier(System::nanoTime)) :
    BioBuzzMainAuto(MainRoutes.RIGHT_022, true, clock)
