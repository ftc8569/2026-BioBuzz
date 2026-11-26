package org.firstinspires.ftc.teamcode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.ftc.NextFTCOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.subsystems.ShooterController.Companion.BLOCKER_CLOSED
import org.firstinspires.ftc.teamcode.subsystems.ShooterController.Companion.BLOCKER_OPEN
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem

// Test OpModes (group "Test") on NextFTC. None moves the blocker before START; telemetry
// refreshes every TELEMETRY_INTERVAL_MS and outputs are written only when their value changes.

/** Blocker servo only: A closes, B opens. Remove game pieces first. */
@TeleOp(name = "Blocker Diagnostic", group = "Test")
class BlockerDiagnosticTeleOp : NextFTCOpMode() {
    private lateinit var blocker: Servo
    private val refresh = TelemetryRefresh()
    private var position: Double? = null // null until START: the servo is untouched in INIT.

    override fun onInit() { blocker = hardwareMap.get(Servo::class.java, "blocker") }
    override fun onStartButtonPressed() = set(BLOCKER_CLOSED)
    override fun onUpdate() {
        if (gamepad1.a) set(BLOCKER_CLOSED) else if (gamepad1.b) set(BLOCKER_OPEN)
        if (refresh.isDue()) { telemetry.addData("A: close / B: open", "%.2f", position); telemetry.update() }
    }
    override fun onStop() { if (position != null) blocker.position = BLOCKER_CLOSED }
    private fun set(value: Double) { if (value != position) { blocker.position = value; position = value } }
}

/** Raw shooter motor test: no controller, intake or blocker movement. Clear the wheels first. */
@TeleOp(name = "Shooter Motor Diagnostic", group = "Test")
class ShooterMotorDiagnosticTeleOp : NextFTCOpMode() {
    private lateinit var motors: List<DcMotorEx>
    private lateinit var blocker: Servo
    private val safety = SafeHardware()
    private val refresh = TelemetryRefresh()
    private val power = DoubleArray(2)
    private var initial = IntArray(2)
    private var setup = ""

    override fun onInit() = guarded {
        val hubs = hardwareMap.getAll(LynxModule::class.java)
        safety.setCaching(hubs, LynxModule.BulkCachingMode.OFF) // Fresh reads for wiring checks.
        blocker = hardwareMap.get(Servo::class.java, "blocker")
        motors = listOf("shooter1", "shooter2").map { safety.motor(hardwareMap, it) }
        motors.forEachIndexed { i, m ->
            m.direction = if (i == 0) DcMotorSimple.Direction.FORWARD else DcMotorSimple.Direction.REVERSE
            m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            m.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            m.power = 0.0
        }
        initial = IntArray(2) { motors[it].currentPosition }
        // Configuration never changes during a run, so format it once.
        setup = motors.joinToString(" / ") {
            "${it.connectionInfo}, ${it.motorType.name}, ${it.motorType.ticksPerRev.toInt()} ticks/rev, " +
                "max ${it.motorType.achieveableMaxTicksPerSecond.toInt()} ticks/s"
        }
    }

    override fun onStartButtonPressed() = guarded {
        safety.onStop { blocker.position = BLOCKER_CLOSED }
        blocker.position = BLOCKER_CLOSED
    }

    override fun onUpdate() = guarded {
        // Hold X or Y for one motor, B for both. Right trigger raises power to 0.80.
        val level = 0.25 + 0.55 * gamepad1.right_trigger.toDouble().coerceIn(0.0, 1.0)
        val wanted = doubleArrayOf(if (gamepad1.x || gamepad1.b) level else 0.0,
            if (gamepad1.y || gamepad1.b) level else 0.0)
        for (i in 0..1) if (wanted[i] != power[i]) { motors[i].power = wanted[i]; power[i] = wanted[i] }
        if (!refresh.isDue()) return@guarded // Encoder reads below are uncached: skip them between refreshes.
        telemetry.addLine("Hold X: motor 1 | Y: motor 2 | B: both | RT: raise power")
        telemetry.addData("Power 1 / 2", "%.2f / %.2f", power[0], power[1])
        telemetry.addData("Ticks/s 1 / 2", "%.0f / %.0f", motors[0].velocity, motors[1].velocity)
        telemetry.addData("Ticks moved 1 / 2", "%d / %d",
            motors[0].currentPosition - initial[0], motors[1].currentPosition - initial[1])
        telemetry.addData("Setup 1 / 2", setup)
        telemetry.addLine("Spinning with zero ticks: check encoder cables and ports.")
        telemetry.update()
    }

    private inline fun guarded(action: () -> Unit) {
        try { action() } catch (failure: Throwable) { throw safety.stop(failure)!! }
    }
    override fun onStop() { safety.stop(null)?.let { throw it } }
}

/** Optional Limelight check. The camera starts only when this OpMode is started. */
@Configurable
@TeleOp(name = "Vision Diagnostic", group = "Test")
class VisionDiagnosticTeleOp : NextFTCOpMode() {
    private var vision: VisionSubsystem? = null
    private val refresh = TelemetryRefresh()
    private var failure: String? = null

    override fun onInit() = guarded("INIT camera") {
        vision = VisionSubsystem.optional(hardwareMap)
        telemetry.msTransmissionInterval = TELEMETRY_INTERVAL_MS.toInt()
        telemetry.addData("Vision status", if (vision == null) "No configured limelight" else "Ready; START begins camera")
        telemetry.update()
    }
    override fun onStartButtonPressed() = guarded("START camera") { vision?.start(pipelineIndex) }
    override fun onUpdate() = guarded("LOOP camera") {
        val camera = vision
        val frame = camera?.update()
        if (!refresh.isDue()) return@guarded
        telemetry.addData("Vision status", (frame?.status ?: "No configured limelight") as Any)
        telemetry.addData("Visible tags", camera?.getVisibleTagIds() ?: emptyList<Int>())
        telemetry.addData("Frame age (ms)", frame?.ageMillis ?: 0)
        frame?.estimate?.let {
            telemetry.addData("Hive robot pose (m / rad)", it.robot)
            telemetry.addData("Hive fit residual (m)", it.residualMeters)
        }
        telemetry.update()
    }
    override fun onStop() { vision.also { vision = null }?.stop() }

    private inline fun guarded(phase: String, action: () -> Unit) {
        failure?.let {
            if (refresh.isDue()) { telemetry.addData("Vision error", it as Any); telemetry.update() }
            return
        }
        try { action() } catch (error: RuntimeException) {
            try { onStop() } catch (cleanup: RuntimeException) { if (error !== cleanup) error.addSuppressed(cleanup) }
            failure = "$phase: ${error.message}"
            RobotLog.ee("BioBuzzVision", error, failure)
            telemetry.addData("Vision error", failure as Any?)
            telemetry.update()
        }
    }
    companion object { @JvmField var pipelineIndex = 0 }
}

/** Match shooter and controls with no drive, intake, odometry or camera. INIT stays stopped. */
@TeleOp(name = "Shooter Tuning", group = "Test")
class ShooterTuningTeleOp : NextFTCOpMode() {
    private val pads = GamepadSnapshot(this)
    private val safety = SafeHardware()
    private val controls = ShooterControls()
    private val errorRefresh = TelemetryRefresh()
    private var hubs = emptyList<LynxModule>()
    private var shooter: ShooterSubsystem? = null
    private var display: RobotTelemetry? = null
    private var started = false
    private var failure: String? = null
    private val up: Boolean get() = pads.g1.dpad_up || pads.g2.dpad_up
    private val down: Boolean get() = pads.g1.dpad_down || pads.g2.dpad_down

    init { addComponents(pads, BindingsComponent) }

    override fun onInit() = guarded("INIT") {
        hubs = hardwareMap.getAll(LynxModule::class.java)
        safety.setCaching(hubs, LynxModule.BulkCachingMode.MANUAL)
        // Acquire independent stop handles before the subsystem can partially initialize.
        var error: Throwable? = null
        for (name in listOf("shooter1", "shooter2")) error = SafeHardware.attempt(error) {
            safety.motor(hardwareMap, name).power = 0.0
        }
        SafeHardware.rethrow(error)
        shooter = ShooterSubsystem(hardwareMap).also {
            safety.onStop { it.forceStop() }
            safety.onStop { it.stopVoltagePolling() }
            it.enablePhasedIo()
        }
        display = RobotTelemetry(telemetry, PanelsTelemetry.ftcTelemetry)
        // D-pad preset works during INIT; manual mode (Y) only after START, as before.
        controls.bind({ started && pads.g1.y }, { up }, { down })
        report()
    }
    override fun onWaitForStart() = guarded("INIT") { report() }
    override fun onStartButtonPressed() = guarded("START") {
        shooter?.apply { startBlockerControl(); startVoltagePolling() } ?: return@guarded
        started = true
    }
    override fun onUpdate() = guarded("LOOP") {
        val device = shooter?.takeIf { started } ?: return@guarded
        hubs.forEach { it.clearBulkCache() } // One owner and one invalidation per active loop.
        device.beginFrame()
        val power = controls.requestedPower(pads.g1.left_bumper, pads.g1.right_bumper)
        device.setControlMode(controls.mode)
        if (power > 0.0) device.updateShot(power, controls.mode) else device.setFlywheelPower(power)
        device.setManualBlockerOpen(pads.g1.b || pads.g2.b)
        device.commitOutputs()
        report()
    }
    private fun report() {
        val device = shooter ?: return
        display?.publish {
            addData("Shooter tuning", if (started) "ACTIVE" else "READY: outputs stopped")
            addData("Controls", "Hold RB: LQR | Y: manual | LB: reverse | D-pad: preset | B: manual blocker")
            addData("Live LQR settings", "Panels > ShooterLqrTuning")
            reportShooter(device, controls)
        }
    }
    override fun onStop() {
        started = false
        SafeHardware.rethrow(safety.stop(null))
    }
    private inline fun guarded(phase: String, action: () -> Unit) {
        if (failure == null) {
            try { action() } catch (error: Throwable) {
                shooter = null
                started = false
                safety.stop(error)
                failure = "$phase: ${error.javaClass.simpleName}: ${error.message}"
                RobotLog.ee("BioBuzzShooterTuning", error, failure)
                errorRefresh.reset()
            }
        }
        failure?.let {
            if (errorRefresh.isDue()) { telemetry.addData("Shooter tuning error", it as Any); telemetry.update() }
        }
    }
}
