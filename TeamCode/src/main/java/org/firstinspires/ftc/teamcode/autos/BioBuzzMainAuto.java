package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * Runs a Routine step by step: drive, shoot, wait. Poses are red alliance; blue is the red pose turned 180 degrees
 * about the field center. Pedro inches, field 141.5 x 141.5, heading in degrees CCW from +x.
 * Compiled against the Pedro Pathing source (core), never run on a robot.
 */
public abstract class BioBuzzMainAuto extends OpMode {
    // ---- tune these ----
    /** The simulator's step time limits are for its robot; allow this much longer on the real one. */
    private static final double TIMEOUT_SCALE = 1.5;
    /** A shot gets flywheel spin-up plus up to 5 s of feeding (3 balls take 4-5 s), whatever the sim's limit was. */
    private static final double SHOOT_TIMEOUT_SEC = 7.0;
    /** With no camera hook, wait at most this long for the raised CELL's tags instead of the sim's 8-14 s. */
    private static final double VISION_FALLBACK_SEC = 2.5;
    private static final double AUTO_SEC = 30.0;
    private static final double FIELD_IN = 141.5;   // the field is 141.5 in square; center 70.75

    /** false = drive the paths only, with the shooter and intake left alone (for a first test on the field). */
    private static final boolean USE_MECHANISMS = true;

    protected abstract Routine routine();
    protected abstract boolean isBlue();

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private Follower follower;
    private Mechanisms mech;
    private Routine routine;
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stepTimer = new ElapsedTime();
    private int index = -1;
    private boolean forcedPark = false;
    private boolean done = false;

    private Pose pose(double x, double y, double headingDeg) {
        return isBlue() ? poseFactory.of(FIELD_IN - x, FIELD_IN - y, headingDeg + 180) : poseFactory.of(x, y, headingDeg);
    }

    @Override
    public void init() {
        routine = routine();
        follower = Constants.create(hardwareMap);
        follower.setPose(pose(routine.startX, routine.startY, routine.startHeadingDeg));
        follower.update();
        mech = USE_MECHANISMS ? new RobotMechanisms(hardwareMap) : new NoMechanisms();
    }

    @Override
    public void start() {
        autoTimer.reset();
        mech.spinUpShooter();
        advance();
    }

    @Override
    public void loop() {
        follower.update();
        mech.update();
        if (!done) tick();
        AutoStep s = (index >= 0 && index < routine.steps.size()) ? routine.steps.get(index) : null;
        telemetry.addData("routine", routine.name + (isBlue() ? " BLUE" : " RED"));
        telemetry.addData("step", s == null ? "done" : (index + 1) + "/" + routine.steps.size() + "  " + s.label + "  " + s.type);
        telemetry.addData("time", "%.1f s (step %.1f s)", autoTimer.seconds(), stepTimer.seconds());
        telemetry.addData("pose", "x %.1f  y %.1f  h %.0f", follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()));
        telemetry.addData("mechanisms", mech.status());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (mech != null) mech.stopAll();
    }

    // ---- step runner ----
    private void tick() {
        int last = routine.steps.size() - 1;
        if (!forcedPark && index < last && autoTimer.seconds() >= AUTO_SEC - routine.finishAtSec) {
            forcedPark = true;
            leave(routine.steps.get(index));
            enter(last);
            return;
        }
        AutoStep s = routine.steps.get(index);
        if (finished(s)) { leave(s); advance(); }
    }

    private void advance() {
        if (index + 1 >= routine.steps.size()) { done = true; mech.stopShooting(); return; }
        enter(index + 1);
    }

    private void enter(int i) {
        index = i;
        AutoStep s = routine.steps.get(i);
        stepTimer.reset();
        mech.intake(s.intake);
        switch (s.type) {
            case DRIVE:
            case PUSH:
                Path p = pathTo(s);
                if (p != null) startFollowing(p);
                break;
            case SHOOT:
                mech.startShooting(s.count);
                break;
            default:
                break;
        }
    }

    private void leave(AutoStep s) {
        if (s.type == AutoStep.Type.SHOOT) mech.stopShooting();
    }

    private boolean finished(AutoStep s) {
        double t = stepTimer.seconds();
        switch (s.type) {
            case DRIVE:
            case PUSH:
                return !followerBusy() || (s.intake && mech.hopperFull()) || t > s.timeoutSec * TIMEOUT_SCALE;
            case SHOOT:
                return mech.shootingDone() || t > SHOOT_TIMEOUT_SEC;
            case WAIT:
                return t >= s.timeoutSec;
            case WAIT_TIP:
                return mech.tipLanded() || t >= s.timeoutSec;
            case WAIT_CELL_AUDIENCE:
                return mech.seesRaisedCell(false) || t >= (mech.hasVision() ? s.timeoutSec : Math.min(s.timeoutSec, VISION_FALLBACK_SEC));
            case WAIT_CELL_REAR:
                return mech.seesRaisedCell(true) || t >= (mech.hasVision() ? s.timeoutSec : Math.min(s.timeoutSec, VISION_FALLBACK_SEC));
            default:
                return true;
        }
    }

    /** A line to each via point and then the target. The heading turns the short way on each piece. Null if already there. */
    private Path pathTo(AutoStep s) {
        Pose from = follower.pose();
        List<Pose> chain = new ArrayList<>();
        for (double[] v : s.via) chain.add(pose(v[0], v[1], v[2]));
        chain.add(pose(s.x, s.y, s.headingDeg));
        List<Path> parts = new ArrayList<>();
        Pose a = from;
        double h = from.heading();
        for (Pose b : chain) {
            if (Math.hypot(b.x() - a.x(), b.y() - a.y()) < 0.5) continue;   // too short to be a path
            double turn = Math.atan2(Math.sin(b.heading() - h), Math.cos(b.heading() - h));
            parts.add(Paths.line(a, b).linear(h, h + turn));
            h += turn;
            a = b;
        }
        if (parts.isEmpty()) return null;
        return parts.size() == 1 ? parts.get(0) : Paths.path(parts.toArray(new Path[0]));
    }

    private void startFollowing(Path path) { follower.follow(path); }
    private boolean followerBusy() { return follower.isBusy(); }
}
