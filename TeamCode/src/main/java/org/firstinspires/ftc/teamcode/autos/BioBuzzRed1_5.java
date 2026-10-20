package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.autos.ShootCommand;
import static com.pedropathing.ivy.groups.Groups.sequential;

/** Shoot, collect on path 1, drive path 2, shoot again, then drive path 3. */
@Autonomous(name = "BioBuzzRed1.5")
public class BioBuzzRed1_5 extends AutoOpMode {
    private Path path1;
    private Path path2;
    private Path path3;
    private Command routine;
    private final PoseFactory poses = PoseFactory.degrees();
    private static final double FIELD_SIZE_IN = 141.5;
    private final boolean blue;

    public BioBuzzRed1_5() {
        this(false);
    }

    protected BioBuzzRed1_5(boolean blue) {
        this.blue = blue;
    }

    /** Match the backup autos' 180-degree alliance rotation about field center. */
    private Pose pose(double x, double y, double headingDegrees) {
        return poses.of(blue ? FIELD_SIZE_IN - x : x, blue ? FIELD_SIZE_IN - y : y,
                headingDegrees + (blue ? 180.0 : 0.0));
    }

    @Override protected Pose startingPose() { return pose(58.4934, 9.0389, -90.0); }

    @Override protected void buildRoutine(Pose startPose) {
        Pose path1End = pose(10.7034, 8.5727, -165.0);
        Pose path1Control1 = pose(34.4053, 12.9376, 0.0);
        Pose path2Start = pose(10.7034, 8.5727, -115.0);
        Pose path2End = pose(58.1696, 132.804, 90.0);
        Pose path2Control1 = pose(64.9361, 49.5595, 0.0);
        Pose path2Control2 = pose(-0.2438, 135.2562, 0.0);
        Pose path2Control3 = pose(63.3781, 115.8686, 0.0);
        Pose path3End = pose(12.3128, 115.8693, 4.3906);
        Pose path3Control1 = pose(57.986, 119.3084, 0.0);

        Scheduler.reset();
        follower.holdEnd.set(false);
        path1 = Paths.curve(startPose, path1Control1, path1End).linear(startPose, path1End);
        path2 = Paths.curve(path2Start, path2Control1, path2Control2, path2Control3, path2End)
                .linear(path2Start, path2End);
        path3 = Paths.curve(path2End, path3Control1, path3End).reverseTangent();
    }

    @Override
    protected void startRoutine() {
        routine = sequential(
                new ShootCommand(follower, shooter, intake),
                followPath(path1, 1.0),
                followPath(path2, 0.0),
                new ShootCommand(follower, shooter, intake),
                followPath(path3, 0.0));
        Scheduler.schedule(routine);
    }

    /** Scope collection to the path and wait for Pedro to finish following before advancing. */
    private Command followPath(Path path, double intakePower) {
        return new CommandBuilder()
                .requiring(follower, intake)
                .setStart(() -> {
                    intake.setPower(intakePower);
                    follower.follow(path);
                })
                .setDone(() -> !follower.following())
                .setEnd(condition -> {
                    intake.setPower(0.0);
                    if (condition != EndCondition.NATURALLY) follower.stop();
                });
    }

    @Override
    protected void updateRoutine() {
        operation = "LOOP routine";
        Scheduler.execute();
        operation = "LOOP Pedro / Pinpoint";
        follower.update();
    }

    @Override protected void reportTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("Path state", follower.mode());
        telemetry.addData("X (in)", pose.x());
        telemetry.addData("Y (in)", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
    }

    @Override protected void stopRoutine() {
        try { if (routine != null) Scheduler.cancel(routine); }
        finally { Scheduler.reset(); }
    }
}
