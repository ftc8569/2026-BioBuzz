package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Base auto: drives one straight line from the start pose to the end pose.
 * Pedro inches, heading in degrees CCW from +x.
 */
@Autonomous(name = "BioBuzz Auto")
public class BioBuzzAuto extends OpMode {
    private Follower follower;

    protected Pose startingPose() {
        return PoseFactory.degrees().of(57.01798561151079, 9.221582733812928, 90);
    }

    protected Path buildPath(Pose start) {
        Pose end = PoseFactory.degrees().of(8.96906474820144, 97.8935251798561, 270);
        return Paths.line(start, end).constant(Math.toRadians(270));
    }

    @Override
    public void init() {
        Scheduler.reset();
        Pose start = startingPose();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        Scheduler.schedule(sequential(follow(follower, buildPath(start))));
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.stop();
            follower.update();
        }
    }
}
