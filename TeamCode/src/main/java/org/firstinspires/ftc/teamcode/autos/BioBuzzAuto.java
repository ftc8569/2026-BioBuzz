package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "BioBuzz Auto")
public class BioBuzzAuto extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    // TODO: replace with your real poses, in inches and degrees (Pedro coordinates).
    // These are the docs' example values, not your field positions.
    private final Pose startPose = p.of(24, 24, 0);
    private final Pose score = p.of(36, 36, 45);
    private final Pose park = p.of(48, 48, 90);

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        // TODO: build the path from startPose to park (Creating a Path)
    }

    @Override
    public void start() {
        // TODO: begin following the path (Following a Path)
    }

    @Override
    public void loop() {
        // TODO: update the follower each loop (Following a Path)
    }
}