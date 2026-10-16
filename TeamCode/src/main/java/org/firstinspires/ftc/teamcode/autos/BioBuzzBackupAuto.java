package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Backup auto: one straight line from the start pose to the loading zone (LEAVE + PARK). The robot
 * is placed FACING its alliance wall and keeps that heading the whole way (it slides sideways, no
 * turning), so its corners never swing into the wall. No obstacle avoidance.
 * Pedro inches, field 141.5 x 141.5, heading in degrees CCW from +x.
 */
public abstract class BioBuzzBackupAuto extends OpMode {
    // TODO: set to the real robot length in inches.
    protected static final double ROBOT_LENGTH_IN = 18;
    private static final double FIELD_IN = 141.5;
    private static final double RED_PARK_X = 0.16 + 3.5 + ROBOT_LENGTH_IN / 2;

    private final double startX, startY, startHeadingDeg, parkX, parkY;

    private Follower follower;

    /** Red OpModes pass red coordinates; blue ones pass the 180-degree rotated equivalents. */
    protected BioBuzzBackupAuto(boolean blue, double startX, double startY, double startHeadingDeg,
                                double parkY) {
        this.startX = startX;
        this.startY = startY;
        this.startHeadingDeg = startHeadingDeg;
        this.parkX = blue ? FIELD_IN - RED_PARK_X : RED_PARK_X;
        this.parkY = parkY;
    }

    @Override
    public void init() {
        Scheduler.reset();
        PoseFactory p = PoseFactory.degrees();
        Pose startPose = p.of(startX, startY, startHeadingDeg);
        Pose parkPose = p.of(parkX, parkY, startHeadingDeg);

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        Path toPark = Paths.line(startPose, parkPose)
                .constant(Math.toRadians(startHeadingDeg));
        Scheduler.schedule(sequential(follow(follower, toPark)));
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }
}
