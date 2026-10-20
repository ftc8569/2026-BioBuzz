package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Backup auto: one straight line from the start pose to the loading zone, ending facing the
 * alliance wall (LEAVE + PARK). No obstacle avoidance.
 * Pedro inches, field 141.5 x 141.5, heading in degrees CCW from +x.
 */
public abstract class BioBuzzBackupAuto extends AutoOpMode {
    // TODO: set to the real robot length in inches.
    protected static final double ROBOT_LENGTH_IN = 18;
    private static final double FIELD_IN = 141.5;
    private static final double RED_PARK_X = 0.16 + 3.5 + ROBOT_LENGTH_IN / 2;

    private final double startX, startY, startHeadingDeg, endHeadingDeg, parkX, parkY;

    private Path toPark;

    /**
     * Red OpModes pass red coordinates; blue ones pass the 180-degree rotated equivalents.
     * endHeadingDeg is unwrapped (e.g. 180 -> 360) so the turn direction is explicit.
     */
    protected BioBuzzBackupAuto(boolean blue, double startX, double startY, double startHeadingDeg,
                                double endHeadingDeg, double parkY) {
        this.startX = startX;
        this.startY = startY;
        this.startHeadingDeg = startHeadingDeg;
        this.endHeadingDeg = endHeadingDeg;
        this.parkX = blue ? FIELD_IN - RED_PARK_X : RED_PARK_X;
        this.parkY = parkY;
    }

    @Override protected boolean usesShooter() { return false; }

    @Override protected Pose startingPose() {
        return PoseFactory.degrees().of(startX, startY, startHeadingDeg);
    }

    @Override protected void buildRoutine(Pose startPose) {
        Pose parkPose = PoseFactory.degrees().of(parkX, parkY, endHeadingDeg);
        toPark = Paths.line(startPose, parkPose)
                .linear(Math.toRadians(startHeadingDeg), Math.toRadians(endHeadingDeg));
    }

    @Override protected void startRoutine() { follower.follow(toPark); }

    @Override protected void updateRoutine() {
        operation = "LOOP Pedro / Pinpoint";
        follower.update();
    }
}
