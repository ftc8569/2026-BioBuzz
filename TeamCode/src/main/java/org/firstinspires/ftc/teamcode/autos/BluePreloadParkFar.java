package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.function.LongSupplier;

/** Shoot the preload using the shared procedure, then follow the 180-degree-rotated far-park curve. */
@Autonomous(name = "BluePreloadParkFar")
public class BluePreloadParkFar extends BioBuzzAuto {
    public BluePreloadParkFar() {
        super();
    }

    BluePreloadParkFar(LongSupplier nanoTime) {
        super(nanoTime);
    }

    @Override
    protected Pose startingPose() {
        return bluePose(58.4934, 9.0389, -90.0);
    }

    @Override
    protected Path buildPath(Pose start) {
        Pose control1 = bluePose(10.3025, 9.6131, 0.0);
        Pose control2 = bluePose(57.8311, 121.1131, 0.0);
        Pose end = bluePose(12.3656, 117.243, -90.0);
        return Paths.curve(start, control1, control2, end).linear(start, end);
    }

    /** Rotate about the center of the same 141.5-inch field used by the other blue autos. */
    private Pose bluePose(double x, double y, double headingDegrees) {
        return PoseFactory.degrees().of(141.5 - x, 141.5 - y, headingDegrees + 180.0);
    }
}
