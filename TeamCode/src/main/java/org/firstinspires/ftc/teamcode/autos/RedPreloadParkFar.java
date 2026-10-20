package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.function.LongSupplier;

/** Shoot the preload using the shared procedure, then follow the far-park curve. */
@Autonomous(name = "RedPreloadParkFar")
public class RedPreloadParkFar extends BioBuzzAuto {
    public RedPreloadParkFar() {
        super();
    }

    RedPreloadParkFar(LongSupplier nanoTime) {
        super(nanoTime);
    }

    @Override
    protected Pose startingPose() {
        return PoseFactory.degrees().of(58.4934, 9.0389, -90.0);
    }

    @Override
    protected Path buildPath(Pose start) {
        PoseFactory poses = PoseFactory.degrees();
        Pose control1 = poses.of(10.3025, 9.6131, 0.0);
        Pose control2 = poses.of(57.8311, 121.1131, 0.0);
        Pose end = poses.of(12.3656, 117.243, -90.0);
        return Paths.curve(start, control1, control2, end).linear(start, end);
    }
}
