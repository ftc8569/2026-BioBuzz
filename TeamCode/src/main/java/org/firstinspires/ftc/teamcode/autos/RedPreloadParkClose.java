package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.function.LongSupplier;

/** Shoot the preload using the shared procedure, then follow the close-park curve. */
@Autonomous(name = "RedPreloadParkClose")
public class RedPreloadParkClose extends BioBuzzAuto {
    public RedPreloadParkClose() {
        super();
    }

    RedPreloadParkClose(LongSupplier nanoTime) {
        super(nanoTime);
    }

    @Override
    protected Pose startingPose() {
        return PoseFactory.degrees().of(58.4934, 6.6, -90.0);
    }

    @Override
    protected Path buildPath(Pose start) {
        PoseFactory poses = PoseFactory.degrees();
        Pose control1 = poses.of(10.3025, 9.6131, 0.0);
        Pose control2 = poses.of(58.2467, 97.8414, 0.0);
        Pose end = poses.of(11.3267, 94.1791, 90.0);
        return Paths.curve(start, control1, control2, end).linear(start, end);
    }
}
