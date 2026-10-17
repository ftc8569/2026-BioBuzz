package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.function.LongSupplier;

/** Same shooting sequence as BasicRed, using the exact pedro paths/basicblue.pp route. */
@Autonomous(name = "BasicBlue Auto")
public class BasicBlueAuto extends BioBuzzAuto {
    public BasicBlueAuto() {
        super();
    }

    BasicBlueAuto(LongSupplier nanoTime) {
        super(nanoTime);
    }

    @Override
    protected Pose startingPose() {
        return PoseFactory.degrees().of(83.07841726618705, 132.19424460431654, 90);
    }

    @Override
    protected Path buildPath(Pose start) {
        Pose end = PoseFactory.degrees().of(132.9597122302158, 41.497122302158274, 90);
        return Paths.line(start, end).constant(Math.toRadians(90));
    }
}
