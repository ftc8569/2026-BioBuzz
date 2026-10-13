package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MAIN right-022 (red)", group = "BioBuzz Main")
public class MainRight022Red extends BioBuzzMainAuto {
    @Override protected Routine routine() { return Routines.RIGHT_022(); }
    @Override protected boolean isBlue() { return false; }
}
