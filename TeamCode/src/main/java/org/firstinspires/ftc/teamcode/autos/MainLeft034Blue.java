package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MAIN left-034 (blue)", group = "BioBuzz Main")
public class MainLeft034Blue extends BioBuzzMainAuto {
    @Override protected Routine routine() { return Routines.LEFT_034(); }
    @Override protected boolean isBlue() { return true; }
}
