package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MAIN left-034 (red)", group = "BioBuzz Main")
public class MainLeft034Red extends BioBuzzMainAuto {
    @Override protected Routine routine() { return Routines.LEFT_034(); }
    @Override protected boolean isBlue() { return false; }
}
