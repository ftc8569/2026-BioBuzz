package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MAIN left-020 (red)", group = "BioBuzz Main")
public class MainLeft020Red extends BioBuzzMainAuto {
    @Override protected Routine routine() { return Routines.LEFT_020(); }
    @Override protected boolean isBlue() { return false; }
}
