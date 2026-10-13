package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MAIN left-020 (blue)", group = "BioBuzz Main")
public class MainLeft020Blue extends BioBuzzMainAuto {
    @Override protected Routine routine() { return Routines.LEFT_020(); }
    @Override protected boolean isBlue() { return true; }
}
