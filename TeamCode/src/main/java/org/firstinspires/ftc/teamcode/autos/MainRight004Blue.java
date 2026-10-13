package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MAIN right-004 (blue)", group = "BioBuzz Main")
public class MainRight004Blue extends BioBuzzMainAuto {
    @Override protected Routine routine() { return Routines.RIGHT_004(); }
    @Override protected boolean isBlue() { return true; }
}
