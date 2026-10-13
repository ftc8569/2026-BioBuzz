package org.firstinspires.ftc.teamcode.autos;

import java.util.List;

/** A whole autonomous: where it starts (red alliance) and its steps in order. The last step is the PARK. */
public final class Routine {
    public final String name;
    public final double startX, startY, startHeadingDeg;
    /** When this many seconds of AUTO are left, skip to the last step (the PARK), like the simulator does. */
    public final double finishAtSec;
    public final List<AutoStep> steps;

    public Routine(String name, double startX, double startY, double startHeadingDeg, double finishAtSec, List<AutoStep> steps) {
        this.name = name; this.startX = startX; this.startY = startY; this.startHeadingDeg = startHeadingDeg;
        this.finishAtSec = finishAtSec; this.steps = steps;
    }
}
