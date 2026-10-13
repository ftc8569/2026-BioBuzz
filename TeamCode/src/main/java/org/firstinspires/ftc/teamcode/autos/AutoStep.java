package org.firstinspires.ftc.teamcode.autos;

/** One step of a routine. Poses are RED alliance, Pedro inches and degrees; the runner flips them for blue. */
public final class AutoStep {
    public enum Type { DRIVE, PUSH, SHOOT, WAIT, WAIT_TIP, WAIT_CELL_AUDIENCE, WAIT_CELL_REAR }

    public final Type type;
    public final String label;          // which step of the simulator routine this is, e.g. "s5 push"
    public final double x, y, headingDeg;   // where the robot should be at the end of the step
    public final double[][] via;        // optional {x, y, headingDeg} points to pass through on the way
    public final int count;             // balls to shoot (SHOOT only)
    public final double timeoutSec;     // the simulator's time limit for this step
    public final boolean intake;        // run the intake during this step

    public AutoStep(Type type, String label, double x, double y, double headingDeg, double[][] via,
                    int count, double timeoutSec, boolean intake) {
        this.type = type; this.label = label; this.x = x; this.y = y; this.headingDeg = headingDeg;
        this.via = via; this.count = count; this.timeoutSec = timeoutSec; this.intake = intake;
    }
}
