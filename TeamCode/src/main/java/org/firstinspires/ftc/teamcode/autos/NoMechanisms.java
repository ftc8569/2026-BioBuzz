package org.firstinspires.ftc.teamcode.autos;

/** Does nothing, so the paths can be driven with no shooter or intake wired in. Used when USE_MECHANISMS is false in BioBuzzMainAuto. */
public class NoMechanisms implements Mechanisms {
    private String last = "none";
    @Override public void intake(boolean on) { last = "intake " + (on ? "ON" : "off"); }
    @Override public void spinUpShooter() { last = "spin up shooter"; }
    @Override public void startShooting(int balls) { last = "SHOOT " + balls; }
    @Override public void stopShooting() { last = "stop shooting"; }
    @Override public void stopAll() { last = "all stopped"; }
    @Override public String status() { return last; }
}
