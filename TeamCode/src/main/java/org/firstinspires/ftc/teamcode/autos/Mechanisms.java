package org.firstinspires.ftc.teamcode.autos;

/**
 * Everything the autonomous needs from the robot besides driving. RobotMechanisms implements it with IntakeSubsystem and ShooterSubsystem; NoMechanisms does nothing.
 * Only intake / spinUpShooter / startShooting / stopShooting / stopAll are required. The sensor methods have
 * defaults that make the routine fall back to the simulator's timers.
 *
 * Simulator robot: the SHOOTER faces the REAR (it launches out the back) and the INTAKE is on the FRONT.
 * It is a FIFO shooter: one ball out per shot, in the order they were loaded (about 0.3 s per ball in the sim).
 */
public interface Mechanisms {
    default void update() {}              // called every loop; run shooter / intake state machines here
    void intake(boolean on);              // front intake on or off
    void spinUpShooter();                 // get the flywheel up to speed (called once at start)
    void startShooting(int balls);        // fire this many balls, one at a time
    void stopShooting();
    void stopAll();                       // end of AUTO

    default boolean shootingDone() { return false; }          // true when startShooting's balls are all out
    default boolean hopperFull() { return false; }            // a full hopper ends an intake drive early
    default boolean hasVision() { return false; }             // true if you can see the raised CELL's tags
    default boolean seesRaisedCell(boolean rear) { return false; }  // rear = rear CELL, else audience CELL
    default boolean tipLanded() { return false; }             // a TIP just scored (to end the TIP wait early)
    default String status() { return ""; }                    // shown on telemetry
}
