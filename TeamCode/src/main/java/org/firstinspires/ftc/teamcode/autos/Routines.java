package org.firstinspires.ftc.teamcode.autos;

import java.util.ArrayList;
import java.util.List;

/**
 * GENERATED from the simulator (caryden/ftc-biobuzz-sim) with a FIFO shooter: right-004, right-022, left-034, left-020.
 * Poses are RED alliance, Pedro inches and degrees; shooter faces the rear, intake on the front.

 * Step order, shot counts and time limits are the simulator's. Hopper-full and camera waits become your Mechanisms hooks.
 */
public final class Routines {
    private Routines() {}
    private static final double[][] NO_VIA = new double[0][];

    // RIGHT_004: start (7.8, 66.8) heading 0.0, 23 steps, PARK forced with 4.0 s left
    public static Routine RIGHT_004() {
        List<AutoStep> s = new ArrayList<>();
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s1 driveTo", 60.6, 17.6, -90.0, NO_VIA, 0, 4.0, false));
        s.add(new AutoStep(AutoStep.Type.SHOOT, "s2 shoot", 60.6, 17.6, -90.0, NO_VIA, 4, 2.5, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s3 driveTo", 21.8, 47.9, 180.0, NO_VIA, 0, 3.5, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s4 driveTo", 14.1, 47.4, 180.0, NO_VIA, 0, 1.5, true));
        s.add(new AutoStep(AutoStep.Type.PUSH, "s5 push", 13.5, 47.3, 180.0, NO_VIA, 0, 2.2, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s6 driveTo", 61.1, 17.4, -90.0, NO_VIA, 0, 4.0, false));
        s.add(new AutoStep(AutoStep.Type.WAIT_CELL_AUDIENCE, "s7 waitUntil", 61.1, 17.4, -90.0, NO_VIA, 0, 14.0, false));
        s.add(new AutoStep(AutoStep.Type.SHOOT, "s8 shoot", 61.1, 17.4, -90.0, NO_VIA, 4, 2.5, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s9 driveTo", 60.3, 10.7, -147.0, NO_VIA, 0, 2.5, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s10 driveTo", 18.2, 10.4, -150.0, NO_VIA, 0, 3.0, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s11 driveTo", 60.7, 18.1, -90.0, NO_VIA, 0, 3.5, false));
        s.add(new AutoStep(AutoStep.Type.SHOOT, "s12 shoot", 60.7, 18.1, -90.0, NO_VIA, 4, 2.5, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s13 driveTo", 57.9, 46.7, -90.0, NO_VIA, 0, 2.0, false));
        s.add(new AutoStep(AutoStep.Type.WAIT, "s14 wait", 57.9, 46.7, -90.0, NO_VIA, 0, 2.0, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s15 driveTo", 60.2, 10.4, -150.0, NO_VIA, 0, 2.5, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s16 driveTo", 15.4, 10.5, -150.0, NO_VIA, 0, 3.0, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s17 driveTo", 58.6, 123.1, 90.0, new double[][]{{12.3, 15.8, 170.4}, {31.5, 88.5, 90.0}}, 0, 5.0, false));
        s.add(new AutoStep(AutoStep.Type.SHOOT, "s18 shoot", 58.6, 123.1, 90.0, NO_VIA, 4, 3.0, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s19 driveTo", 59.3, 131.3, 92.0, NO_VIA, 0, 1.5, true));
        s.add(new AutoStep(AutoStep.Type.PUSH, "s20 push", 59.6, 131.9, 93.0, NO_VIA, 0, 0.6, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s21 driveTo", 58.6, 123.1, 90.0, NO_VIA, 0, 2.0, false));
        s.add(new AutoStep(AutoStep.Type.SHOOT, "s22 shoot", 58.6, 123.1, 90.0, NO_VIA, 4, 1.5, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s23 driveTo", 10.8, 92.5, 180.0, new double[][]{{59.6, 131.9, 93.3}, {55.7, 123.6, 159.0}}, 0, 6.0, true));
        return new Routine("RIGHT_004", 7.8, 66.8, 0.0, 4.0, s);
    }

    // LEFT_034: start (29.4, 133.7) heading 90.0, 25 steps, PARK forced with 3.0 s left
    public static Routine LEFT_034() {
        List<AutoStep> s = new ArrayList<>();
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s1 driveTo", 36.3, 120.5, 126.0, NO_VIA, 0, 3.0, false));
        s.add(new AutoStep(AutoStep.Type.WAIT_CELL_REAR, "s2 waitUntil", 36.3, 120.5, 126.0, NO_VIA, 0, 8.0, false));
        s.add(new AutoStep(AutoStep.Type.SHOOT, "s3 shoot", 36.3, 120.5, 126.0, NO_VIA, 4, 2.5, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s4 driveTo", 47.9, 119.3, 91.0, NO_VIA, 0, 3.0, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s5 driveTo", 47.4, 127.4, 90.0, NO_VIA, 0, 1.5, true));
        s.add(new AutoStep(AutoStep.Type.PUSH, "s6 push", 47.3, 128.0, 90.0, NO_VIA, 0, 2.2, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s7 driveTo", 35.4, 120.9, 126.0, NO_VIA, 0, 3.0, false));
        s.add(new AutoStep(AutoStep.Type.SHOOT, "s8 shoot", 35.4, 120.9, 126.0, NO_VIA, 4, 2.5, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s9 driveTo", 35.0, 120.7, 127.0, NO_VIA, 0, 2.5, false));
        s.add(new AutoStep(AutoStep.Type.WAIT_TIP, "s10 waitTip", 35.0, 120.7, 127.0, NO_VIA, 0, 4.0, false));
        s.add(new AutoStep(AutoStep.Type.WAIT, "s11 wait", 35.0, 120.7, 127.0, NO_VIA, 0, 2.8, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s12 driveTo", 59.9, 121.2, 90.0, NO_VIA, 0, 2.5, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s13 driveTo", 59.2, 132.1, 88.0, NO_VIA, 0, 2.0, true));
        s.add(new AutoStep(AutoStep.Type.PUSH, "s14 push", 59.3, 133.3, 86.0, NO_VIA, 0, 0.8, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s15 driveTo", 35.2, 121.0, 127.0, new double[][]{{56.4, 124.6, 121.4}}, 0, 3.0, false));
        s.add(new AutoStep(AutoStep.Type.SHOOT, "s16 shoot", 35.2, 121.0, 127.0, NO_VIA, 4, 12.0, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s17 driveTo", 33.0, 131.0, 149.0, NO_VIA, 0, 2.5, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s18 driveTo", 11.9, 130.8, 150.0, NO_VIA, 0, 3.0, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s19 driveTo", 10.9, 125.9, -123.0, NO_VIA, 0, 2.0, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s20 driveTo", 10.8, 87.9, -120.0, NO_VIA, 0, 3.0, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s21 driveTo", 36.4, 121.7, 127.0, NO_VIA, 0, 3.0, false));
        s.add(new AutoStep(AutoStep.Type.SHOOT, "s22 shoot", 36.4, 121.7, 127.0, NO_VIA, 4, 8.0, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s23 driveTo", 10.5, 126.1, -121.0, NO_VIA, 0, 2.0, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s24 driveTo", 10.8, 87.9, -120.0, NO_VIA, 0, 3.0, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s25 driveTo", 11.2, 119.7, 180.0, NO_VIA, 0, 6.0, true));
        return new Routine("LEFT_034", 29.4, 133.7, 90.0, 3.0, s);
    }

    // RIGHT_022: start (7.8, 66.8) heading 0.0, 23 steps, PARK forced with 2.0 s left
    public static Routine RIGHT_022() {
        List<AutoStep> s = new ArrayList<>();
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s1 driveTo", 55.0, 12.1, -90.0, NO_VIA, 0, 4.0, false));
        s.add(new AutoStep(AutoStep.Type.SHOOT, "s2 shoot", 55.0, 12.1, -90.0, NO_VIA, 4, 2.5, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s3 driveTo", 21.8, 48.0, 180.0, NO_VIA, 0, 3.5, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s4 driveTo", 14.1, 47.4, 180.0, NO_VIA, 0, 1.5, true));
        s.add(new AutoStep(AutoStep.Type.PUSH, "s5 push", 13.5, 47.3, 180.0, NO_VIA, 0, 2.2, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s6 driveTo", 55.5, 11.9, -90.0, NO_VIA, 0, 4.0, false));
        s.add(new AutoStep(AutoStep.Type.WAIT_CELL_AUDIENCE, "s7 waitUntil", 55.5, 11.9, -90.0, NO_VIA, 0, 14.0, false));
        s.add(new AutoStep(AutoStep.Type.SHOOT, "s8 shoot", 55.5, 11.9, -90.0, NO_VIA, 4, 2.5, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s9 driveTo", 60.1, 10.7, -147.0, NO_VIA, 0, 2.5, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s10 driveTo", 15.4, 10.6, -150.0, NO_VIA, 0, 3.0, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s11 driveTo", 55.2, 12.5, -90.0, NO_VIA, 0, 3.5, false));
        s.add(new AutoStep(AutoStep.Type.SHOOT, "s12 shoot", 55.2, 12.5, -90.0, NO_VIA, 4, 2.5, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s13 driveTo", 58.1, 46.7, -90.0, NO_VIA, 0, 2.0, false));
        s.add(new AutoStep(AutoStep.Type.WAIT, "s14 wait", 58.1, 46.7, -90.0, NO_VIA, 0, 2.9, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s15 driveTo", 60.0, 11.6, -151.0, NO_VIA, 0, 2.5, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s16 driveTo", 43.9, 10.7, -150.0, NO_VIA, 0, 3.0, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s17 driveTo", 58.6, 123.1, 90.0, new double[][]{{37.9, 12.8, -168.4}, {34.4, 25.8, 115.4}, {31.3, 79.0, 90.0}, {38.4, 100.3, 90.0}}, 0, 5.0, false));
        s.add(new AutoStep(AutoStep.Type.SHOOT, "s18 shoot", 58.6, 123.1, 90.0, NO_VIA, 4, 3.0, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s19 driveTo", 59.2, 132.1, 92.0, NO_VIA, 0, 1.5, true));
        s.add(new AutoStep(AutoStep.Type.PUSH, "s20 push", 59.0, 133.3, 95.0, NO_VIA, 0, 0.6, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s21 driveTo", 58.0, 122.5, 90.0, NO_VIA, 0, 2.0, false));
        s.add(new AutoStep(AutoStep.Type.SHOOT, "s22 shoot", 58.0, 122.5, 90.0, NO_VIA, 4, 1.5, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s23 driveTo", 10.8, 92.5, 180.0, NO_VIA, 0, 6.0, true));
        return new Routine("RIGHT_022", 7.8, 66.8, 0.0, 2.0, s);
    }

    // LEFT_020: start (29.4, 133.7) heading 90.0, 21 steps, PARK forced with 4.0 s left
    public static Routine LEFT_020() {
        List<AutoStep> s = new ArrayList<>();
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s1 driveTo", 34.9, 117.6, 127.0, NO_VIA, 0, 3.0, false));
        s.add(new AutoStep(AutoStep.Type.WAIT_CELL_REAR, "s2 waitUntil", 34.9, 117.6, 127.0, NO_VIA, 0, 8.0, false));
        s.add(new AutoStep(AutoStep.Type.SHOOT, "s3 shoot", 34.9, 117.6, 127.0, NO_VIA, 4, 2.5, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s4 driveTo", 48.0, 119.4, 91.0, NO_VIA, 0, 3.0, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s5 driveTo", 47.4, 127.4, 90.0, NO_VIA, 0, 1.5, true));
        s.add(new AutoStep(AutoStep.Type.PUSH, "s6 push", 47.3, 128.0, 90.0, NO_VIA, 0, 2.2, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s7 driveTo", 34.3, 118.1, 127.0, NO_VIA, 0, 3.0, false));
        s.add(new AutoStep(AutoStep.Type.SHOOT, "s8 shoot", 34.3, 118.1, 127.0, NO_VIA, 4, 2.5, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s9 driveTo", 33.8, 117.8, 127.0, NO_VIA, 0, 2.5, false));
        s.add(new AutoStep(AutoStep.Type.WAIT_TIP, "s10 waitTip", 33.8, 117.8, 127.0, NO_VIA, 0, 4.0, false));
        s.add(new AutoStep(AutoStep.Type.WAIT, "s11 wait", 33.8, 117.8, 127.0, NO_VIA, 0, 2.3, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s12 driveTo", 59.9, 121.2, 90.0, NO_VIA, 0, 2.5, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s13 driveTo", 60.1, 128.5, 90.0, NO_VIA, 0, 2.0, true));
        s.add(new AutoStep(AutoStep.Type.PUSH, "s14 push", 60.1, 128.5, 90.0, NO_VIA, 0, 0.8, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s15 driveTo", 34.0, 118.2, 127.0, NO_VIA, 0, 3.0, false));
        s.add(new AutoStep(AutoStep.Type.SHOOT, "s16 shoot", 34.0, 118.2, 127.0, NO_VIA, 4, 12.0, false));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s17 driveTo", 33.0, 131.1, 150.0, NO_VIA, 0, 2.5, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s18 driveTo", 11.9, 130.7, 150.0, NO_VIA, 0, 3.0, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s19 driveTo", 10.9, 125.9, -123.0, NO_VIA, 0, 2.0, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s20 driveTo", 10.8, 87.9, -120.0, NO_VIA, 0, 3.0, true));
        s.add(new AutoStep(AutoStep.Type.DRIVE, "s25 driveTo", 11.2, 119.7, 180.0, NO_VIA, 0, 6.0, true));
        return new Routine("LEFT_020", 29.4, 133.7, 90.0, 4.0, s);
    }

}
