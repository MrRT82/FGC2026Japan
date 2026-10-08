package org.firstinspires.ftc.teamcode;

public class SimpleDriveClimbTest {
    static int count;
    static void check(boolean condition, String name) {
        count++;
        if (!condition) throw new AssertionError(name);
    }
    static void near(double actual, double expected, String name) {
        check(Math.abs(actual - expected) < 1e-9, name);
    }
    public static void main(String[] args) {
        double[] v = SimpleDriveClimb.driveMix(1, 0);
        near(v[0], 1, "forward left"); near(v[1], 1, "forward right");
        v = SimpleDriveClimb.driveMix(-1, 0);
        near(v[0], -1, "reverse left"); near(v[1], -1, "reverse right");
        v = SimpleDriveClimb.driveMix(0, 1);
        near(v[0], 1, "right turn left"); near(v[1], -1, "right turn right");
        v = SimpleDriveClimb.driveMix(1, 1);
        near(v[0], 1, "combined normalized left"); near(v[1], 0, "combined right");
        v = SimpleDriveClimb.driveMix(.5, -.25);
        near(v[0], .25, "mixed left"); near(v[1], .75, "mixed right");
        SimpleDriveClimb.ClimbControl c = new SimpleDriveClimb.ClimbControl(100);
        near(c.power(100), 0, "initial hold zero");
        near(c.power(90), .05, "P restores lower position");
        near(c.power(110), -.05, "P restores higher position");
        c.update(true, false, 100);
        near(c.power(100), .3, "RT starts up");
        c.update(true, false, 120);
        check(c.mode == SimpleDriveClimb.Mode.UP, "holding RT never double presses");
        c.update(false, false, 150);
        check(c.mode == SimpleDriveClimb.Mode.UP, "release retains motion");
        c.update(true, false, 160);
        check(c.mode == SimpleDriveClimb.Mode.HOLD && c.holdPosition == 160, "second RT captures stop position");
        c.update(true, false, 165);
        check(c.holdPosition == 160, "hold target does not drift");
        near(c.power(165), -.025, "P at stopped position");
        c.update(false, false, 165); c.update(false, true, 165);
        near(c.power(165), -.3, "LT starts down");
        c.update(false, false, 120); c.update(false, true, 110);
        check(c.mode == SimpleDriveClimb.Mode.HOLD && c.holdPosition == 110, "second LT holds");
        c.update(false, false, 110); c.update(true, false, 110);
        c.update(false, true, 120);
        check(c.mode == SimpleDriveClimb.Mode.DOWN, "opposite trigger changes direction");
        c.update(true, true, 115);
        check(c.mode == SimpleDriveClimb.Mode.HOLD && c.holdPosition == 115, "simultaneous stops");
        c.update(true, false, 110);
        check(c.mode == SimpleDriveClimb.Mode.HOLD && c.holdPosition == 115, "one held trigger cannot restart");
        c.update(false, false, 110); c.update(true, false, 110);
        check(c.mode == SimpleDriveClimb.Mode.UP, "release both and repress restarts");
        c.update(true, true, 123); c.update(true, true, 130);
        check(c.holdPosition == 123, "held simultaneous buttons do not recapture target");
        near(c.power(Integer.MIN_VALUE), 1, "large positive error clips without integer overflow");
        near(c.power(Integer.MAX_VALUE), -1, "large negative error clips");
        System.out.println("PASS: " + count + " drive, trigger and P-control checks; no hardware motion.");
    }
}
