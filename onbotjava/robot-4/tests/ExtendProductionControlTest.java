package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;

/** Desktop SDK fakes: exercises the production controller; never connects to a Hub. */
public final class ExtendProductionControlTest {
    private static int checks;
    private static void check(boolean ok, String message) {
        checks++;
        if (!ok) throw new AssertionError(message);
    }
    private static void near(double actual, double expected, String message) {
        check(Math.abs(actual - expected) < 1e-9, message + ": " + actual + " != " + expected);
    }
    private static void set(Object object, String name, Object value) throws Exception {
        Field f = object.getClass().getDeclaredField(name);
        f.setAccessible(true);
        f.set(object, value);
    }
    private static Object get(Object object, String name) throws Exception {
        Field f = object.getClass().getDeclaredField(name);
        f.setAccessible(true);
        return f.get(object);
    }
    private static double callDouble(ExtendController c, String name, double value) throws Exception {
        Method m = ExtendController.class.getDeclaredMethod(name, double.class);
        m.setAccessible(true);
        return (Double) m.invoke(c, value);
    }
    private static final class Clock extends ElapsedTime {
        double dt = .025;
        @Override public double seconds() { return dt; }
        @Override public void reset() { }
    }
    private static final class Rig {
        double angle = 40, supply = 3.3, sent;
        int writes;
        boolean failSensor;
        final Clock clock = new Clock();
        final ExtendController c;
        final CachedPower cache;
        Rig() throws Exception {
            DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class}, (p, m, a) -> {
                        if (m.getName().equals("setPower")) {
                            sent = (Double) a[0]; writes++;
                            check(Double.isFinite(sent) && Math.abs(sent) <= 1, "finite bounded motor output");
                        }
                        if (m.getName().equals("getPower")) return sent;
                        if (m.getName().equals("getCurrent")) return 0.0;
                        return null;
                    });
            RobotHardware hw = new RobotHardware();
            hw.extendMotor = motor;
            // OnBot's compile JAR has stubbed concrete SDK methods; override both reads explicitly.
            hw.potentiometer = new AnalogInput(null, 0) {
                @Override public double getVoltage() {
                    if (failSensor) throw new IllegalStateException("synthetic ADC failure");
                    return angle / 135 * 3.3;
                }
                @Override public double getMaxVoltage() { return supply; }
            };
            c = new ExtendController(hw);
            set(c, "loopTimer", clock);
            cache = (CachedPower) get(c, "extendOut");
        }
        void tick(double measuredAngle) { angle = measuredAngle; c.extend_loop(); }
        void seedVelocity(double velocity, double nextAngle) throws Exception {
            c.poteVel = velocity;
            set(c, "velInitialized", true);
            set(c, "prevAngleForVel", nextAngle - velocity * clock.dt);
        }
    }

    private static void profileAndDirection() throws Exception {
        Rig r = new Rig();
        r.tick(40);
        near(r.c.targetAngle, 40.0625, "first target starts at measured angle");
        near(r.c.targetVelocity, 2.5, "100 deg/s2 ramp at 25 ms");
        near(r.sent, .0375, "25 ms drive slew");
        near(r.c.kd, .0015, "up gain");
        for (int n = 0; n < 220; n++) {
            double old = r.c.targetAngle;
            double oldVelocity = r.c.targetVelocity;
            r.c.extendUpNow(); // The main calls this on every firing loop.
            r.tick(old);
            check(r.c.targetAngle >= old && r.c.targetAngle <= 75, "up profile cannot overshoot");
            check(r.c.targetVelocity <= 25 + 1e-9, "up speed limit");
            check(Math.abs(r.c.targetVelocity - oldVelocity) <= 2.5 + 1e-8,
                    "up acceleration/deceleration includes final arrival and hold");
            check(r.c.pidDirection == 1, "up direction retained at hold");
        }
        near(r.c.targetAngle, 75, "up goal unchanged");
        r.c.extendMiddle();
        near(r.c.targetAngle, 75, "middle command does not jump target");
        r.tick(75);
        near(r.c.targetVelocity, -2.5, "down acceleration");
        near(r.c.kd, .0025, "down gain");
        for (int n = 0; n < 220; n++) {
            double old = r.c.targetAngle;
            double oldVelocity = r.c.targetVelocity;
            r.c.extendMiddle();
            r.tick(old);
            check(r.c.targetAngle <= old && r.c.targetAngle >= 38, "middle arrival cannot overshoot");
            check(r.c.targetVelocity >= -25 - 1e-9, "down speed limit");
            check(Math.abs(r.c.targetVelocity - oldVelocity) <= 2.5 + 1e-8,
                    "down acceleration/deceleration includes final arrival and hold");
        }
        near(r.c.targetAngle, 38, "middle goal unchanged");
        r.seedVelocity(0, 37);
        r.tick(37); // Positive hold error must not select the up PID.
        near(r.c.targetVelocity, 0, "hold target velocity");
        check(r.c.pidDirection == -1, "hold retains last profile direction despite positive error");
        near(r.c.kd, .0025, "hold retains down D");
        r.c.extendDown();
        for (int n = 0; n < 240; n++) r.tick(r.c.targetAngle);
        near(r.c.targetAngle, 20, "down goal unchanged");

        r = new Rig();
        r.tick(40);
        for (int n = 0; n < 15; n++) r.tick(r.c.targetAngle);
        // Explicitly build a moving up profile, then reverse its destination.
        r.c.setTarget(75);
        set(r.c, "targetAngle", 45.0);
        set(r.c, "profileVelocity", 25.0);
        r.c.pidDirection = 1;
        r.c.extendDown();
        r.tick(45);
        check(r.c.targetVelocity > 0 && r.c.pidDirection == 1,
                "PID follows decelerating up profile, not new down goal");
        for (int n = 0; n < 12; n++) r.tick(r.c.targetAngle);
        check(r.c.targetVelocity < 0 && r.c.pidDirection == -1, "gain switches after profile reverses");
    }

    private static void pidAndFeedforward() throws Exception {
        Rig r = new Rig();
        r.tick(40);
        r.tick(41);
        near(r.c.poteVel, 14, "EMA35 velocity from 1 deg per 25 ms");
        double expectedPid = .07 * r.c.error + .005 * r.c.integral
                + .0015 * (r.c.targetVelocity - 14);
        near(r.c.pidOut, expectedPid, "D uses target minus filtered velocity");
        double expectedFf = 1.9969002748 / 12.6 * Math.cos(Math.toRadians(41 - 20))
                + .0404679237 / 12.6 * r.c.targetVelocity + .9482379897 / 12.6;
        near(r.c.ffOut, expectedFf, "up dynamic G+V+FC at reference voltage");
        near(r.c.requestedOutput, expectedFf + expectedPid, "unclipped requested output");
        near(callDouble(r.c, "gravityFF", 0), 1.9969002748 / 12.6, "no old gravity constant");
        for (double velocity : new double[]{-25, -.5, 0, .49, .5, 25}) {
            r.c.targetVelocity = velocity;
            r.c.poteVel = -100; // Measured direction must not choose friction direction.
            for (double error : new double[]{-3, -1, 0, 1, 3}) {
                boolean moving = Math.abs(velocity) >= .5;
                double sign = Math.signum(moving ? velocity : error);
                double mag = sign >= 0 ? (moving ? .9482379897 : .9624524741)
                        : (moving ? 1.9725096967 : 2.5390997778);
                double expected = sign * mag / 12.6 * (moving ? 1 : Math.min(1, Math.abs(error) / 2));
                near(callDouble(r.c, "frictionFF", error), expected, "dynamic/static friction boundary");
            }
        }
        r.c.snapTarget(75);
        r.c.integral = .49;
        r.tick(40);
        near(r.c.integral, .5, "positive integral cap");
        r.c.snapTarget(20);
        r.c.integral = -.49;
        r.tick(40);
        near(r.c.integral, -.5, "negative integral cap");
        r.c.ffEnabled = false;
        r.tick(40);
        near(r.c.ffOut, 0, "existing FF disable supported");
    }

    private static void asymmetricSlew() throws Exception {
        Rig r = new Rig();
        for (double dt : new double[]{.01, .02, .025, .05}) {
            set(r.c, "dtActual", dt);
            for (double velocity : new double[]{-20, -2, -1.99, 0, 1.99, 2, 20}) {
                r.c.poteVel = velocity;
                for (double previous : new double[]{-.8, -.2, 0, .2, .8}) {
                    r.cache.setNow(previous);
                    for (double request : new double[]{-10, -.25, 0, .25, 10}) {
                        double desired = Math.max(-1, Math.min(1, request));
                        double rate = Math.abs(velocity) >= 2 && (desired - previous) * velocity < 0 ? 4 : 1.5;
                        double actual = callDouble(r.c, "slewOutput", request);
                        double wantedStep = Math.min(Math.abs(desired - previous), rate * dt);
                        near(actual, previous + Math.signum(desired - previous) * wantedStep,
                                "slew quadrant/threshold/dt/clipping");
                        near(r.c.slewLimit, rate * dt, "reported slew limit");
                    }
                }
            }
        }
        r.cache.setNow(.2);
        r.cache.set(.203); // Suppressed by the existing output cache.
        near(r.sent, .2, "cache really skipped small change");
        set(r.c, "dtActual", .025);
        r.c.poteVel = 0;
        near(callDouble(r.c, "slewOutput", 1), .2375, "slew starts from actually sent value");
    }

    private static void stopsFaultsAndRecovery() throws Exception {
        Rig r = new Rig();
        r.clock.dt = 30; // Operator spends 30 seconds between INIT and first movement.
        r.tick(40);
        near(r.c.targetAngle, 40.0625, "INIT wait cannot enlarge first profile step");
        near(r.sent, .0375, "INIT wait cannot enlarge first output step");
        r.clock.dt = .025;
        for (double power : new double[]{-.55, -.003, .003, .8}) {
            r.cache.setNow(power);
            int before = r.writes;
            r.c.stop();
            check(r.writes == before + 1 && r.sent == 0, "stop always writes immediate zero");
        }
        r.tick(40);
        r.c.extendUpNow();
        r.c.stallFault = true;
        r.c.extendUpNow();
        check(r.c.stallFault, "held up command cannot clear stall latch");
        r.tick(40);
        near(r.sent, 0, "latched stall bypasses slew");
        r.c.extendDown();
        check(!r.c.stallFault, "different destination permits retry");
        r.tick(40);
        r.c.yieldCurrent = true;
        r.tick(39);
        near(r.sent, 0, "power yield bypasses slew");
        r.c.yieldCurrent = false;
        r.seedVelocity(0, 35);
        r.tick(35);
        near(r.c.targetAngle, 34.9375, "yield recovery syncs profile to new actual angle");
        check(Math.abs(r.sent) <= .0375 + 1e-9, "yield recovery starts from zero output");

        for (double invalid : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -6, 111}) {
            r.cache.setNow(.5);
            r.tick(invalid);
            check(r.c.poteFault && r.sent == 0, "invalid angle bypasses slew");
            r.tick(40);
            check(!r.c.poteFault && Double.isFinite(r.sent) && Double.isFinite(r.c.poteVel),
                    "invalid angle cannot poison velocity filter after recovery");
        }
        r.tick(100);
        check(!r.c.poteFault && r.c.highAngleRecovery, "90 to 110 deg enters controlled recovery");
        near(r.sent, -.20, "high-angle recovery uses bounded downward output");
        r.tick(89);
        near(r.sent, -.20, "high-angle recovery hysteresis remains active above 88 deg");
        r.tick(88);
        check(!r.c.highAngleRecovery && Math.abs(r.sent) <= .0375 + 1e-9,
                "high-angle recovery exits through zero and restarts bounded slew");
        for (double supply : new double[]{0, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
            r.supply = supply;
            r.tick(40);
            check(r.c.poteFault && r.sent == 0, "invalid ADC reference stops");
        }
        r.supply = 3.3;
        r.failSensor = true;
        r.cache.setNow(.5);
        boolean threw = false;
        try { r.tick(40); } catch (IllegalStateException expected) { threw = true; }
        check(threw && r.sent == 0, "ADC exception sends zero before propagating");
        r.failSensor = false;
        r.tick(40);
        check(Double.isFinite(r.sent), "ADC exception recovery");

        boolean rejected = false;
        try { r.c.setTarget(Double.NaN); } catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "non-finite command rejected");
        r.c.setTarget(200);
        near((Double) get(r.c, "targetGoal"), 88, "upper command bound preserved");
        r.c.setTarget(-200);
        near((Double) get(r.c, "targetGoal"), 20, "lower command bound preserved");
    }

    private static void jamReversal() throws Exception {
        Rig r = new Rig();
        r.c.extendUpNow();
        r.tick(40);
        r.c.snapTarget(75);
        set(r.c, "prevStuckCheckMs", System.currentTimeMillis() - 200);
        set(r.c, "prevStuckAngle", 40.0);
        set(r.c, "poteStuckStart", System.currentTimeMillis() - 700);
        r.tick(40);
        check(r.c.isStuck && r.sent == 0, "existing jam detector first stops");
        r.c.yieldCurrent = true;
        r.tick(40);
        near(r.sent, -.55, "jam reversal bypasses PIDF/slew and has priority over yield");
        set(r.c, "stuckReverseStart", System.currentTimeMillis() - 800);
        r.tick(40);
        near(r.sent, -.55, "1000 ms reversal duration preserved");
        r.c.yieldCurrent = false;
        set(r.c, "stuckReverseStart", System.currentTimeMillis() - 1100);
        r.seedVelocity(-10, 35);
        r.c.integral = .5;
        r.tick(35);
        check(!r.c.isStuck && !r.c.reverse, "returns to PID after reverse timeout");
        near(r.c.targetAngle, 35.0625, "post-jam profile starts from current position");
        near(r.c.targetVelocity, 2.5, "post-jam profile restarts its velocity");
        check(r.sent >= 0 && r.sent <= .10 + 1e-9,
                "post-jam recovery clears reverse output before bounded PIDF restart");
        near(r.c.integral, .0625 * .025, "post-jam integral reset before new error");
        r.c.extendUpNow();
        r.tick(35);
        near(r.c.targetAngle, 35.1875, "repeated goal does not reset recovering profile");
        r.c.snapTarget(25);
        set(r.c, "prevStuckCheckMs", System.currentTimeMillis() - 200);
        set(r.c, "prevStuckAngle", 40.0);
        set(r.c, "poteStuckStart", System.currentTimeMillis() - 700);
        r.tick(40);
        check(!r.c.isStuck, "downward command cannot trigger upward jam reversal");
        r.c.isStuck = true;
        set(r.c, "stuckReverseStart", System.currentTimeMillis());
        r.tick(Double.NaN);
        near(r.sent, 0, "sensor fault wins even during jam reversal");
        r.c.stop();
        check(!r.c.isStuck && !r.c.reverse, "explicit stop cancels jam state");
    }

    private static void slowRaiseAndDrop() throws Exception {
        // 射撃・排出の上昇: 巡航は通常25deg/sの1/3で、目標を越えない。
        Rig r = new Rig();
        r.tick(40);
        r.c.setTargetSlow(75);
        double peak = 0;
        for (int n = 0; n < 800 && r.c.targetAngle < 75; n++) {
            r.tick(r.c.targetAngle);
            peak = Math.max(peak, r.c.targetVelocity);
            check(r.c.targetAngle <= 75, "slow raise cannot overshoot");
        }
        near(r.c.targetAngle, 75, "slow raise reaches goal");
        check(Math.abs(peak - 25.0 / 3) < 1e-6, "slow raise cruise is one third of normal: " + peak);

        // 詰まり判定: 低速指令では指令速度の半分(約4.17deg/s)、通常指令では従来の5deg/s。
        double[][] cases = {{1, 4.5, 0}, {1, 3.0, 1}, {0, 4.5, 1}};
        for (double[] k : cases) {
            r = new Rig();
            r.tick(40);
            if (k[0] == 1) r.c.setTargetSlow(75); else r.c.setTarget(75);
            set(r.c, "prevStuckCheckMs", System.currentTimeMillis() - 200);
            set(r.c, "prevStuckAngle", 40.0 - k[1] * .2);
            set(r.c, "poteStuckStart", System.currentTimeMillis() - 700);
            r.tick(40);
            check(r.c.isStuck == (k[2] == 1), "stuck threshold follows commanded speed: " + k[0] + "/" + k[1]);
        }

        // LBの下降: 目標+12degより上は全力。出力は1周期0.3125(25ms)ずつ素早く変える。
        r = new Rig();
        for (int n = 0; n < 5; n++) r.tick(75);
        r.c.dropTo(20);
        check(r.c.dropping, "drop starts above slowdown zone");
        double prev = r.sent;
        r.tick(70);
        near(r.sent, Math.max(-1, prev - .3125), "drop enters full power through fast slew");
        long start = (Long) get(r.c, "dropStartMs");
        for (int n = 0; n < 6; n++) { r.c.dropTo(20); r.tick(60); }
        near(r.sent, -1, "full power drop");
        check(r.c.dropping && (Long) get(r.c, "dropStartMs") == start, "held LB keeps drop state");
        near(r.c.targetAngle, 60, "target follows angle during full-power drop");
        // 減速区間へ入ると、今の下降速度から目標で止まる減速プロファイルへつなぐ。
        r.seedVelocity(-80, 31);
        r.tick(31);
        check(!r.c.dropping && (Boolean) get(r.c, "dropSettling"), "slowdown zone hands over to profile");
        check(r.c.targetVelocity < 0 && r.c.targetVelocity >= -80 - 1e-9,
                "handover keeps downward speed without speeding up");
        near(r.c.slewLimit, .3125, "fast output slew while settling");
        double last = r.c.targetAngle;
        for (int n = 0; n < 200 && (Boolean) get(r.c, "dropSettling"); n++) {
            r.tick(r.c.targetAngle);
            check(r.c.targetAngle <= last + 1e-9 && r.c.targetAngle >= 20, "settling moves down without overshoot");
            last = r.c.targetAngle;
        }
        near(r.c.targetAngle, 20, "drop ends at goal");
        check(!(Boolean) get(r.c, "dropSettling"), "settling ends at goal");

        // 全力区間の時間上限、下限付近からのLB、別指令による取消、電力譲渡。
        r = new Rig();
        for (int n = 0; n < 5; n++) r.tick(75);
        r.c.dropTo(20);
        set(r.c, "dropStartMs", System.currentTimeMillis() - 1600);
        r.tick(70);
        check(!r.c.dropping && (Boolean) get(r.c, "dropSettling"), "full-power drop has a time limit");
        r = new Rig();
        for (int n = 0; n < 5; n++) r.tick(28);
        r.c.dropTo(20);
        check(!r.c.dropping, "inside slowdown zone uses the fast profile only");
        near((Double) get(r.c, "slewRate"), 100, "LB uses fast profile");
        r = new Rig();
        for (int n = 0; n < 5; n++) r.tick(75);
        r.c.dropTo(20);
        r.c.setTarget(75);
        check(!r.c.dropping, "new command cancels drop");
        r.c.dropTo(20);
        r.c.yieldCurrent = true;
        r.tick(70);
        near(r.sent, 0, "power yield wins over drop");
        r.c.yieldCurrent = false;
        r.tick(70);
        check(r.c.dropping && r.sent < 0, "drop resumes after yield");
    }

    public static void main(String[] args) throws Exception {
        profileAndDirection();
        pidAndFeedforward();
        asymmetricSlew();
        stopsFaultsAndRecovery();
        jamReversal();
        slowRaiseAndDrop();
        System.out.println("PASS: " + checks + " production extension checks (fake SDK hardware; no physical test)");
    }
}
