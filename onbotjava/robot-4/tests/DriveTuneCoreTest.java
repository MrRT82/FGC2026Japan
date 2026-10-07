package org.firstinspires.ftc.teamcode.tuning;
import java.io.*;
import java.nio.file.*;

public final class DriveTuneCoreTest {
    private static int checks;
    private static final long MS = 1_000_000L;
    private static void check(boolean value, String message) {
        checks++; if (!value) throw new AssertionError(message);
    }
    private static boolean near(double a, double b) { return Math.abs(a - b) < 1e-9; }
    private static DriveTuneConfig config() {
        DriveTuneConfig c = new DriveTuneConfig();
        c.motionEnabled = true; c.gearReduction = 15.15; c.wheelDiameterMm = 90;
        c.maxPower = .5; c.maxTravelM = 1; c.maxTurnDeg = 90; c.runMs = 1000;
        return c; // Synthetic test settings only, not robot limits.
    }
    public static void main(String[] args) throws Exception {
        long t = 1_000 * MS;
        DriveTuneCore c = new DriveTuneCore(new DriveTuneConfig());
        c.update(t, true, true, true, 0, 0, 0, 0, 0, .5);
        check(c.state.equals("MONITOR") && c.left == 0 && c.right == 0, "default must never move");

        DriveTuneConfig cfg = config();
        double ticksPerMetre = 1.01 / cfg.metresPerTick();
        check(near(cfg.metresPerTick(), Math.PI * .09 / (28 * 15.15)), "metres per motor tick");
        c = new DriveTuneCore(cfg);
        c.update(t, false, true, true, 0, 0, 0, 0, 0, 0);
        check(c.state.equals("IDLE"), "start needs deadman");
        c.update(t += 20 * MS, true, true, true, 0, 0, 0, 0, 0, 0);
        check(c.running() && c.trialId == 1 && c.left == 0, "start loop outputs zero");
        c.update(t += 20 * MS, true, false, true, 0, 0, 500, 500, 0, 0);
        check(c.phase.equals("CRUISE") && c.left == .3 && c.right == .3, "step acceleration as match code");
        c.select(1, 1, true, true);
        check(c.trial == 0 && c.level == 0 && c.direction == 1 && c.brake == 0, "selection frozen while running");
        c.update(t += 20 * MS, true, false, true, ticksPerMetre, ticksPerMetre, 500, 500, 0, 0);
        check(c.phase.equals("BRAKE") && c.event.equals("BRAKE_TRAVEL") && near(c.left, -.015), "match brake pulse");
        c.update(t += 20 * MS, true, false, true, ticksPerMetre, ticksPerMetre, 200, 200, 0, 0);
        check(c.command == 0 && c.left == 0, "match brake then zero");
        c.update(t += 20 * MS, true, false, true, ticksPerMetre, ticksPerMetre, 0, 0, 0, 0);
        c.update(t += 100 * MS, true, false, true, ticksPerMetre, ticksPerMetre, 0, 0, 0, 0);
        check(c.running(), "stop needs 200 ms still");
        c.update(t += 100 * MS, true, false, true, ticksPerMetre, ticksPerMetre, 0, 0, 0, 0);
        check(c.state.equals("IDLE") && c.stopReason.equals("STOPPED"), "stopped ends trial");
        check(near(DriveTuneCore.matchBrake(1), -.05) && DriveTuneCore.matchBrake(.02) == 0, "DriveController neutral law");

        cfg = config(); cfg.accelPerSec = 1; cfg.brakePerSec = 2; cfg.powers = new double[]{.7};
        c = new DriveTuneCore(cfg);
        c.select(0, 0, true, true);
        check(c.power() == .5 && c.direction == -1 && c.brakeText().startsWith("RAMP"), "power capped, reverse, ramp");
        c.update(t, true, true, true, 0, 0, 0, 0, 0, 0);
        c.update(t += 20 * MS, true, false, true, 0, 0, 0, 0, 0, 0);
        check(c.phase.equals("ACCEL") && near(c.left, -.02), "acceleration ramp");
        for (int n = 0; n < 30; n++) c.update(t += 20 * MS, true, false, true, 0, 0, -900, -900, 0, 0);
        check(c.phase.equals("CRUISE") && near(c.left, -.5), "reaches capped power");
        c.update(t += 20 * MS, true, false, true, -ticksPerMetre, -ticksPerMetre, -900, -900, 0, 0);
        check(c.event.equals("BRAKE_TRAVEL") && near(c.left, -.46), "reverse travel limit, ramped brake");
        c.update(t += 20 * MS, true, false, true, -ticksPerMetre, -ticksPerMetre, -900, -900, 0, 0);
        check(near(c.left, -.42), "brake ramp rate");
        c.update(t += 20 * MS, false, false, true, 0, 0, -900, -900, 0, 0);
        check(c.state.equals("IDLE") && c.stopReason.equals("DEADMAN_RELEASED") && c.left == 0, "deadman release");

        c = new DriveTuneCore(config());
        c.update(t, true, true, true, 0, 0, 0, 0, 0, 0);
        c.update(t += 101 * MS, true, false, true, 0, 0, 0, 0, 0, 0);
        check(c.state.equals("FAULT") && c.fault.equals("LOOP_DELAY") && c.left == 0, "delay latch");
        c.update(t += 20 * MS, true, true, true, 0, 0, 0, 0, 0, 0);
        check(!c.running(), "fault does not self-clear");
        c = new DriveTuneCore(config());
        c.update(t, true, true, false, 0, 0, 0, 0, 0, 0);
        check(c.fault.equals("LOGGER_FAILED") && !c.running(), "logger failure");
        c = new DriveTuneCore(config());
        c.trip("OPERATOR_ABORT");
        c.update(t, true, true, true, 0, 0, 0, 0, 0, 0);
        check(c.state.equals("FAULT") && c.left == 0, "operator abort latches");

        c = new DriveTuneCore(config());
        c.select(1, 0, false, false);
        c.update(t, true, true, true, 0, 0, 0, 0, Double.NaN, 0);
        check(!c.running() && c.event.equals("START_REJECTED_IMU"), "turn needs IMU");
        c.update(t += 20 * MS, true, true, true, 0, 0, 0, 0, 170, 0);
        c.update(t += 20 * MS, true, false, true, 0, 0, 0, 0, 170, 0);
        check(c.left == -.3 && c.right == .3, "positive turn raises right side");
        c.update(t += 20 * MS, true, false, true, 0, 0, 0, 0, -170, 0);
        check(near(c.turnedDeg, 20), "turned angle unwraps across 180");
        c.update(t += 20 * MS, true, false, true, 0, 0, 0, 0, -100, 0);
        check(c.phase.equals("BRAKE") && c.event.equals("BRAKE_ANGLE"), "turn angle limit");
        c.update(t += 20 * MS, true, false, true, 0, 0, 0, 0, Double.NaN, 0);
        check(c.fault.equals("IMU_INVALID") && c.left == 0, "IMU loss stops turn");

        c = new DriveTuneCore(config());
        c.select(2, 0, false, false);
        c.update(t, true, true, true, 0, 0, 0, 0, 10, .9);
        check(c.wantsImu() && near(c.headingTarget, 55) && c.left == 0, "heading target from current yaw");
        c.update(t += 20 * MS, true, false, true, 0, 0, 0, 0, 10, .4);
        check(near(c.left, -.4) && near(c.right, .4), "heading uses ImuController output");
        c.update(t += 20 * MS, true, false, true, 0, 0, 0, 0, 20, .9);
        check(near(c.right, .5), "heading output capped");
        c.update(t += 20 * MS, true, false, true, 0, 0, 0, 0, 120, .9);
        check(c.fault.equals("HEADING_RUNAWAY") && c.right == 0, "overshoot runaway");
        c = new DriveTuneCore(config());
        c.select(2, 0, false, false);
        c.update(t, true, true, true, 0, 0, 0, 0, 0, 0);
        c.update(t += 20 * MS, true, false, true, 0, 0, 0, 0, -31, 0);
        check(c.fault.equals("HEADING_RUNAWAY"), "wrong-way runaway");
        c = new DriveTuneCore(config());
        c.select(2, 0, false, false);
        c.update(t, true, true, true, 0, 0, 0, 0, 0, 0);
        for (int n = 0; n < 160; n++) c.update(t += 20 * MS, true, false, true, 0, 0, 0, 0, 45, 0);
        check(c.state.equals("IDLE") && c.stopReason.equals("TIME"), "heading trial time");

        Path temp = Files.createTempDirectory("r4-drive-test");
        check(!DriveTuneConfig.load(temp.resolve("missing.properties").toFile()).motionEnabled, "missing config");
        File open = temp.resolve("open.properties").toFile();
        Files.write(open.toPath(), "motion_enabled=true\n".getBytes("UTF-8"));
        boolean rejected = false;
        try { DriveTuneConfig.load(open); } catch (IOException expected) { rejected = true; }
        check(rejected, "motion needs wheel, gear and limits");
        File good = temp.resolve("good.properties").toFile();
        Files.write(good.toPath(), ("motion_enabled=true\ngear_reduction=15.15\nwheel_diameter_mm=90\nmax_power=0.6\n"
            + "max_travel_m=1.5\nmax_turn_deg=180\npowers=0.2, 0.4\nimu_kp=0.03\nbrake_per_s=3\n").getBytes("UTF-8"));
        DriveTuneConfig loaded = DriveTuneConfig.load(good);
        check(loaded.motionEnabled && loaded.powers.length == 2 && loaded.powers[1] == .4, "powers list");
        check(loaded.imuKp == .03 && Double.isNaN(loaded.imuKd) && loaded.brakePerSec == 3, "optional overrides");
        File bad = temp.resolve("bad.properties").toFile();
        Files.write(bad.toPath(), ("motion_enabled=true\ngear_reduction=15.15\nwheel_diameter_mm=90\nmax_power=1.5\n"
            + "max_travel_m=1.5\nmax_turn_deg=180\n").getBytes("UTF-8"));
        rejected = false;
        try { DriveTuneConfig.load(bad); } catch (IOException expected) { rejected = true; }
        check(rejected, "max_power above 1 rejected");
        System.out.println("DriveTuneCoreTest passed: " + checks);
    }
}
