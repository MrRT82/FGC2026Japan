package org.firstinspires.ftc.teamcode.tuning;
import java.io.*;
import java.nio.file.*;

public final class ExtendTuneCoreTest {
    private static int checks;
    private static void check(boolean value, String message) {
        checks++; if (!value) throw new AssertionError(message);
    }
    private static ExtendIdentificationConfig config() {
        ExtendIdentificationConfig c = new ExtendIdentificationConfig();
        c.motionEnabled = true; c.minAngle = 10; c.maxAngle = 70;
        c.motorLimitA = 2; c.hubLimitA = 8; c.minVoltage = 11; c.maxPower = .2;
        return c; // Synthetic test settings only, not robot limits.
    }
    public static void main(String[] args) throws Exception {
        ExtendTuneCore c = new ExtendTuneCore(new ExtendIdentificationConfig());
        c.update(40, 1_000_000_000L, true, true, 1, "", true);
        check(c.sent == 0 && c.state.equals("MONITOR"), "default must never move");
        c = new ExtendTuneCore(config());
        long t = 1_000_000_000L;
        c.update(40, t, true, true, 0, "", true);
        check(c.state.equals("ACTIVE") && c.target == 40, "arm at measured angle");
        c.update(40, t += 20_000_000L, true, false, 1, "", true);
        check(Math.abs(c.target - 40.2) < 1e-8 && c.goal == 45, "target slew");
        check(Math.abs(c.sent) <= 1, "output bound is full duty only");
        c.update(40, t += 20_000_000L, false, false, 0, "", true);
        check(c.sent == 0, "deadman release");
        c.update(40, t += 20_000_000L, true, false, 0, "", true);
        check(c.sent == 0, "repress requires new A edge");
        c.update(40, t += 20_000_000L, true, true, 0, "", true);
        c.update(40, t += 20_000_000L, true, false, 0, "ANGLE_READ_DELAY", true);
        check(c.sent == 0 && c.fault.equals("ANGLE_READ_DELAY"), "reported fault latches");
        c.update(40, t += 20_000_000L, true, true, 0, "", true);
        check(c.sent == 0, "fault does not self-clear");
        c = new ExtendTuneCore(config());
        c.update(40, t, true, true, 0, "", true);
        c.update(40, t + 101_000_000L, true, false, 0, "", true);
        check(c.sent == 0 && c.fault.equals("LOOP_DELAY"), "delay latch");
        c = new ExtendTuneCore(config()); c.update(Double.NaN, t, true, true, 0, "", true);
        check(c.sent == 0 && c.fault.equals("ANGLE_INVALID"), "NaN angle");
        c = new ExtendTuneCore(config()); c.update(40, t, true, true, 0, "", false);
        check(c.sent == 0 && c.fault.equals("LOGGER_FAILED"), "logger failure");
        c = new ExtendTuneCore(config()); c.update(85, t, true, true, 0, "", true);
        check(c.state.equals("ACTIVE") && c.fault.isEmpty(), "no angle limit");
        c = new ExtendTuneCore(config()); c.update(40, t, true, true, 0, "", true);
        for (int n = 1; n <= 400; n++) c.update(40, t + n * 20_000_000L, true, false, 0, "", true);
        check(c.state.equals("ACTIVE") && c.fault.isEmpty(), "no trial time limit");
        c = new ExtendTuneCore(config()); c.update(40, t, true, true, 0, "", true);
        for (int n = 1; n < 100; n++) c.update(40, t + n * 20_000_000L, true, false, n == 1 ? 1 : 0, "", true);
        check(c.state.equals("ACTIVE") && c.fault.isEmpty() && c.sent > 0, "no stall limit; stall does not reverse");
        Path temp = Files.createTempDirectory("r4-tune-test");
        File log = temp.resolve("log.csv").toFile();
        TuneCsvWriter writer = new TuneCsvWriter(log, "a,b");
        check(writer.offer("1,2"), "queue row"); writer.close();
        check(new String(Files.readAllBytes(log.toPath()), "UTF-8").contains("1,2"), "close drains");
        check(!writer.offer("3,4"), "closed rejects");
        check(!ExtendIdentificationConfig.load(temp.resolve("missing.properties").toFile()).motionEnabled, "missing config");
        File bad = temp.resolve("bad.properties").toFile();
        File open = temp.resolve("open.properties").toFile();
        Files.write(open.toPath(), "motion_enabled=true\n".getBytes("UTF-8"));
        check(ExtendIdentificationConfig.load(open).motionEnabled, "no limit settings required");
        Files.write(bad.toPath(), "motion_enabled=true\nkp=-1\n".getBytes("UTF-8"));
        boolean rejected = false;
        try { ExtendIdentificationConfig.load(bad); } catch (IOException expected) { rejected = true; }
        check(rejected, "invalid gain rejected");
        System.out.println("ExtendTuneCoreTest passed: " + checks);
    }
}
