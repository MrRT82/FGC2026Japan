package org.firstinspires.ftc.teamcode.tuning;

import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;

/** Checks legacy Auto isolation from identification-only validation. No hardware. */
public final class ExtendConfigCompatibilityTest {
    private static int checks;
    private static void check(boolean ok, String why) {
        checks++;
        if (!ok) throw new AssertionError(why);
    }
    public static void main(String[] args) throws Exception {
        File file = File.createTempFile("extend-compat-", ".properties");
        try {
            String base = "motion_enabled=true\nmin_angle_deg=21\nmax_angle_deg=75\n"
                + "motor_limit_A=2\nhub_limit_A=8\nmin_voltage_V=11\nmax_power=.35\n"
                + "kp=.03\nki=.005\nkd=0\ngravity_cos=.131\ngravity_constant=0\n"
                + "gravity_phase_deg=-10\nfriction=.15\nballs=20\n"
                + "auto_stations=30,45,60\nauto_step_deg=5\nauto_hold_s=3\n"
                + "auto_step_s=4\nauto_arrive_s=8\nauto_slew_dps=27\nauto_max_power=.5\nauto_kv_up=.01\n";
            Files.write(file.toPath(), base.getBytes(StandardCharsets.UTF_8));
            ExtendTuneConfig legacy = ExtendTuneConfig.load(file);
            ExtendIdentificationConfig v2 = ExtendIdentificationConfig.load(file);
            check(legacy.autoValid() && v2.valid(), "both readers accept common settings");
            check(legacy.kp == v2.kp && legacy.friction == v2.friction
                && legacy.gravityCos == v2.gravityCos && legacy.gravityPhaseDeg == v2.gravityPhaseDeg,
                "gains preserved");
            check(v2.trialMode.equals("PID") && v2.motorHubIndex == -1, "safe v2 defaults");
            check(legacy.autoStations.length == 3 && legacy.autoStations[2] == 60
                && legacy.autoStepS == 4 && legacy.autoSlewDps == 27, "Auto schedule preserved");
            legacy.autoSlewDps = 100; legacy.autoMaxPower = Double.NaN; legacy.autoKvUp = .1;
            legacy.autoStations = new double[] {5, 85};
            check(legacy.autoValid(), "no Auto slew, output, kV or station-range caps");
            legacy.autoSlewDps = 27; legacy.autoKvUp = .01;
            legacy.autoStations = new double[] {60};
            ExtendTuneAutoCore slew = new ExtendTuneAutoCore(legacy);
            slew.update(40, 1_000_000_000L, true, true, "", true);
            check(Math.abs(slew.target - 40.54) < 1e-9, "configured 27 deg/s Auto target slew");
            check(Math.abs(slew.sent) <= 1, "Auto output bound is full duty only");
            check(slew.ff >= .27, "upward velocity feedforward applied immediately");
            legacy.autoStations = new double[] {20}; legacy.autoKvDown = .01;
            ExtendTuneAutoCore down = new ExtendTuneAutoCore(legacy);
            down.update(40, 1_000_000_000L, true, true, "", true);
            check(Math.abs(down.target - 39.46) < 1e-9 && down.ff < slew.ff - .27, "downward velocity feedforward applied");
            legacy.autoKvDown = -.01; check(!legacy.autoValid(), "negative downward kV rejected");
            legacy.autoKvDown = 0;
            legacy.autoStations = new double[] {30, 45, 60};
            Files.write(file.toPath(), (base + "trial_mode=BREAKAWAY_POS\n").getBytes(StandardCharsets.UTF_8));
            check(ExtendTuneConfig.load(file).autoValid(), "v2 fields do not change legacy validation");
            boolean rejected = false;
            try { ExtendIdentificationConfig.load(file); } catch (java.io.IOException expected) { rejected = true; }
            check(rejected, "unconfigured breakaway rejected by v2");
            legacy.autoHoldS = 20;
            ExtendTuneAutoCore auto = new ExtendTuneAutoCore(legacy);
            for (int n = 0; n <= 450; n++)
                auto.update(30, 1_000_000_000L + n * 20_000_000L, true, n == 0, "", true);
            check(auto.state.equals("ACTIVE") && auto.fault.isEmpty(), "legacy Auto not limited to v2 eight seconds");
            auto.release();
            check(auto.sent == 0, "legacy release zeros output");
        } finally { Files.deleteIfExists(file.toPath()); }
        System.out.println("ExtendConfigCompatibilityTest passed: " + checks);
    }
}
