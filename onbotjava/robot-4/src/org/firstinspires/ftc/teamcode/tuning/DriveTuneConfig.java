package org.firstinspires.ftc.teamcode.tuning;

import java.io.*;
import java.util.Properties;

/** Robot 4 drive only. No motion until motion_enabled=true and every travel limit is valid. */
public final class DriveTuneConfig {
    public static final String PATH = "/sdcard/FIRST/robot4_drive_tune.properties";
    public boolean motionEnabled;
    public String gearRatio = "unknown", motorModel = "unknown";
    /** Actual motor revolutions per wheel revolution; converts encoder ticks to metres. */
    public double gearReduction = Double.NaN, wheelDiameterMm = Double.NaN, countsPerMotorRev = 28;
    public double maxPower = Double.NaN, maxTravelM = Double.NaN, maxTurnDeg = Double.NaN;
    public double runMs = 1500, brakeTimeoutMs = 3000, headingMs = 3000;
    /** Command change per second. 0 = step, as in the match code. */
    public double accelPerSec = 0, brakePerSec = 0;
    public double[] powers = {0.3, 0.5, 0.7, 1.0};
    public double[] headingSteps = {45, 90, 180};
    /** NaN keeps the match-code ImuController value. */
    public double imuKp = Double.NaN, imuKd = Double.NaN, imuKs = Double.NaN, imuMaxTurnPower = Double.NaN;

    public static DriveTuneConfig load(File file) throws IOException {
        DriveTuneConfig c = new DriveTuneConfig();
        if (!file.exists()) return c;
        Properties p = new Properties();
        try (InputStream in = new FileInputStream(file)) { p.load(in); }
        c.motionEnabled = Boolean.parseBoolean(p.getProperty("motion_enabled", "false").trim());
        c.gearRatio = p.getProperty("gear_ratio", "unknown").trim();
        c.motorModel = p.getProperty("motor_model", "unknown").trim();
        c.gearReduction = number(p, "gear_reduction", c.gearReduction);
        c.wheelDiameterMm = number(p, "wheel_diameter_mm", c.wheelDiameterMm);
        c.countsPerMotorRev = number(p, "counts_per_motor_rev", c.countsPerMotorRev);
        c.maxPower = number(p, "max_power", c.maxPower);
        c.maxTravelM = number(p, "max_travel_m", c.maxTravelM);
        c.maxTurnDeg = number(p, "max_turn_deg", c.maxTurnDeg);
        c.runMs = number(p, "run_ms", c.runMs);
        c.brakeTimeoutMs = number(p, "brake_timeout_ms", c.brakeTimeoutMs);
        c.headingMs = number(p, "heading_ms", c.headingMs);
        c.accelPerSec = number(p, "accel_per_s", c.accelPerSec);
        c.brakePerSec = number(p, "brake_per_s", c.brakePerSec);
        c.powers = list(p, "powers", c.powers);
        c.headingSteps = list(p, "heading_steps_deg", c.headingSteps);
        c.imuKp = number(p, "imu_kp", c.imuKp);
        c.imuKd = number(p, "imu_kd", c.imuKd);
        c.imuKs = number(p, "imu_ks", c.imuKs);
        c.imuMaxTurnPower = number(p, "imu_max_turn_power", c.imuMaxTurnPower);
        if (c.motionEnabled && !c.valid()) throw new IOException("Invalid drive tune settings: " + file);
        return c;
    }
    private static double number(Properties p, String key, double fallback) {
        String v = p.getProperty(key);
        return v == null ? fallback : Double.parseDouble(v.trim());
    }
    private static double[] list(Properties p, String key, double[] fallback) {
        String v = p.getProperty(key);
        if (v == null) return fallback;
        String[] parts = v.split(",");
        double[] out = new double[parts.length];
        for (int n = 0; n < parts.length; n++) out[n] = Double.parseDouble(parts[n].trim());
        return out;
    }
    static boolean finite(double v) { return !Double.isNaN(v) && !Double.isInfinite(v); }
    private static boolean positive(double v) { return finite(v) && v > 0; }
    private static boolean optionalGain(double v) { return Double.isNaN(v) || (finite(v) && v >= 0); }

    public boolean valid() {
        if (!positive(gearReduction) || !positive(wheelDiameterMm) || !positive(countsPerMotorRev)) return false;
        if (!positive(maxPower) || maxPower > 1) return false;
        if (!positive(maxTravelM) || !positive(maxTurnDeg) || maxTurnDeg > 720) return false;
        if (!positive(runMs) || runMs > 5000 || !positive(brakeTimeoutMs) || brakeTimeoutMs > 10000) return false;
        if (!positive(headingMs) || headingMs > 10000) return false;
        if (!finite(accelPerSec) || accelPerSec < 0 || !finite(brakePerSec) || brakePerSec < 0) return false;
        if (powers.length == 0 || headingSteps.length == 0) return false;
        for (double v : powers) if (!positive(v) || v > 1) return false;
        for (double v : headingSteps) if (!positive(v) || v > 180) return false;
        return optionalGain(imuKp) && optionalGain(imuKd) && optionalGain(imuKs)
            && (Double.isNaN(imuMaxTurnPower) || (positive(imuMaxTurnPower) && imuMaxTurnPower <= 1));
    }
    public double metresPerTick() {
        return Math.PI * wheelDiameterMm / 1000.0 / (countsPerMotorRev * gearReduction);
    }
    public static String text(double[] values) {
        StringBuilder b = new StringBuilder();
        for (int n = 0; n < values.length; n++) b.append(n == 0 ? "" : ",").append(values[n]);
        return b.toString();
    }
}
