package org.firstinspires.ftc.teamcode.tuning;

import java.io.*;
import java.util.Properties;

/** Robot 4 only. No motion is permitted until measured limits are supplied. */
public final class ExtendTuneConfig {
    public static final String PATH = "/sdcard/FIRST/robot4_extend_tune.properties";
    public boolean motionEnabled;
    public double minAngle = Double.NaN, maxAngle = Double.NaN;
    public double motorLimitA = Double.NaN, hubLimitA = Double.NaN, minVoltage = Double.NaN;
    public double maxPower = Double.NaN;
    public double kp = .0175, ki = .005, kd = 0;
    public double gravityCos = .0638, gravityConstant = .0694, gravityPhaseDeg = 17, friction = .123;
    public int balls = -1;
    public String gearRatio = "unknown", motorModel = "unknown";
    /** Extend_TuneAuto_R4 schedule (ignored by Extend_Tune_R4). */
    public double[] autoStations = {30, 45, 60};
    public double autoStepDeg = 5, autoHoldS = 3, autoStepS = 4, autoArriveS = 8, autoSlewDps = 10;

    public static ExtendTuneConfig load(File file) throws IOException {
        ExtendTuneConfig c = new ExtendTuneConfig();
        if (!file.exists()) return c;
        Properties p = new Properties();
        try (InputStream in = new FileInputStream(file)) { p.load(in); }
        c.motionEnabled = Boolean.parseBoolean(p.getProperty("motion_enabled", "false"));
        c.minAngle = number(p, "min_angle_deg", c.minAngle);
        c.maxAngle = number(p, "max_angle_deg", c.maxAngle);
        c.motorLimitA = number(p, "motor_limit_A", c.motorLimitA);
        c.hubLimitA = number(p, "hub_limit_A", c.hubLimitA);
        c.minVoltage = number(p, "min_voltage_V", c.minVoltage);
        c.maxPower = number(p, "max_power", c.maxPower);
        c.kp = number(p, "kp", c.kp); c.ki = number(p, "ki", c.ki); c.kd = number(p, "kd", c.kd);
        c.gravityCos = number(p, "gravity_cos", c.gravityCos);
        c.gravityConstant = number(p, "gravity_constant", c.gravityConstant);
        c.gravityPhaseDeg = number(p, "gravity_phase_deg", c.gravityPhaseDeg);
        c.friction = number(p, "friction", c.friction);
        c.balls = Integer.parseInt(p.getProperty("balls", "-1"));
        c.gearRatio = p.getProperty("gear_ratio", "unknown");
        c.motorModel = p.getProperty("motor_model", "unknown");
        String stations = p.getProperty("auto_stations");
        if (stations != null) {
            String[] parts = stations.split(",");
            c.autoStations = new double[parts.length];
            for (int n = 0; n < parts.length; n++) c.autoStations[n] = Double.parseDouble(parts[n].trim());
        }
        c.autoStepDeg = number(p, "auto_step_deg", c.autoStepDeg);
        c.autoHoldS = number(p, "auto_hold_s", c.autoHoldS);
        c.autoStepS = number(p, "auto_step_s", c.autoStepS);
        c.autoArriveS = number(p, "auto_arrive_s", c.autoArriveS);
        c.autoSlewDps = number(p, "auto_slew_dps", c.autoSlewDps);
        if (c.motionEnabled && !c.valid()) throw new IOException("Invalid motion limits: " + file);
        return c;
    }
    private static double number(Properties p, String key, double fallback) {
        return Double.parseDouble(p.getProperty(key, Double.toString(fallback)));
    }
    static boolean finite(double v) { return !Double.isNaN(v) && !Double.isInfinite(v); }
    public boolean valid() {
        return finite(minAngle) && finite(maxAngle) && minAngle >= 0 && maxAngle <= 88
            && maxAngle - minAngle >= 2 && positive(motorLimitA) && positive(hubLimitA)
            && positive(minVoltage) && positive(maxPower) && maxPower <= .35
            && finite(kp) && kp >= 0 && finite(ki) && ki >= 0 && finite(kd) && kd >= 0
            && finite(gravityCos) && finite(gravityConstant) && finite(gravityPhaseDeg)
            && finite(friction) && friction >= 0;
    }
    private static boolean positive(double v) { return finite(v) && v > 0; }
    /** Every station and station +/- step stays 2 deg inside the limits. */
    public boolean autoValid() {
        if (!valid() || autoStations.length == 0 || !positive(autoStepDeg) || autoStepDeg > 10
            || !positive(autoHoldS) || !positive(autoStepS) || !positive(autoArriveS)
            || !positive(autoSlewDps) || autoSlewDps > 30) return false;
        for (double s : autoStations)
            if (!finite(s) || s - autoStepDeg < minAngle + 2 || s + autoStepDeg > maxAngle - 2) return false;
        return true;
    }
    public String autoStationsText() {
        StringBuilder b = new StringBuilder();
        for (int n = 0; n < autoStations.length; n++) b.append(n == 0 ? "" : ",").append(autoStations[n]);
        return b.toString();
    }
}
