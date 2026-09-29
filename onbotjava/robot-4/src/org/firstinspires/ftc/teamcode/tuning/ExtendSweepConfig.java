package org.firstinspires.ftc.teamcode.tuning;

import java.io.*;
import java.util.Properties;

/** Robot 4 gain sweep at match-level limits. No motion until measured limits are supplied. */
public final class ExtendSweepConfig {
    public static final String PATH = "/sdcard/FIRST/robot4_extend_sweep.properties";
    public boolean motionEnabled;
    public double minAngle = Double.NaN, maxAngle = Double.NaN;
    public double motorLimitA = Double.NaN, hubLimitA = Double.NaN, minVoltage = Double.NaN;
    public double maxPower = Double.NaN, slewDps = 100, overspeedDps = 150;
    public double gravityCos = .131, gravityConstant = 0, gravityPhaseDeg = -10, friction = .15, ki = .005;
    /** Candidates run in order (kp outer, kd inner): put the gentlest first. */
    public double[] kps = {.03, .05, .08, .12, .18}, kds = {0, .001, .003};
    public double startDeg = 25, holdS = 2.5, arriveS = 6;
    public double[] goals = {70, 25, 55, 40, 55};
    public int balls = -1;
    public String gearRatio = "unknown", motorModel = "unknown";

    public static ExtendSweepConfig load(File file) throws IOException {
        ExtendSweepConfig c = new ExtendSweepConfig();
        if (!file.exists()) return c;
        Properties p = new Properties();
        try (InputStream in = new FileInputStream(file)) { p.load(in); }
        c.motionEnabled = Boolean.parseBoolean(p.getProperty("motion_enabled", "false").trim());
        c.minAngle = number(p, "min_angle_deg", c.minAngle);
        c.maxAngle = number(p, "max_angle_deg", c.maxAngle);
        c.motorLimitA = number(p, "motor_limit_A", c.motorLimitA);
        c.hubLimitA = number(p, "hub_limit_A", c.hubLimitA);
        c.minVoltage = number(p, "min_voltage_V", c.minVoltage);
        c.maxPower = number(p, "max_power", c.maxPower);
        c.slewDps = number(p, "slew_dps", c.slewDps);
        c.overspeedDps = number(p, "overspeed_dps", c.overspeedDps);
        c.gravityCos = number(p, "gravity_cos", c.gravityCos);
        c.gravityConstant = number(p, "gravity_constant", c.gravityConstant);
        c.gravityPhaseDeg = number(p, "gravity_phase_deg", c.gravityPhaseDeg);
        c.friction = number(p, "friction", c.friction);
        c.ki = number(p, "ki", c.ki);
        c.kps = numbers(p, "sweep_kp", c.kps);
        c.kds = numbers(p, "sweep_kd", c.kds);
        c.startDeg = number(p, "start_deg", c.startDeg);
        c.holdS = number(p, "hold_s", c.holdS);
        c.arriveS = number(p, "arrive_s", c.arriveS);
        c.goals = numbers(p, "goals", c.goals);
        c.balls = Integer.parseInt(p.getProperty("balls", "-1").trim());
        c.gearRatio = p.getProperty("gear_ratio", "unknown").trim();
        c.motorModel = p.getProperty("motor_model", "unknown").trim();
        if (c.motionEnabled && !c.valid()) throw new IOException("Invalid sweep limits: " + file);
        return c;
    }
    private static double number(Properties p, String key, double fallback) {
        return Double.parseDouble(p.getProperty(key, Double.toString(fallback)).trim());
    }
    private static double[] numbers(Properties p, String key, double[] fallback) {
        String text = p.getProperty(key);
        if (text == null) return fallback;
        String[] parts = text.split(",");
        double[] out = new double[parts.length];
        for (int n = 0; n < parts.length; n++) out[n] = Double.parseDouble(parts[n].trim());
        return out;
    }
    static String text(double[] values) {
        StringBuilder b = new StringBuilder();
        for (int n = 0; n < values.length; n++) b.append(n == 0 ? "" : ",").append(values[n]);
        return b.toString();
    }
    public int candidates() { return kps.length * kds.length; }
    public double kp(int candidate) { return kps[candidate / kds.length]; }
    public double kd(int candidate) { return kds[candidate % kds.length]; }
    private boolean inside(double deg) { return ExtendTuneConfig.finite(deg) && deg >= minAngle + 2 && deg <= maxAngle - 3; }
    public boolean valid() {
        boolean ok = ExtendTuneConfig.finite(minAngle) && ExtendTuneConfig.finite(maxAngle)
            && minAngle >= 0 && maxAngle <= 88 && maxAngle - minAngle >= 10
            && positive(motorLimitA) && motorLimitA <= 4.4 && positive(hubLimitA) && positive(minVoltage)
            && positive(maxPower) && maxPower <= 1 && positive(slewDps) && slewDps <= 150
            && positive(overspeedDps) && overspeedDps <= 200
            && ExtendTuneConfig.finite(gravityCos) && ExtendTuneConfig.finite(gravityConstant)
            && ExtendTuneConfig.finite(gravityPhaseDeg) && ExtendTuneConfig.finite(friction) && friction >= 0
            && ExtendTuneConfig.finite(ki) && ki >= 0 && kps.length > 0 && kds.length > 0
            && candidates() <= 24 && goals.length > 0 && inside(startDeg)
            && positive(holdS) && positive(arriveS);
        for (double k : kps) ok &= ExtendTuneConfig.finite(k) && k > 0 && k <= .5;
        for (double k : kds) ok &= ExtendTuneConfig.finite(k) && k >= 0 && k <= .05;
        for (double g : goals) ok &= inside(g) && Math.abs(g - startDeg) <= 60;
        return ok;
    }
    private static boolean positive(double v) { return ExtendTuneConfig.finite(v) && v > 0; }
}
