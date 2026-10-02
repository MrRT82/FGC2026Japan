package org.firstinspires.ftc.teamcode.tuning;

import java.io.*;
import java.util.Properties;

/** Robot 4 only. No motion until motion_enabled=true. */
public final class ExtendIdentificationConfig {
    public static final String PATH = "/sdcard/FIRST/robot4_extend_tune.properties";
    public boolean motionEnabled;
    public double minAngle = Double.NaN, maxAngle = Double.NaN;
    public double motorLimitA = Double.NaN, hubLimitA = Double.NaN, minVoltage = Double.NaN;
    public double maxPower = Double.NaN;
    public double kp = .0175, ki = .005, kd = 0;
    public double gravityCos = .0638, gravityConstant = .0694, gravityPhaseDeg = 17, friction = .123;
    public int balls = -1;
    public String gearRatio = "unknown", motorModel = "unknown";
    public String trialMode = "PID", datasetRole = "unspecified";
    public int motorHubIndex = -1;
    public int motorToAngleSign = 1;
    public double stepDeg = 5, slewDps = 10;
    public double rampRateVps = .4, rampMaxDeltaV = 1, rampMaxExcursionDeg = 3;
    public boolean rampMode() { return trialMode.startsWith("BREAKAWAY_"); }
    public boolean motorRampMode() { return trialMode.startsWith("BREAKAWAY_MOTOR_"); }
    public int direction() { return trialMode.endsWith("NEG") ? -1 : 1; }

    public static ExtendIdentificationConfig load(File file) throws IOException {
        ExtendIdentificationConfig c = new ExtendIdentificationConfig();
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
        c.trialMode = p.getProperty("trial_mode", "PID");
        c.datasetRole = p.getProperty("dataset_role", "unspecified");
        c.motorHubIndex = Integer.parseInt(p.getProperty("motor_hub_index", "-1"));
        c.motorToAngleSign = Integer.parseInt(p.getProperty("motor_to_angle_sign", "1"));
        c.stepDeg = number(p, "step_deg", c.stepDeg); c.slewDps = number(p, "slew_dps", c.slewDps);
        c.rampRateVps = number(p, "ramp_rate_V_per_s", c.rampRateVps);
        c.rampMaxDeltaV = number(p, "ramp_max_delta_V", c.rampMaxDeltaV);
        c.rampMaxExcursionDeg = number(p, "ramp_max_excursion_deg", c.rampMaxExcursionDeg);
        if (c.motionEnabled && !c.valid()) throw new IOException("Invalid trial settings: " + file);
        return c;
    }
    private static double number(Properties p, String key, double fallback) {
        return Double.parseDouble(p.getProperty(key, Double.toString(fallback)));
    }
    static boolean finite(double v) { return !Double.isNaN(v) && !Double.isInfinite(v); }
    /** Angle, current, voltage, power and ramp-size settings are recorded only; no limit is enforced. */
    public boolean valid() {
        return finite(kp) && kp >= 0 && finite(ki) && ki >= 0 && finite(kd) && kd >= 0
            && finite(gravityCos) && finite(gravityConstant) && finite(gravityPhaseDeg)
            && finite(friction) && friction >= 0
            && (trialMode.equals("PID") || trialMode.equals("STEP_POS") || trialMode.equals("STEP_NEG")
                || trialMode.equals("BREAKAWAY_POS") || trialMode.equals("BREAKAWAY_NEG")
                || trialMode.equals("BREAKAWAY_MOTOR_POS") || trialMode.equals("BREAKAWAY_MOTOR_NEG"))
            && (datasetRole.equals("unspecified") || datasetRole.equals("train") || datasetRole.equals("validation"))
            && positive(stepDeg) && positive(slewDps) && positive(rampRateVps)
            && (motorToAngleSign == -1 || motorToAngleSign == 1)
            && (trialMode.equals("PID") || (balls >= 0 && !datasetRole.equals("unspecified")))
            && (!rampMode() || motorHubIndex >= 0);
    }
    private static boolean positive(double v) { return finite(v) && v > 0; }
}
