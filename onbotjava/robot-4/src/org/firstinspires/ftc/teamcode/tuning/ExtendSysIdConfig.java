package org.firstinspires.ftc.teamcode.tuning;

import java.io.*;
import java.util.Properties;

/** Robot 4 extension identification. No motion is permitted until measured limits are supplied. */
public final class ExtendSysIdConfig {
    public static final String PATH = "/sdcard/FIRST/robot4_extend_sysid.properties";
    public boolean motionEnabled;
    public double minAngle = Double.NaN, maxAngle = Double.NaN;
    public double motorLimitA = Double.NaN, hubLimitA = Double.NaN, minVoltage = Double.NaN;
    public double maxDuty = Double.NaN;
    // Effort is commanded in volts. The analysis rejects breakaway ramps faster than 2 V/s.
    public double rampVPerS = 1.6, rampBackoffV = 1.0, initialRestV = 0;
    public double taperVPerS = 6, taperSlowVPerS = .3, taperSlowBelowDps = 6, taperSlowFraction = .75,
        taperSlowMarginV = .15;
    public double[] slideScales = {0.85, 1.0, 1.15, 1.3};
    public double longStepDeg = 12, shortStepDeg = 6, guardDeg = 4;
    public double preHoldS = .6, postHoldS = .8, stillBandDeg = .5, motionDetectDeg = 1.0;
    public double slideMaxDps = 30, overspeedDps = 60, slideMaxS = 3, rampMaxS = 8;
    /** Every pulseEvery-th up/down pair is a pulse (validation replay); 0 disables pulses. */
    public int pulseEvery = 4;
    public double pulseScale = 1.25, pulseS = .2;
    public int maxEvents = 32;
    public double maxRuntimeS = 180;
    public int angleSamples = 3, motorHubIndex = -1, balls = -1;
    public String gearRatio = "unknown", motorModel = "unknown";

    public static ExtendSysIdConfig load(File file) throws IOException {
        ExtendSysIdConfig c = new ExtendSysIdConfig();
        if (!file.exists()) return c;
        Properties p = new Properties();
        try (InputStream in = new FileInputStream(file)) { p.load(in); }
        c.motionEnabled = Boolean.parseBoolean(p.getProperty("motion_enabled", "false"));
        c.minAngle = number(p, "min_angle_deg", c.minAngle);
        c.maxAngle = number(p, "max_angle_deg", c.maxAngle);
        c.motorLimitA = number(p, "motor_limit_A", c.motorLimitA);
        c.hubLimitA = number(p, "hub_limit_A", c.hubLimitA);
        c.minVoltage = number(p, "min_voltage_V", c.minVoltage);
        c.maxDuty = number(p, "max_duty", c.maxDuty);
        c.rampVPerS = number(p, "ramp_V_per_s", c.rampVPerS);
        c.rampBackoffV = number(p, "ramp_backoff_V", c.rampBackoffV);
        c.initialRestV = number(p, "initial_rest_V", c.initialRestV);
        c.taperVPerS = number(p, "taper_V_per_s", c.taperVPerS);
        c.taperSlowVPerS = number(p, "taper_slow_V_per_s", c.taperSlowVPerS);
        c.taperSlowBelowDps = number(p, "taper_slow_below_dps", c.taperSlowBelowDps);
        c.taperSlowFraction = number(p, "taper_slow_fraction", c.taperSlowFraction);
        c.taperSlowMarginV = number(p, "taper_slow_margin_V", c.taperSlowMarginV);
        c.slideScales = numbers(p, "slide_scales", c.slideScales);
        c.longStepDeg = number(p, "long_step_deg", c.longStepDeg);
        c.shortStepDeg = number(p, "short_step_deg", c.shortStepDeg);
        c.guardDeg = number(p, "guard_deg", c.guardDeg);
        c.preHoldS = number(p, "pre_hold_s", c.preHoldS);
        c.postHoldS = number(p, "post_hold_s", c.postHoldS);
        c.stillBandDeg = number(p, "still_band_deg", c.stillBandDeg);
        c.motionDetectDeg = number(p, "motion_detect_deg", c.motionDetectDeg);
        c.slideMaxDps = number(p, "slide_max_dps", c.slideMaxDps);
        c.overspeedDps = number(p, "overspeed_dps", c.overspeedDps);
        c.slideMaxS = number(p, "slide_max_s", c.slideMaxS);
        c.rampMaxS = number(p, "ramp_max_s", c.rampMaxS);
        c.pulseEvery = Integer.parseInt(p.getProperty("pulse_every", Integer.toString(c.pulseEvery)).trim());
        c.pulseScale = number(p, "pulse_scale", c.pulseScale);
        c.pulseS = number(p, "pulse_s", c.pulseS);
        c.maxEvents = Integer.parseInt(p.getProperty("max_events", Integer.toString(c.maxEvents)).trim());
        c.maxRuntimeS = number(p, "max_runtime_s", c.maxRuntimeS);
        c.angleSamples = Integer.parseInt(p.getProperty("angle_samples", Integer.toString(c.angleSamples)).trim());
        c.motorHubIndex = Integer.parseInt(p.getProperty("motor_hub_index", Integer.toString(c.motorHubIndex)).trim());
        c.balls = Integer.parseInt(p.getProperty("balls", "-1").trim());
        c.gearRatio = p.getProperty("gear_ratio", "unknown").trim();
        c.motorModel = p.getProperty("motor_model", "unknown").trim();
        if (c.motionEnabled && !c.valid()) throw new IOException("Invalid identification limits: " + file);
        return c;
    }
    private static double number(Properties p, String key, double fallback) {
        return Double.parseDouble(p.getProperty(key, Double.toString(fallback)).trim());
    }
    private static double[] numbers(Properties p, String key, double[] fallback) {
        String text = p.getProperty(key);
        if (text == null) return fallback;
        String[] parts = text.split(",");
        double[] result = new double[parts.length];
        for (int n = 0; n < parts.length; n++) result[n] = Double.parseDouble(parts[n].trim());
        return result;
    }
    public String scalesText() {
        StringBuilder b = new StringBuilder();
        for (int n = 0; n < slideScales.length; n++) b.append(n == 0 ? "" : ",").append(slideScales[n]);
        return b.toString();
    }
    public boolean valid() {
        boolean scalesOk = slideScales.length > 0;
        for (double s : slideScales) scalesOk &= ExtendTuneConfig.finite(s) && s >= .5 && s <= 2;
        return ExtendTuneConfig.finite(minAngle) && ExtendTuneConfig.finite(maxAngle)
            && minAngle >= 0 && maxAngle <= 88 && positive(guardDeg) && positive(longStepDeg)
            && positive(shortStepDeg) && maxAngle - minAngle >= 2 * guardDeg + 1.5 * (longStepDeg + shortStepDeg)
            && positive(motorLimitA) && positive(hubLimitA) && positive(minVoltage)
            && positive(maxDuty) && maxDuty <= .6
            && positive(rampVPerS) && rampVPerS <= 1.8
            && ExtendTuneConfig.finite(rampBackoffV) && rampBackoffV >= 0
            && ExtendTuneConfig.finite(initialRestV) && Math.abs(initialRestV) <= 3
            && positive(taperVPerS) && positive(taperSlowVPerS) && taperSlowVPerS <= taperVPerS
            && positive(taperSlowBelowDps) && taperSlowFraction > 0 && taperSlowFraction < 1
            && ExtendTuneConfig.finite(taperSlowMarginV) && taperSlowMarginV >= 0 && scalesOk
            && preHoldS >= .4 && postHoldS >= .4
            && positive(stillBandDeg) && motionDetectDeg > stillBandDeg
            && positive(slideMaxDps) && slideMaxDps < overspeedDps && overspeedDps <= 90
            && positive(slideMaxS) && positive(rampMaxS)
            && pulseEvery >= 0 && ExtendTuneConfig.finite(pulseScale) && pulseScale >= 1.05 && pulseScale <= 2
            && positive(pulseS) && pulseS <= 1
            && maxEvents >= 4 && maxEvents <= 200 && positive(maxRuntimeS) && maxRuntimeS <= 600
            && angleSamples >= 1 && angleSamples <= 4 && motorHubIndex >= -1;
    }
    private static boolean positive(double v) { return ExtendTuneConfig.finite(v) && v > 0; }
}
