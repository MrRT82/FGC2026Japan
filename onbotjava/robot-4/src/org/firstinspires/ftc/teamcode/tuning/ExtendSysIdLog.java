package org.firstinspires.ftc.teamcode.tuning;

import java.io.*;
import java.util.List;

/** CSV/metadata layout shared by Extend_SysId_R4 and the offline simulation. Times are ms since log origin. */
final class ExtendSysIdLog {
    /** One electrical signal with its own acquisition time and sequence; never a simultaneous snapshot. */
    static final class Reading {
        double value = Double.NaN, durationMs, stampMs = Double.NaN;
        long seq;
        boolean ok;
        double age(double nowMs) { return Double.isNaN(stampMs) ? -1 : nowMs - stampMs; }
        String csv(double nowMs) {
            return "," + value + "," + (ok ? 1 : 0) + "," + seq + "," + age(nowMs) + "," + durationMs;
        }
    }

    private ExtendSysIdLog() { }

    static String readingHeader(String name) {
        return "," + name + "," + name + "_ok," + name + "_seq," + name + "_age_ms," + name + "_read_ms";
    }

    /** Readings: motor current, then voltage/current for each Hub (same order as Extend_Tune_R4). */
    static String header(int hubs) {
        StringBuilder h = new StringBuilder("t_ms,started,state,fault,phase,event,direction,angle_deg,pot_V,pot_max_V,"
            + "angle_read_ms,angle_t_ms,velocity_dps,target_deg,goal_deg,origin_deg,command_V,rest_V,motor_hub_V,"
            + "requested_out,sent_out,saturated,dt_ms,period_ms,output_t_ms,deadman,balls,writer_pending,last_sync_ms");
        h.append(readingHeader("motor_current_A"));
        for (int n = 0; n < hubs; n++) h.append(readingHeader("hub_" + n + "_voltage_V")).append(readingHeader("hub_" + n + "_current_A"));
        return h.toString();
    }

    static String voltageSignal(int hub) { return "hub_" + hub + "_voltage_V"; }

    /** target_deg is NaN: there is no closed-loop target, so tracking statistics must not be computed. */
    static String row(double nowMs, boolean started, ExtendSysIdCore c, double potV, double maxV,
                      double angleReadMs, double angleTMs, double periodMs, double outputTMs,
                      boolean deadman, int balls, int pending, double lastSyncMs, Reading[] readings) {
        StringBuilder r = new StringBuilder(900);
        r.append(nowMs).append(',').append(started ? 1 : 0).append(',').append(c.state).append(',').append(c.fault)
            .append(',').append(c.phase).append(',').append(c.event).append(',').append(c.direction)
            .append(',').append(c.angle).append(',').append(potV).append(',').append(maxV)
            .append(',').append(angleReadMs).append(',').append(angleTMs).append(',').append(c.velocity)
            .append(",NaN,").append(c.goal).append(',').append(c.origin)
            .append(',').append(c.commandV).append(',').append(c.restV).append(',').append(c.motorHubV)
            .append(',').append(c.requested).append(',').append(c.sent).append(',').append(c.saturated ? 1 : 0)
            .append(',').append(c.dt * 1000).append(',').append(periodMs).append(',').append(outputTMs)
            .append(',').append(deadman ? 1 : 0).append(',').append(balls).append(',').append(pending)
            .append(',').append(lastSyncMs);
        for (Reading reading : readings) r.append(reading.csv(nowMs));
        return r.toString();
    }

    static void metadata(File file, String build, ExtendSysIdConfig c, int motorHub,
                         double potMaxV, List<String> hubInfo) throws IOException {
        try (PrintWriter w = new PrintWriter(new OutputStreamWriter(new FileOutputStream(file), "UTF-8"))) {
            w.println("build=" + build); w.println("robot=4"); w.println("expected_adb_serial=dddad6c23c4eeb3b");
            w.println("experiment=extend_sysid_stick_slip");
            w.println("motion_enabled=" + c.motionEnabled); w.println("balls=" + c.balls);
            w.println("gear_ratio=" + c.gearRatio); w.println("motor_model=" + c.motorModel);
            w.println("min_angle_deg=" + c.minAngle); w.println("max_angle_deg=" + c.maxAngle);
            w.println("motor_limit_A=" + c.motorLimitA); w.println("hub_limit_A=" + c.hubLimitA);
            w.println("min_voltage_V=" + c.minVoltage); w.println("max_duty=" + c.maxDuty);
            w.println("ramp_V_per_s=" + c.rampVPerS); w.println("ramp_backoff_V=" + c.rampBackoffV);
            w.println("initial_rest_V=" + c.initialRestV); w.println("taper_V_per_s=" + c.taperVPerS);
            w.println("taper_slow_V_per_s=" + c.taperSlowVPerS); w.println("taper_slow_below_dps=" + c.taperSlowBelowDps);
            w.println("taper_slow_fraction=" + c.taperSlowFraction); w.println("taper_slow_margin_V=" + c.taperSlowMarginV);
            w.println("slide_scales=" + c.scalesText());
            w.println("long_step_deg=" + c.longStepDeg); w.println("short_step_deg=" + c.shortStepDeg);
            w.println("guard_deg=" + c.guardDeg); w.println("pre_hold_s=" + c.preHoldS);
            w.println("post_hold_s=" + c.postHoldS); w.println("still_band_deg=" + c.stillBandDeg);
            w.println("motion_detect_deg=" + c.motionDetectDeg); w.println("slide_max_dps=" + c.slideMaxDps);
            w.println("overspeed_dps=" + c.overspeedDps); w.println("slide_max_s=" + c.slideMaxS);
            w.println("ramp_max_s=" + c.rampMaxS); w.println("max_events=" + c.maxEvents);
            w.println("pulse_every=" + c.pulseEvery); w.println("pulse_scale=" + c.pulseScale);
            w.println("pulse_s=" + c.pulseS);
            w.println("max_runtime_s=" + c.maxRuntimeS); w.println("angle_samples=" + c.angleSamples);
            w.println("motor_hub_index=" + motorHub);
            w.println("motor_hub_voltage_signal=" + voltageSignal(motorHub));
            w.println("command=effort_V/motor_hub_V; open loop; no PID");
            w.println("angle_conversion=mean(voltage)/maxVoltage*135; offset=0; pot_max_V=" + potMaxV);
            w.println("zero_power=BRAKE; not a mechanical holding brake");
            for (int n = 0; n < hubInfo.size(); n++) w.println("hub_" + n + "=" + hubInfo.get(n));
            if (w.checkError()) throw new IOException("Metadata write failed");
        }
    }
}
