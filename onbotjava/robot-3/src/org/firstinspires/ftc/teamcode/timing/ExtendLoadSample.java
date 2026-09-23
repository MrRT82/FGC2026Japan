// 最終変更: 2026-09-23 16:29 JST。計測版専用、制御出力は変更しない。
package org.firstinspires.ftc.teamcode.timing;

/** One current acquisition paired with the latest controller observation, not simultaneous sensors. */
public final class ExtendLoadSample {
    private static final long INTERVAL_NS = 100_000_000L;
    private long lastNs, originNs;
    private int sequence;
    public int initialBalls = -1;
    private String snapshot = "";
    public double currentAmps = Double.NaN;

    static String csvHeader() {
        return ",load_seq,load_initial_balls,load_t_ms,load_current_A,load_current_ok,load_read_ms"
                + ",load_angle_deg,load_velocity_dps,load_angle_age_ms,load_target_deg,load_goal_deg"
                + ",load_requested_out,load_sent_out,load_ff,load_pid,load_i,load_control_dt_ms"
                + ",load_raw_dt_ms,load_stuck,load_reverse,load_stall,load_pote_fault,load_yield"
                + ",load_voltage_V,load_voltage_age_ms,load_period_ms";
    }

    boolean due(long now) { return sequence == 0 || now - lastNs >= INTERVAL_NS; }

    public void sample(SensorReads reads, ExtendController ext, double voltage,
                       double voltageAgeMs, double periodMs) {
        long now = System.nanoTime();
        if (!due(now)) return;
        // Copy control state before the blocking current call. No additional angle/velocity reads.
        double angle = ext.current, velocity = ext.poteVel;
        double angleAge = ext.observationNs == 0 ? -1 : (now - ext.observationNs) / 1e6;
        double target = ext.observedControlTarget, goal = ext.getGoal();
        double requested = ext.output, sent = ext.getSentOutput();
        double ff = ext.ffOut, pid = ext.pidOut, i = ext.ki * ext.integral;
        double dt = ext.getControlDt() * 1000, rawDt = ext.observedRawDt * 1000;
        int stuck = ext.isStuck ? 1 : 0, reverse = ext.reverse ? 1 : 0;
        int stall = ext.stallFault ? 1 : 0, fault = ext.poteFault ? 1 : 0, yield = ext.yielding ? 1 : 0;
        currentAmps = Double.NaN;
        boolean ok = false;
        try {
            currentAmps = reads.current(ext.extend, SensorTiming.DIAG_EXTEND);
            ok = !Double.isNaN(currentAmps) && !Double.isInfinite(currentAmps) && currentAmps >= 0;
        } catch (Exception ignored) { /* Invalid samples remain NaN; never turn a failure into zero load. */ }
        long end = System.nanoTime();
        if (sequence == 0) originNs = now;
        lastNs = now;
        sequence++;
        StringBuilder sb = new StringBuilder(512);
        sb.append(',').append(sequence).append(',').append(initialBalls)
          .append(',').append((now - originNs) / 1e6).append(',').append(currentAmps)
          .append(',').append(ok ? 1 : 0).append(',').append((end - now) / 1e6)
          .append(',').append(angle).append(',').append(velocity).append(',').append(angleAge)
          .append(',').append(target).append(',').append(goal)
          .append(',').append(requested).append(',').append(sent)
          .append(',').append(ff).append(',').append(pid).append(',').append(i)
          .append(',').append(dt).append(',').append(rawDt)
          .append(',').append(stuck).append(',').append(reverse).append(',').append(stall)
          .append(',').append(fault).append(',').append(yield)
          .append(',').append(voltage).append(',').append(voltageAgeMs).append(',').append(periodMs);
        snapshot = sb.toString();
    }

    public void append(StringBuilder sb) { sb.append(snapshot); }
}
