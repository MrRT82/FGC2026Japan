package org.firstinspires.ftc.teamcode.tuning;

/**
 * Operator-held identification trials; no automatic gain changes or jam reversal.
 * No angle, speed, travel, time, stall or output limits beyond +/-1.
 */
public final class ExtendTuneCore {
    final ExtendIdentificationConfig config;
    public String state = "MONITOR", fault = "", phase = "IDLE", event = "", stopReason = "";
    public double angle, velocity, target, goal, sent, requested, ff, p, i, d, dt;
    public double trialMs = -1, baselineV = Double.NaN, rampDeltaV, commandV = Double.NaN;
    public long trialId, lastStillNs, firstMotionNs;
    private double previousAngle, previousError, integral, center;
    private double quietMin, quietMax, quietSumV, rampCenter, frozenV;
    private int quietCount;
    private long previousNs, startedNs, quietNs, baselineNs, rampNs, motionCandidateNs, confirmedNs;
    private boolean initialized;
    public ExtendTuneCore(ExtendIdentificationConfig config) { this.config = config; }
    public void trip(String reason) {
        if (fault.isEmpty()) fault = reason;
        state = "FAULT"; phase = "STOPPED"; event = "FAULT"; stopReason = fault; zero();
    }
    private void zero() { sent = requested = ff = p = i = d = integral = 0; commandV = 0; }
    private void complete(String reason) {
        state = "COMPLETE"; phase = "STOPPED"; event = "COMPLETE"; stopReason = reason; zero();
    }
    public void release() {
        if (state.equals("ACTIVE")) { event = "RELEASE"; stopReason = "DEADMAN_RELEASE"; }
        if (fault.isEmpty()) { state = "MONITOR"; phase = "IDLE"; }
        zero();
    }
    public void update(double observed, long now, boolean deadman, boolean startEdge,
                       int targetStep, String electricalFault, boolean writerOk) {
        update(observed, now, deadman, startEdge, targetStep, electricalFault, writerOk, Double.NaN);
    }
    public void update(double observed, long now, boolean deadman, boolean startEdge,
                       int targetStep, String electricalFault, boolean writerOk, double hubVoltage) {
        event = "";
        dt = initialized ? (now - previousNs) / 1e9 : .020;
        if (!ExtendIdentificationConfig.finite(observed)) { trip("ANGLE_INVALID"); return; }
        angle = observed;
        if (!initialized) { previousAngle = angle; target = goal = angle; }
        if (initialized && (dt < .002 || dt > .100)) {
            if (state.equals("ACTIVE")) trip("LOOP_DELAY");
            velocity = 0;
        } else if (initialized) velocity += .35 * ((angle - previousAngle) / dt - velocity);
        previousAngle = angle; previousNs = now; initialized = true;
        if (!writerOk) trip("LOGGER_FAILED");
        if (!electricalFault.isEmpty()) {
            if (state.equals("ACTIVE")) trip(electricalFault);
            release(); return;
        }
        if (!fault.isEmpty()) { zero(); return; }
        if (!config.motionEnabled || !config.valid() || !deadman) { release(); return; }
        // The ramp divides by this reading; it is a measurement check, not a voltage limit.
        if (config.rampMode() && (!ExtendIdentificationConfig.finite(hubVoltage) || hubVoltage <= 0)) {
            if (state.equals("ACTIVE")) trip("RAMP_VOLTAGE_INVALID"); else release();
            return;
        }
        if (startEdge && !state.equals("ACTIVE")) {
            center = target = goal = angle; integral = previousError = 0;
            velocity = 0; startedNs = now;
            quietNs = baselineNs = rampNs = motionCandidateNs = confirmedNs = lastStillNs = firstMotionNs = 0;
            quietCount = 0; baselineV = Double.NaN; rampDeltaV = 0; stopReason = "";
            trialId++; state = "ACTIVE"; phase = "SETTLE"; event = "TRIAL_START";
        }
        if (!state.equals("ACTIVE")) { zero(); return; }
        trialMs = (now - startedNs) / 1e6;
        if (config.rampMode() && baselineNs != 0) {
            ramp(now, hubVoltage); return;
        }
        if (config.trialMode.equals("PID")) {
            goal += targetStep * config.stepDeg;
            phase = "PID";
            if (targetStep != 0) event = "TARGET_STEP";
        } else if (!config.rampMode()) {
            goal = center + (trialMs >= 1000 ? config.direction() * config.stepDeg : 0);
            if (phase.equals("SETTLE") && trialMs >= 1000) { phase = "STEP"; event = "TARGET_STEP"; }
            if (trialMs >= 1000 && Math.abs(target - goal) < 1e-8) phase = "FINAL_HOLD";
        }
        target += Math.max(-config.slewDps * dt, Math.min(config.slewDps * dt, goal - target));
        double error = target - angle;
        integral = Math.max(-.5, Math.min(.5, integral + error * dt));
        p = config.kp * error; i = config.ki * integral; d = config.kd * (error - previousError) / dt;
        previousError = error;
        double friction = velocity * Math.signum(error) > 15 ? 0 : config.friction * Math.max(-1, Math.min(1, error / 2));
        ff = config.gravityCos * Math.cos(Math.toRadians(angle + config.gravityPhaseDeg)) + config.gravityConstant + friction;
        requested = ff + p + i + d;
        if (!ExtendIdentificationConfig.finite(requested)) { trip("OUTPUT_INVALID"); return; }
        sent = Math.max(-1, Math.min(1, requested));
        commandV = sent * hubVoltage;
        if (config.rampMode()) {
            // Capture a steady 0.5s holding effort; freeze it before the ramp so PID cannot cancel excitation.
            // The potentiometer changes in discrete steps. Differentiating a one-count change can
            // report more than 1 deg/s while the mechanism is stationary, so use the bounded angle
            // window as the stationary test.
            if (Math.abs(error) > .3 || Math.abs(requested) >= 1) {
                quietNs = 0; quietCount = 0;
            } else {
                if (quietNs == 0 || Math.max(quietMax, angle) - Math.min(quietMin, angle) > .15) {
                    quietNs = now; quietMin = quietMax = angle; quietSumV = 0; quietCount = 0;
                }
                quietMin = Math.min(quietMin, angle); quietMax = Math.max(quietMax, angle);
                quietSumV += sent * hubVoltage; quietCount++;
            }
            if (trialMs >= 1000 && quietNs != 0 && now - quietNs >= 500_000_000L) {
                baselineV = quietSumV / quietCount; baselineNs = now; rampCenter = angle;
                phase = "BASELINE"; event = "BASELINE_CAPTURE";
            } else if (trialMs >= 2500) trip("NOT_SETTLED");
        }
    }
    private void ramp(long now, double hubVoltage) {
        p = i = d = integral = ff = 0;
        double displacement = angle - rampCenter;
        int motorDirection = config.motorRampMode() ? config.direction() : config.direction() * config.motorToAngleSign;
        if (rampNs == 0) {
            if (Math.abs(displacement) > .15) { trip("BASELINE_DRIFT"); return; }
            if (now - baselineNs >= 500_000_000L) { rampNs = now; phase = "RAMP"; event = "RAMP_BEGIN"; lastStillNs = now; }
        }
        if (rampNs != 0) {
            if (config.motorRampMode() && Math.abs(displacement) >= .3) {
                firstMotionNs = now;
                complete("BREAKAWAY_RECORDED");
                return;
            }
            if (config.direction() * displacement < -.3) { trip("WRONG_DIRECTION"); return; }
            if (Math.abs(displacement) <= .15) lastStillNs = now;
            if (config.direction() * displacement >= .3) {
                if (motionCandidateNs == 0) { motionCandidateNs = now; firstMotionNs = now; frozenV = baselineV + motorDirection * rampDeltaV; }
                if (confirmedNs == 0 && now - motionCandidateNs >= 100_000_000L) {
                    confirmedNs = now; phase = "POST_MOTION"; event = "MOTION_CONFIRMED";
                }
            } else if (motionCandidateNs != 0) { trip("MOTION_NOT_SUSTAINED"); return; }
            if (confirmedNs != 0 && now - confirmedNs >= 150_000_000L) { complete("BREAKAWAY_RECORDED"); return; }
            if (motionCandidateNs == 0) rampDeltaV = config.rampRateVps * (now - rampNs) / 1e9;
        }
        commandV = motionCandidateNs != 0 ? frozenV : baselineV + motorDirection * rampDeltaV;
        requested = commandV / hubVoltage;
        if (!ExtendIdentificationConfig.finite(requested)) { trip("OUTPUT_INVALID"); return; }
        // Full duty cannot be exceeded; a clipped ramp is no longer the commanded ramp.
        if (Math.abs(requested) > 1) { complete("FULL_DUTY_NO_RESULT"); return; }
        sent = requested;
    }
}
