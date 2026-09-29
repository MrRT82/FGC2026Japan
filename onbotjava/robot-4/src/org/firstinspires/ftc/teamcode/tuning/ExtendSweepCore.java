package org.firstinspires.ftc.teamcode.tuning;

/**
 * Operator-held gain sweep for Extend_TuneSweep_R4. Same control law as ExtendTuneCore
 * (FF + PID, slewed target, error-shaped friction) at match-level power/slew limits.
 * For each candidate (kp, kd): move to startDeg (unscored), then visit every goal and hold.
 * Any guard trip latches and ends the sweep; earlier candidates stay in the log.
 */
public final class ExtendSweepCore {
    final ExtendSweepConfig config;
    public String state = "MONITOR", fault = "", segment = "";
    public double angle, velocity, target, goal, sent, requested, ff, p, i, d, dt, kp, kd;
    public int candidate = -1, goalIndex = -1;
    private double previousAngle, previousError, integral, stallAngle;
    private long previousNs, segmentNs, stallNs;
    private boolean initialized;

    public ExtendSweepCore(ExtendSweepConfig config) { this.config = config; }
    public void trip(String reason) { if (fault.isEmpty()) fault = reason; state = "FAULT"; zero(); }
    private void zero() { sent = requested = ff = p = i = d = integral = 0; }
    /** Deadman release pauses; RB + A restarts the current candidate from its start move. */
    public void release() { if (state.equals("ACTIVE")) state = "PAUSED"; zero(); }

    private void enter(int c, int g, long now) {
        candidate = c; goalIndex = g; segmentNs = now;
        kp = config.kp(c); kd = config.kd(c);
        goal = g < 0 ? config.startDeg : config.goals[g];
        segmentTravel = Math.abs(goal - target);
        segment = g < 0 ? "START" : "GOAL_" + g;
        integral = previousError = 0;
        stallAngle = angle; stallNs = now;
    }

    public void update(double observed, long now, boolean deadman, boolean startEdge,
                       String electricalFault, boolean writerOk) {
        dt = initialized ? (now - previousNs) / 1e9 : .020;
        if (!ExtendTuneConfig.finite(observed) || observed < -5 || observed > 90) {
            trip("ANGLE_INVALID"); return;
        }
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
        if (!fault.isEmpty() || state.equals("COMPLETE")) { zero(); return; }
        if (!config.motionEnabled || !config.valid() || !deadman) { release(); return; }
        if (angle <= config.minAngle || angle >= config.maxAngle) { trip("ANGLE_LIMIT"); return; }
        if (startEdge && !state.equals("ACTIVE")) {
            target = angle;
            enter(Math.max(0, candidate), -1, now);
            state = "ACTIVE";
        }
        if (!state.equals("ACTIVE")) { zero(); return; }
        if (Math.abs(velocity) > config.overspeedDps) { trip("OVERSPEED"); return; }
        double inSegment = (now - segmentNs) / 1e9;
        boolean arrived = Math.abs(target - goal) < 1e-9 && Math.abs(angle - goal) < 1.5;
        // START ends on arrival (or timeout); goals always hold the full time so settling is recorded.
        boolean next = goalIndex < 0 ? (arrived && inSegment >= .5) || inSegment >= config.arriveS
                                     : inSegment >= config.holdS + segmentTravel / config.slewDps;
        if (next) {
            if (goalIndex + 1 < config.goals.length) enter(candidate, goalIndex + 1, now);
            else if (candidate + 1 < config.candidates()) enter(candidate + 1, -1, now);
            else { state = "COMPLETE"; segment = "DONE"; zero(); return; }
        }
        target += Math.max(-config.slewDps * dt, Math.min(config.slewDps * dt, goal - target));
        double error = target - angle;
        integral = Math.max(-.5, Math.min(.5, integral + error * dt));
        p = kp * error; i = config.ki * integral;
        d = kd * (error - previousError) / dt;
        previousError = error;
        double friction = velocity * Math.signum(error) > 15 ? 0
            : config.friction * Math.max(-1, Math.min(1, error / 2));
        ff = config.gravityCos * Math.cos(Math.toRadians(angle + config.gravityPhaseDeg))
            + config.gravityConstant + friction;
        requested = ff + p + i + d;
        if (!ExtendTuneConfig.finite(requested)) { trip("OUTPUT_INVALID"); return; }
        sent = Math.max(-config.maxPower, Math.min(config.maxPower, requested));
        // Same no-motion guard as ExtendTuneCore.
        if (Math.abs(angle - stallAngle) >= 1 || Math.abs(error) < 3 || Math.abs(sent) < .1) {
            stallAngle = angle; stallNs = now;
        } else if (now - stallNs > 500_000_000L) trip("NO_MOTION");
    }
    /** Goal distance from the target at segment start; the hold starts after the slew. */
    private double segmentTravel;
}
