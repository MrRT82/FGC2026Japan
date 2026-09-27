package org.firstinspires.ftc.teamcode.tuning;

/** Bounded, operator-held trial. No automatic jam reversal or gain changes. */
public final class ExtendTuneCore {
    final ExtendTuneConfig config;
    public String state = "MONITOR", fault = "";
    public double angle, velocity, target, goal, sent, requested, ff, p, i, d, dt;
    private double previousAngle, previousError, integral, center, stallAngle;
    private long previousNs, startedNs, stallNs;
    private boolean initialized;
    public ExtendTuneCore(ExtendTuneConfig config) { this.config = config; }
    public void trip(String reason) { if (fault.isEmpty()) fault = reason; state = "FAULT"; zero(); }
    private void zero() { sent = requested = ff = p = i = d = integral = 0; }
    public void release() { if (fault.isEmpty()) state = "MONITOR"; zero(); }
    public void update(double observed, long now, boolean deadman, boolean startEdge,
                       int targetStep, String electricalFault, boolean writerOk) {
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
        if (!fault.isEmpty()) { zero(); return; }
        if (!config.motionEnabled || !config.valid() || !deadman) { release(); return; }
        if (angle <= config.minAngle || angle >= config.maxAngle) { trip("ANGLE_LIMIT"); return; }
        if (startEdge && !state.equals("ACTIVE")) {
            center = target = goal = angle; integral = previousError = 0;
            stallAngle = angle; stallNs = startedNs = now; state = "ACTIVE";
        }
        if (!state.equals("ACTIVE")) { zero(); return; }
        if (now - startedNs >= 8_000_000_000L) { state = "COMPLETE"; zero(); return; }
        if (Math.abs(velocity) > 45) { trip("OVERSPEED"); return; }
        double low = Math.max(config.minAngle + 1, center - 10);
        double high = Math.min(config.maxAngle - 1, center + 10);
        goal = Math.max(low, Math.min(high, goal + targetStep * 5));
        target += Math.max(-10 * dt, Math.min(10 * dt, goal - target));
        double error = target - angle;
        integral = Math.max(-.5, Math.min(.5, integral + error * dt));
        p = config.kp * error; i = config.ki * integral;
        d = config.kd * (error - previousError) / dt;
        previousError = error;
        double friction = velocity * Math.signum(error) > 15 ? 0
            : config.friction * Math.max(-1, Math.min(1, error / 2));
        ff = config.gravityCos * Math.cos(Math.toRadians(angle + config.gravityPhaseDeg))
            + config.gravityConstant + friction;
        requested = ff + p + i + d;
        if (!ExtendTuneConfig.finite(requested)) { trip("OUTPUT_INVALID"); return; }
        sent = Math.max(-config.maxPower, Math.min(config.maxPower, requested));
        if (Math.abs(angle - stallAngle) >= 1 || Math.abs(error) < 3 || Math.abs(sent) < .1) {
            stallAngle = angle; stallNs = now;
        } else if (now - stallNs > 500_000_000L) trip("NO_MOTION");
    }
}
