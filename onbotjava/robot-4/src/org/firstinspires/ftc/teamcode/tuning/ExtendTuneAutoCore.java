package org.firstinspires.ftc.teamcode.tuning;

/**
 * Operator-held automatic schedule for Extend_TuneAuto_R4. Same control law and guards as
 * ExtendTuneCore (FF + PID, 10 deg/s target slew, 45 deg/s overspeed, 500 ms no-motion),
 * but the goal follows a schedule: for each station, move -> hold -> +step -> back -> -step -> back.
 * The integral is cleared at every goal change, as ExtendController.setTarget() does.
 */
public final class ExtendTuneAutoCore {
    final ExtendTuneConfig config;
    public String state = "MONITOR", fault = "", segment = "";
    public double angle, velocity, target, goal, sent, requested, ff, p, i, d, dt;
    public int station = -1;
    private double previousAngle, previousError, integral, stallAngle;
    private long previousNs, segmentNs, stallNs;
    private int step;
    private boolean initialized;

    public ExtendTuneAutoCore(ExtendTuneConfig config) { this.config = config; }
    public void trip(String reason) { if (fault.isEmpty()) fault = reason; state = "FAULT"; zero(); }
    private void zero() { sent = requested = ff = p = i = d = integral = 0; }
    /** Deadman release pauses; RB + A resumes the current station from its move segment. */
    public void release() { if (state.equals("ACTIVE")) state = "PAUSED"; zero(); }

    /** Segment names and goal offsets per station; MOVE ends on arrival, others on time. */
    private static final String[] SEGMENTS = {"MOVE", "HOLD", "STEP_UP", "BACK_DOWN", "STEP_DOWN", "BACK_UP"};
    private static final double[] OFFSETS = {0, 0, 1, 0, -1, 0};

    private double segmentSeconds(int k) {
        return k == 0 ? config.autoArriveS : k == 1 ? config.autoHoldS : config.autoStepS;
    }

    private void enter(int stationIndex, int k, long now) {
        station = stationIndex; step = k; segment = SEGMENTS[k]; segmentNs = now;
        goal = config.autoStations[station] + OFFSETS[k] * config.autoStepDeg;
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
        if (!config.motionEnabled || !config.autoValid() || !deadman) { release(); return; }
        if (angle <= config.minAngle || angle >= config.maxAngle) { trip("ANGLE_LIMIT"); return; }
        if (startEdge && !state.equals("ACTIVE")) {
            // Start (or resume) from where the arm is, then slew to the station.
            target = angle;
            enter(Math.max(0, station), 0, now);
            state = "ACTIVE";
        }
        if (!state.equals("ACTIVE")) { zero(); return; }
        if (Math.abs(velocity) > 45) { trip("OVERSPEED"); return; }
        double inSegment = (now - segmentNs) / 1e9;
        boolean arrived = Math.abs(target - goal) < 1e-9 && Math.abs(angle - goal) < 1.5;
        if (step == 0 ? arrived || inSegment >= segmentSeconds(0) : inSegment >= segmentSeconds(step)) {
            if (step + 1 < SEGMENTS.length) enter(station, step + 1, now);
            else if (station + 1 < config.autoStations.length) enter(station + 1, 0, now);
            else { state = "COMPLETE"; segment = "DONE"; zero(); return; }
        }
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
