package org.firstinspires.ftc.teamcode.tuning;

import java.util.ArrayDeque;

/**
 * Open-loop stick/slip identification for robot 4. Hardware-free; the OpMode feeds samples.
 *
 * Ramp event: hold still -> slow effort ramp until breakaway -> slide -> taper -> hold still.
 * Pulse event (every pulseEvery-th pair, once breakaway is known): hold still -> effort step
 * clearly above breakaway for a short time -> rest -> hold still. Pulses are the validation
 * replays: their onset does not depend on where static friction lies within its bracket.
 * Effort is commanded in volts (duty = V / motor-Hub voltage) so the logged effort
 * duty*voltage, which the analysis uses, ramps monotonically. Up/down events alternate;
 * long/short steps walk the arm across the configured range. No closed-loop control.
 */
public final class ExtendSysIdCore {
    public static final String EVENT_HEADER = "event,status,direction,scale,distance_goal_deg,rest_V,origin_deg,"
        + "pre_start_ms,ramp_start_ms,ramp_start_net_V,onboard_last_still_ms,onboard_first_motion_ms,"
        + "breakaway_V,breakaway_net_V,slide_V,slide_end_ms,slide_end_reason,stop_ms,end_ms,"
        + "peak_displacement_deg,max_speed_dps,kind";
    private static final int WINDOW = 32;
    private static final double STILL_WINDOW_MS = 300, MOTOR_HUB_MAX_AGE_MS = 120;
    /** Travel limit as a multiple of the step: the slide uses half, the taper may reach 1.5x. */
    private static final double OVERSHOOT = 1.5;
    final ExtendSysIdConfig config;
    public String state = "MONITOR", fault = "", phase = "IDLE", endReason = "";
    public double angle, velocity, dt, commandV, restV, requested, sent, motorHubV = Double.NaN;
    public boolean saturated;
    public int event = -1, direction;
    public double scale, distanceGoal, origin = Double.NaN, goal = Double.NaN;
    /** Finished event rows (EVENT_HEADER columns), consumed by the OpMode. */
    public final ArrayDeque<String> finishedEvents = new ArrayDeque<>();

    private final double[] windowT = new double[WINDOW], windowA = new double[WINDOW];
    private int windowCount, windowHead;
    private double previousAngle, previousMs, phaseStartMs;
    private boolean initialized, walkUp = true, eventOpen, taperSlow;
    /** Absolute breakaway effort per direction: [0] negative, [1] positive. */
    private final double[] lastBreakawayV = {Double.NaN, Double.NaN};
    /** Learned net effort where the slow taper stops the arm, per direction. */
    private final double[] stopNet = {Double.NaN, Double.NaN};
    private double slowSinceMs;
    private double preStartMs, preStartAngle, rampStartMs, rampStartNet, lastStillMs, firstMotionMs;
    private double breakawayV, breakawayNet, slideV, slideEndMs, stopMs, peakDisplacement, maxSpeed, taperNet;
    private String slideEndReason = "";
    /** "ramp" or "pulse". */
    public String kind = "";

    public ExtendSysIdCore(ExtendSysIdConfig config) { this.config = config; restV = config.initialRestV; }

    public void trip(String reason) {
        if (fault.isEmpty()) fault = reason;
        closeEvent("fault_" + fault);
        state = "FAULT"; zero();
    }
    /** Deadman released: abort the current event; RB + A resumes with a new event. */
    public void release() {
        if (state.equals("ACTIVE")) { closeEvent("aborted_release"); state = "PAUSED"; }
        zero();
    }
    private void finish(String reason) {
        closeEvent("aborted_" + reason);
        endReason = reason; state = "COMPLETE"; phase = "IDLE"; zero();
    }
    private void zero() { commandV = requested = sent = 0; saturated = false; }

    public void update(double observed, double tMs, boolean deadman, boolean startEdge,
                       double hubVolts, double hubAgeMs, String electricalFault, boolean writerOk) {
        dt = initialized ? (tMs - previousMs) / 1000 : .020;
        if (!ExtendTuneConfig.finite(observed)) {
            trip("ANGLE_INVALID"); return;
        }
        angle = observed;
        if (!initialized) previousAngle = angle;
        if (initialized && (dt < .002 || dt > .100)) {
            if (state.equals("ACTIVE")) trip("LOOP_DELAY");
            velocity = 0;
        } else if (initialized) velocity += .35 * ((angle - previousAngle) / dt - velocity);
        previousAngle = angle; previousMs = tMs; initialized = true;
        windowT[windowHead] = tMs; windowA[windowHead] = angle;
        windowHead = (windowHead + 1) % WINDOW; windowCount = Math.min(WINDOW, windowCount + 1);
        motorHubV = hubVolts;
        if (!writerOk) trip("LOGGER_FAILED");
        if (!electricalFault.isEmpty()) {
            if (state.equals("ACTIVE")) trip(electricalFault);
            zero(); return;
        }
        if (!fault.isEmpty() || state.equals("COMPLETE")) { zero(); return; }
        if (!config.motionEnabled || !config.valid() || !deadman) { release(); return; }
        // duty = effort_V / motor-Hub V needs a fresh reading; a measurement check, not a voltage limit.
        boolean voltsOk = ExtendTuneConfig.finite(hubVolts) && hubVolts > 0
            && hubAgeMs >= 0 && hubAgeMs <= MOTOR_HUB_MAX_AGE_MS;
        if (startEdge && !state.equals("ACTIVE")) {
            if (!voltsOk) { trip("MOTOR_HUB_VOLTAGE_STALE"); return; }
            state = "ACTIVE";
            nextEvent(tMs);
        }
        if (!state.equals("ACTIVE")) { zero(); return; }
        if (!voltsOk) { trip("MOTOR_HUB_VOLTAGE_STALE"); return; }
        step(tMs);
        if (!state.equals("ACTIVE")) return;
        requested = commandV / hubVolts;
        saturated = Math.abs(requested) > 1;
        // A ramp clipped at full duty is no longer the commanded slow ramp: stop instead of logging it.
        if (saturated && phase.equals("RAMP")) { trip("NO_BREAKAWAY"); return; }
        if (!ExtendTuneConfig.finite(requested)) { trip("OUTPUT_INVALID"); return; }
        sent = Math.max(-1, Math.min(1, requested));
    }

    private void nextEvent(double tMs) {
        for (int attempt = 0; attempt < 2; attempt++) {
            event++;
            if (event >= config.maxEvents) { finish("EVENTS_DONE"); return; }
            direction = event % 2 == 0 ? 1 : -1;
            double roomUp = (config.maxAngle - config.guardDeg - angle) / OVERSHOOT;
            double roomDown = (angle - (config.minAngle + config.guardDeg)) / OVERSHOOT;
            if (direction > 0 && walkUp && roomUp < config.longStepDeg) walkUp = false;
            if (direction < 0 && !walkUp && roomDown < config.longStepDeg) walkUp = true;
            boolean longStep = (direction > 0) == walkUp;
            distanceGoal = Math.min(longStep ? config.longStepDeg : config.shortStepDeg,
                                    direction > 0 ? roomUp : roomDown);
            if (distanceGoal >= 1) break;
            finishedEvents.add(event + ",skipped_no_room," + direction + ",,,,,,,,,,,,,,,,,,,");
            if (attempt == 1) { trip("NO_ROOM"); return; }
        }
        scale = config.slideScales[(event / 2) % config.slideScales.length];
        boolean pulseTurn = config.pulseEvery > 0 && (event / 2) % config.pulseEvery == config.pulseEvery - 1;
        kind = pulseTurn && !Double.isNaN(lastBreakawayV[index(direction)]) ? "pulse" : "ramp";
        if (kind.equals("pulse")) scale = config.pulseScale;
        preStartMs = tMs; preStartAngle = angle;
        rampStartMs = rampStartNet = lastStillMs = firstMotionMs = Double.NaN;
        breakawayV = breakawayNet = slideV = slideEndMs = stopMs = Double.NaN;
        peakDisplacement = maxSpeed = 0; slideEndReason = "";
        origin = goal = Double.NaN;
        eventOpen = true;
        enter("PRE_HOLD", tMs);
    }

    private void enter(String next, double tMs) { phase = next; phaseStartMs = tMs; }

    private void step(double tMs) {
        double inPhase = tMs - phaseStartMs;
        double displacement = Double.isNaN(origin) ? 0 : direction * (angle - origin);
        if (!phase.equals("PRE_HOLD") && !phase.equals("RAMP")) {
            peakDisplacement = Math.max(peakDisplacement, displacement);
            maxSpeed = Math.max(maxSpeed, Math.abs(velocity));
        }
        switch (phase) {
            case "PRE_HOLD":
                commandV = restV;
                // The rest effort must hold the arm; a wrong estimate (or initial_rest_V) lets it drift.
                if (Math.abs(angle - preStartAngle) > 2 * config.motionDetectDeg) { trip("DRIFT_AT_REST"); return; }
                if (inPhase >= config.preHoldS * 1000 && stillFor(tMs)) {
                    origin = windowMean(tMs); goal = origin + direction * distanceGoal;
                    double last = lastBreakawayV[index(direction)];
                    rampStartMs = lastStillMs = tMs;
                    if (kind.equals("pulse")) {
                        slideV = restV + direction * config.pulseScale * Math.max(0, direction * (last - restV));
                        rampStartNet = direction * (slideV - restV);
                        commandV = slideV;
                        enter("PULSE", tMs);
                        break;
                    }
                    rampStartNet = Double.isNaN(last) ? 0
                        : Math.max(0, direction * (last - restV) - config.rampBackoffV);
                    enter("RAMP", tMs);
                    commandV = restV + direction * rampStartNet;
                } else if (inPhase > config.preHoldS * 1000 + 3000) trip("NOT_STILL");
                break;
            case "PULSE": {
                commandV = slideV;
                if (displacement <= -config.motionDetectDeg) { trip("UNEXPECTED_MOTION"); return; }
                if (Double.isNaN(firstMotionMs) && displacement >= config.motionDetectDeg) firstMotionMs = tMs;
                String end = "";
                if (inPhase >= config.pulseS * 1000) end = "DURATION";
                else if (displacement >= OVERSHOOT * distanceGoal) end = "DISTANCE";
                else if (Math.abs(velocity) > config.slideMaxDps) end = "SPEED";
                else if (nearLimit()) end = "GUARD";
                if (!end.isEmpty()) { slideEndReason = end; slideEndMs = tMs; commandV = restV; enter("POST_HOLD", tMs); }
                break;
            }
            case "RAMP":
                if (displacement <= -config.motionDetectDeg) {
                    trip(event == 0 ? "WRONG_DIRECTION_OR_FALLING" : "UNEXPECTED_MOTION"); return;
                }
                if (displacement <= config.stillBandDeg / 2) lastStillMs = tMs;
                if (displacement >= config.motionDetectDeg) {
                    // commandV still holds the effort that produced the motion.
                    firstMotionMs = tMs; breakawayV = commandV;
                    breakawayNet = direction * (commandV - restV);
                    lastBreakawayV[index(direction)] = breakawayV;
                    slideV = restV + direction * scale * breakawayNet;
                    commandV = slideV;
                    enter("SLIDE", tMs);
                    break;
                }
                if (inPhase > config.rampMaxS * 1000) { trip("NO_BREAKAWAY"); return; }
                commandV = restV + direction * (rampStartNet + config.rampVPerS * inPhase / 1000);
                break;
            case "SLIDE": {
                commandV = slideV;
                String end = "";
                // Half the travel is left for the taper's deceleration and slow final approach.
                if (displacement >= distanceGoal / 2) end = "DISTANCE";
                else if (Math.abs(velocity) > config.slideMaxDps) end = "SPEED";
                else if (nearLimit()) end = "GUARD";
                else if (inPhase > config.slideMaxS * 1000) end = "TIMEOUT";
                else if (inPhase > STILL_WINDOW_MS && stillFor(tMs)) end = "STALL";
                if (!end.isEmpty()) {
                    slideEndReason = end; slideEndMs = tMs;
                    taperNet = direction * (slideV - restV); taperSlow = false;
                    if (end.equals("GUARD") || end.equals("STALL")) { commandV = restV; enter("POST_HOLD", tMs); }
                    else enter("TAPER", tMs);
                }
                break;
            }
            case "TAPER": {
                // Fast decrease while quick, then (latched) slow through dynamic friction: this
                // records slow sliding and ends with the arm stopping near the Coulomb level.
                if (!taperSlow) {
                    double learned = stopNet[index(direction)];
                    taperSlow = Double.isNaN(learned)
                        ? Math.abs(velocity) <= config.taperSlowBelowDps || taperNet <= config.taperSlowFraction * breakawayNet
                        : taperNet <= learned + config.taperSlowMarginV;
                    if (taperSlow) slowSinceMs = tMs;
                }
                taperNet -= (taperSlow ? config.taperSlowVPerS : config.taperVPerS) * dt;
                boolean runaway = inPhase > 200 && Math.abs(velocity) > config.slideMaxDps;
                boolean stopped = inPhase > STILL_WINDOW_MS && stillFor(tMs);
                if (stopped && taperSlow) learnStopNet(tMs);
                if (taperNet <= 0 || nearLimit() || displacement >= OVERSHOOT * distanceGoal || runaway || stopped) {
                    commandV = restV; enter("POST_HOLD", tMs);
                } else commandV = restV + direction * taperNet;
                break;
            }
            case "POST_HOLD":
                commandV = restV;
                // Forward coasting is larger near vertical (little gravity to decelerate).
                // Backward drift means the rest effort fails.
                if (displacement < -2 * config.motionDetectDeg
                        || displacement > OVERSHOOT * distanceGoal + config.guardDeg) { trip("DRIFT_AT_REST"); return; }
                if (Double.isNaN(firstMotionMs) && displacement >= config.motionDetectDeg) firstMotionMs = tMs;
                if (Double.isNaN(stopMs) && stillFor(tMs)) stopMs = tMs;
                if (!Double.isNaN(stopMs) && tMs - stopMs >= config.postHoldS * 1000) {
                    closeEvent("complete"); updateRest(); nextEvent(tMs);
                } else if (inPhase > config.postHoldS * 1000 + 4000) trip("NOT_STOPPED");
                break;
            default:
                trip("PHASE_INVALID");
        }
    }

    /**
     * Net effort at which the slow taper stopped the arm (~dynamic friction), so the next taper in
     * this direction starts its slow part just above it. The still window lags the stop by 300 ms.
     * If the arm stopped right after the slow part began, it started too low: search upward.
     */
    private void learnStopNet(double tMs) {
        int k = index(direction);
        double stoppedAt = taperNet + config.taperSlowVPerS * STILL_WINDOW_MS / 1000;
        if (tMs - slowSinceMs >= STILL_WINDOW_MS + 200) stopNet[k] = stoppedAt;
        else stopNet[k] = (Double.isNaN(stopNet[k]) ? stoppedAt : stopNet[k]) + .1;
        stopNet[k] = Math.min(stopNet[k], breakawayNet);
    }

    /** Midpoint of the latest up/down breakaway efforts: an estimate inside the stiction band. */
    private void updateRest() {
        if (!Double.isNaN(lastBreakawayV[0]) && !Double.isNaN(lastBreakawayV[1]))
            restV = (lastBreakawayV[0] + lastBreakawayV[1]) / 2;
    }

    private boolean nearLimit() {
        double hard = config.guardDeg / 2;
        return direction > 0 ? angle >= config.maxAngle - hard : angle <= config.minAngle + hard;
    }

    private static int index(int direction) { return direction > 0 ? 1 : 0; }

    /** Angle range over the last 300 ms is within the still band. */
    private boolean stillFor(double tMs) {
        double low = Double.POSITIVE_INFINITY, high = Double.NEGATIVE_INFINITY, oldest = tMs;
        int n = 0;
        for (int k = 0; k < windowCount; k++) {
            int j = (windowHead - 1 - k + WINDOW) % WINDOW;
            if (tMs - windowT[j] > STILL_WINDOW_MS) break;
            low = Math.min(low, windowA[j]); high = Math.max(high, windowA[j]);
            oldest = windowT[j]; n++;
        }
        return n >= 5 && tMs - oldest >= STILL_WINDOW_MS - 40 && high - low <= config.stillBandDeg;
    }

    private double windowMean(double tMs) {
        double sum = 0; int n = 0;
        for (int k = 0; k < windowCount; k++) {
            int j = (windowHead - 1 - k + WINDOW) % WINDOW;
            if (tMs - windowT[j] > STILL_WINDOW_MS) break;
            sum += windowA[j]; n++;
        }
        return n == 0 ? angle : sum / n;
    }

    private void closeEvent(String status) {
        if (!eventOpen) return;
        eventOpen = false;
        finishedEvents.add(event + "," + status + "," + direction + "," + scale + "," + distanceGoal
            + "," + restV + "," + origin + "," + preStartMs + "," + rampStartMs + "," + rampStartNet
            + "," + lastStillMs + "," + firstMotionMs + "," + breakawayV + "," + breakawayNet
            + "," + slideV + "," + slideEndMs + "," + slideEndReason + "," + stopMs + "," + previousMs
            + "," + peakDisplacement + "," + maxSpeed + "," + kind);
    }
}
