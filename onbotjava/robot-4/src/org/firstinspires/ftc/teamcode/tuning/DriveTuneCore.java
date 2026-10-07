package org.firstinspires.ftc.teamcode.tuning;

import org.firstinspires.ftc.teamcode.DriveController;

/**
 * Robot 4 drive trials. Drive motors run RUN_USING_ENCODER, so a command is a wheel-speed target
 * and 0 makes the Hub velocity loop brake actively. Powers follow driveWithImuCorrection:
 * left = forward - turn, right = forward + turn.
 */
public final class DriveTuneCore {
    public static final String[] TRIALS = {"STRAIGHT", "TURN", "HEADING"};
    /** MATCH reproduces DriveController neutral sticks; RAMP lowers the command at brake_per_s. */
    public static final String[] BRAKES = {"MATCH", "RAMP"};
    static final long LOOP_LIMIT_NS = 100_000_000L;
    static final double STOP_TICKS_PER_S = 30;
    static final long STOP_HOLD_NS = 200_000_000L;
    static final double RUNAWAY_DEG = 60, WRONG_WAY_DEG = 30;

    private final DriveTuneConfig c;
    public String state, fault = "", phase = "", event = "", stopReason = "";
    public int trial, level, direction = 1, brake, trialId;
    public double command, left, right, travelM, turnedDeg, trialMs, headingTarget = Double.NaN;
    private long startNs, lastNs, brakeStartNs, stillSinceNs;
    private double startLeft, startRight, prevHeading = Double.NaN;

    public DriveTuneCore(DriveTuneConfig config) {
        c = config;
        state = config.motionEnabled ? "IDLE" : "MONITOR";
    }
    public boolean running() { return state.equals("RUNNING"); }
    public boolean wantsImu() { return running() && trial == 2; }
    public double power() { return Math.min(c.powers[Math.min(level, c.powers.length - 1)], c.maxPower); }
    public double step() { return c.headingSteps[Math.min(level, c.headingSteps.length - 1)]; }
    public String brakeText() {
        return brake == 0 ? "MATCH" : c.brakePerSec <= 0 ? "ZERO" : "RAMP " + c.brakePerSec + "/s";
    }

    /** Selection is frozen while a trial runs. */
    public void select(int trialStep, int levelStep, boolean flipDirection, boolean flipBrake) {
        if (running()) return;
        trial = Math.floorMod(trial + trialStep, TRIALS.length);
        int size = trial == 2 ? c.headingSteps.length : c.powers.length;
        level = Math.max(0, Math.min(size - 1, level + levelStep));
        if (flipDirection) direction = -direction;
        if (flipBrake) brake = 1 - brake;
    }

    public void update(long now, boolean deadman, boolean start, boolean writerHealthy,
                       double leftTicks, double rightTicks, double leftTps, double rightTps,
                       double heading, double imuOut) {
        event = "";
        long dtNs = lastNs == 0 ? 0 : now - lastNs;
        lastNs = now;
        boolean headingOk = DriveTuneConfig.finite(heading);
        if (headingOk && DriveTuneConfig.finite(prevHeading)) turnedDeg += wrap(heading - prevHeading);
        prevHeading = headingOk ? heading : Double.NaN;
        if (!c.motionEnabled) { output(0); return; }
        if (!writerHealthy) trip("LOGGER_FAILED");
        if (running() && dtNs > LOOP_LIMIT_NS) trip("LOOP_DELAY");
        if (!fault.isEmpty()) { output(0); return; }
        if (!running()) {
            output(0);
            if (deadman && start) begin(now, leftTicks, rightTicks, heading);
            return;
        }
        if (!deadman) { finish("DEADMAN_RELEASED"); return; }
        if (trial != 0 && !headingOk) { trip("IMU_INVALID"); return; }
        trialMs = (now - startNs) / 1e6;
        travelM = ((leftTicks - startLeft) + (rightTicks - startRight)) / 2 * c.metresPerTick();
        if (trial == 2) heading(imuOut);
        else drive(now, dtNs / 1e9, leftTps, rightTps);
    }

    private void begin(long now, double leftTicks, double rightTicks, double heading) {
        if (trial != 0 && !DriveTuneConfig.finite(heading)) { event = "START_REJECTED_IMU"; return; }
        trialId++;
        state = "RUNNING"; stopReason = ""; event = "START";
        phase = trial == 2 ? "HOLD" : "ACCEL";
        startNs = now; startLeft = leftTicks; startRight = rightTicks;
        turnedDeg = 0; travelM = 0; trialMs = 0; command = 0; stillSinceNs = 0;
        headingTarget = trial == 2 ? wrap(heading + direction * step()) : Double.NaN;
        output(0);
    }

    private void drive(long now, double dt, double leftTps, double rightTps) {
        if (!phase.equals("BRAKE")) {
            String limit = trialMs >= c.runMs ? "TIME"
                : trial == 0 && Math.abs(travelM) >= c.maxTravelM ? "TRAVEL"
                : trial == 1 && Math.abs(turnedDeg) >= c.maxTurnDeg ? "ANGLE" : "";
            if (limit.isEmpty()) {
                double target = direction * power();
                command = approach(command, target, c.accelPerSec, dt);
                String next = command == target ? "CRUISE" : "ACCEL";
                if (!next.equals(phase)) { phase = next; event = next; }
            } else {
                phase = "BRAKE"; event = "BRAKE_" + limit;
                brakeStartNs = now; stillSinceNs = 0;
            }
        }
        if (phase.equals("BRAKE")) {
            command = brake == 0 ? matchBrake(command) : approach(command, 0, c.brakePerSec, dt);
            boolean still = command == 0
                && Math.max(Math.abs(leftTps), Math.abs(rightTps)) < STOP_TICKS_PER_S;
            if (!still) stillSinceNs = 0;
            else if (stillSinceNs == 0) stillSinceNs = now;
            if (stillSinceNs != 0 && now - stillSinceNs >= STOP_HOLD_NS) { finish("STOPPED"); return; }
            if ((now - brakeStartNs) / 1e6 >= c.brakeTimeoutMs) { finish("BRAKE_TIMEOUT"); return; }
        }
        output(command);
    }

    private void heading(double imuOut) {
        double along = direction * turnedDeg;
        if (along > step() + RUNAWAY_DEG || along < -WRONG_WAY_DEG) { trip("HEADING_RUNAWAY"); return; }
        if (trialMs >= c.headingMs) { finish("TIME"); return; }
        command = DriveTuneConfig.finite(imuOut) ? imuOut : 0;
        output(command);
    }

    /** DriveController.driveManual with both sticks neutral, applied to the last command. */
    static double matchBrake(double last) {
        return Math.abs(last) > DriveController.STOP_THRESHOLD ? -last * DriveController.BRAKE_STRENGTH : 0;
    }
    static double approach(double from, double to, double perSec, double dt) {
        if (perSec <= 0) return to;
        double step = perSec * dt, diff = to - from;
        return Math.abs(diff) <= step ? to : from + Math.signum(diff) * step;
    }
    private void output(double u) {
        double lim = DriveTuneConfig.finite(c.maxPower) ? c.maxPower : 0;
        double v = Math.max(-lim, Math.min(lim, u));
        if (trial == 0) { left = v; right = v; }
        else { left = -v; right = v; }
    }
    private void finish(String reason) {
        state = "IDLE"; phase = "DONE"; event = "END"; stopReason = reason; command = 0;
        output(0);
    }
    public void trip(String reason) {
        if (fault.isEmpty()) fault = reason;
        if (!state.equals("MONITOR")) state = "FAULT";
        stopReason = fault; command = 0;
        output(0);
    }
    static double wrap(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}
