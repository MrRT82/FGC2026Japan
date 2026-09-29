package org.firstinspires.ftc.teamcode.tuning;

/** Synthetic inputs only: no hardware communication. */
public final class ExtendIdentificationTest {
    private static int checks;
    private static void check(boolean ok, String message) { checks++; if (!ok) throw new AssertionError(message); }
    private static ExtendIdentificationConfig config(String mode) {
        ExtendIdentificationConfig c = new ExtendIdentificationConfig();
        c.motionEnabled = true; c.minAngle = 1; c.maxAngle = 75; c.maxPower = .25;
        c.motorLimitA = 2; c.hubLimitA = 8; c.minVoltage = 11;
        c.trialMode = mode; c.datasetRole = "train"; c.balls = 0; c.motorHubIndex = 0;
        c.kp = c.ki = c.kd = c.friction = c.gravityCos = 0; c.gravityConstant = .1;
        return c;
    }
    private static void tick(ExtendTuneCore c, int n, double angle, boolean held, String fault, double volts) {
        c.update(angle, 1_000_000_000L + n * 20_000_000L, held, n == 0, 0, fault, true, volts);
    }
    public static void main(String[] args) {
        ExtendIdentificationConfig cfg = config("BREAKAWAY_POS");
        check(cfg.valid(), "ramp fixture valid");
        cfg.motorHubIndex = -1; check(!cfg.valid(), "ramp requires assigned Hub"); cfg.motorHubIndex = 0;
        cfg.balls = -1; check(!cfg.valid(), "load must be known"); cfg.balls = 0;
        cfg.datasetRole = "unspecified"; check(!cfg.valid(), "role must be recorded"); cfg.datasetRole = "train";
        cfg.rampRateVps = Double.NaN; check(!cfg.valid(), "invalid ramp rate"); cfg.rampRateVps = .4;
        cfg.rampMaxDeltaV = 4; check(cfg.valid(), "expanded ramp limit accepted");
        cfg.rampMaxDeltaV = 4.001; check(!cfg.valid(), "ramp above expanded limit rejected"); cfg.rampMaxDeltaV = 1;
        cfg.motorToAngleSign = -1; check(cfg.valid(), "reversed motor-angle sign accepted");
        cfg.motorToAngleSign = 0; check(!cfg.valid(), "zero motor-angle sign rejected");
        cfg.motorToAngleSign = 2; check(!cfg.valid(), "non-unit motor-angle sign rejected");
        cfg.motorToAngleSign = 1;
        ExtendTuneCore c = new ExtendTuneCore(cfg);
        tick(c, 0, 40, true, "", Double.NaN);
        check(c.sent == 0 && !c.state.equals("ACTIVE"), "no valid voltage no start");
        c = new ExtendTuneCore(cfg);
        for (int n = 0; n <= 85; n++) tick(c,n,40,true,"",12);
        check(c.phase.equals("RAMP"), "hold then frozen baseline then ramp");
        check(Math.abs(c.baselineV - 1.2) < 1e-9, "capture holding voltage");
        check(c.p == 0 && c.i == 0 && c.d == 0, "PID disabled while ramping");
        check(Math.abs(c.rampDeltaV - .08) < 1e-9, "0.4V/s ramp");
        check(c.lastStillNs > 0 && c.trialId == 1, "trial id and still timestamp");
        double voltageCommand = c.commandV;
        tick(c,86,40,true,"",11.5);
        check(Math.abs(c.sent * 11.5 - (voltageCommand + .008)) < 1e-9, "voltage normalization");
        c = new ExtendTuneCore(cfg);
        for (int n = 0; n <= 85; n++)
            tick(c, n, n % 2 == 0 ? 41.972727272727276 : 42.054545454545455, true, "", 12);
        check(c.phase.equals("RAMP"), "quantized stationary sensor reaches ramp");
        tick(c,87,40,false,"",12);
        check(c.sent == 0 && c.stopReason.equals("DEADMAN_RELEASE"), "ramp deadman release");
        tick(c,88,40,true,"",12); check(c.sent == 0, "no automatic restart");
        c = new ExtendTuneCore(cfg);
        for (int n=0;n<=100;n++) tick(c,n,40,true,"",12);
        tick(c,101,40.32,true,"",12); double frozen = c.commandV;
        check(c.firstMotionNs > 0, "onset candidate timestamp");
        for (int n=102;n<=107;n++) tick(c,n,40.32,true,"",12);
        check(c.phase.equals("POST_MOTION") && c.commandV == frozen, "sustained motion freezes input");
        for (int n=108;n<=120;n++) tick(c,n,40.32,true,"",12);
        check(c.sent == 0 && c.stopReason.equals("BREAKAWAY_RECORDED"), "record then zero");
        c = new ExtendTuneCore(cfg);
        for(int n=0;n<=350;n++) tick(c,n,40,true,"",12);
        check(c.sent == 0 && c.stopReason.equals("NO_BREAKAWAY_WITHIN_LIMIT"), "bounded static dwell");
        c = new ExtendTuneCore(config("BREAKAWAY_NEG"));
        for(int n=0;n<=100;n++) tick(c,n,40,true,"",12);
        check(c.commandV < c.baselineV, "negative ramp decreases signed effort");
        tick(c,101,40.32,true,"",12); check(c.fault.equals("WRONG_DIRECTION") && c.sent == 0, "wrong direction stop");
        cfg = config("BREAKAWAY_NEG"); cfg.motorToAngleSign = -1;
        c = new ExtendTuneCore(cfg);
        for(int n=0;n<=100;n++) tick(c,n,40,true,"",12);
        check(c.commandV > c.baselineV, "reversed sign maps negative angle to positive motor effort");
        tick(c,101,39.68,true,"",12);
        check(c.firstMotionNs > 0 && c.fault.isEmpty(), "negative angle motion accepted with reversed sign");
        cfg = config("BREAKAWAY_POS"); cfg.motorToAngleSign = -1;
        c = new ExtendTuneCore(cfg);
        for(int n=0;n<=100;n++) tick(c,n,40,true,"",12);
        check(c.commandV < c.baselineV, "reversed sign maps positive angle to negative motor effort");
        tick(c,101,40.32,true,"",12);
        check(c.firstMotionNs > 0 && c.fault.isEmpty(), "positive angle motion accepted with reversed sign");
        c = new ExtendTuneCore(cfg);
        for(int n=0;n<=55;n++) tick(c,n,40,true,"",12);
        tick(c,56,40.2,true,"",12); check(c.fault.equals("BASELINE_DRIFT"), "freeze drift abort");
        c = new ExtendTuneCore(cfg);
        for(int n=0;n<=100;n++) tick(c,n,40,true,"",12);
        tick(c,101,40,true,"MOTOR_CURRENT",12);
        check(c.fault.equals("MOTOR_CURRENT") && c.sent == 0, "ramp current guard retained");
        c = new ExtendTuneCore(cfg);
        for(int n=0;n<=150;n++) tick(c,n,40+n*.01,true,"",12);
        check(c.sent == 0 && !c.fault.isEmpty(), "cannot settle refuses ramp");
        cfg = config("STEP_NEG"); cfg.slewDps = 2;
        c = new ExtendTuneCore(cfg);
        for(int n=0;n<=49;n++) tick(c,n,40,true,"",12);
        check(c.target == 40, "step prehold");
        tick(c,50,40,true,"",12);
        check(c.goal == 35 && Math.abs(c.target-39.96)<1e-9 && c.event.equals("TARGET_STEP"), "scripted step and configured slew");
        c = new ExtendTuneCore(cfg); tick(c,0,3,true,"",12);
        check(c.fault.equals("TRIAL_RANGE") && c.sent == 0, "reject insufficient clearance before motion");
        cfg = config("BREAKAWAY_POS");
        c = new ExtendTuneCore(cfg);
        for(int n=0;n<=100;n++) tick(c,n,40,true,"",12);
        tick(c,101,40,true,"",Double.NaN);
        check(c.sent == 0 && c.fault.equals("RAMP_VOLTAGE_INVALID"), "voltage failure during ramp");
        c = new ExtendTuneCore(cfg);
        for(int n=0;n<=100;n++) tick(c,n,40,true,"",12);
        c.update(40,3_200_000_000L,true,false,0,"",true,12);
        check(c.sent == 0 && c.fault.equals("LOOP_DELAY"), "ramp loop delay stop");
        cfg.maxPower = .101;
        c = new ExtendTuneCore(cfg);
        for(int n=0;n<=100;n++) tick(c,n,40,true,"",12);
        check(c.sent == 0 && c.stopReason.equals("OUTPUT_LIMIT_NO_RESULT"), "ramp clips are not a valid threshold result");
        System.out.println("ExtendIdentificationTest passed: " + checks);
    }
}
