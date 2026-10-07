// ============================================================================
// 最終変更: 2026-10-07 JST (UTC+09:00)
// 変更内容: 足回りギヤ25:1→15:1(速度1.806倍)に合わせ、kpHold/kdHoldを旧値÷1.806へ換算。実機未検証。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode;

// 通常運転の方位補助。角度deg、角速度deg/s、dt秒。
public class DriveAssist {

    public enum Mode { OFF, SETTLING, HOLD, RATE }

    public double turnDeadzone = 0.05;

    public double fwdDeadzone = 0.05;

    public double captureRateDps = 15.0;

    public long settleTimeoutMs = 400;

    // ギヤ25:1→15:1で出力1あたりの旋回速度が1.806倍。旧値kpHold 0.015、kdHold 0.0020を÷1.806。
    public double kpHold = 0.00831;

    public double kdHold = 0.00111;

    // 据え置きのため補正の最大旋回速度は旧ギヤの約1.8倍。旧ギヤと同じにするなら0.194。
    public double holdMaxOut = 0.35;

    public double holdDeadbandDeg = 0.5;

    public double maxRateDps = 180.0;

    // 0では旋回中の角速度フィードバックは無効で、手動FFのみ使う。
    public double kpRate = 0.0;

    public double rateTrimMax = 0.25;

    public double rateFilterAlpha = 0.60;

    public double rateSanityMax = 600.0;

    private Mode mode = Mode.OFF;
    private double targetHeading = 0.0;
    private double rate = 0.0;
    private double prevHeading = 0.0;
    private boolean hasPrev = false;
    private double lastOut = 0.0;
    private double lastError = 0.0;
    private int spikes = 0;
    private long settleStartMs = 0;

    // 自動旋回中も呼び、手動復帰に備えて角速度の履歴を連続させる。
    public void observe(double heading, double dt) {
        updateRate(heading, dt);
    }

    public void standby() {
        mode = Mode.OFF;
        lastOut = 0.0;
        lastError = 0.0;
    }

    public void reset() {
        mode = Mode.OFF;
        hasPrev = false;
        rate = 0.0;
        lastOut = 0.0;
        lastError = 0.0;
    }

    // 旋回入力→RATE、旋回終了→SETTLING、収束または時間切れ→HOLD。両入力中立はOFF。
    public double update(double turnStick, double forwardStick,
                         double ffTurnOut, double heading, double dt) {

        updateRate(heading, dt);

        boolean turning = Math.abs(turnStick) > turnDeadzone;
        boolean moving  = Math.abs(forwardStick) > fwdDeadzone;

        if (!turning && !moving) {
            mode = Mode.OFF;
            lastOut = 0.0;
            lastError = 0.0;
            return 0.0;
        }

        if (turning) {
            mode = Mode.RATE;
            lastOut = rateOutput(ffTurnOut, turnStick * maxRateDps);
            return lastOut;
        }

        if (mode == Mode.RATE || mode == Mode.OFF) {

            mode = Mode.SETTLING;
            settleStartMs = System.currentTimeMillis();
        }

        if (mode == Mode.SETTLING) {
            boolean settled = Math.abs(rate) <= captureRateDps;
            boolean timedOut = (System.currentTimeMillis() - settleStartMs) >= settleTimeoutMs;
            if (settled || timedOut) {
                targetHeading = heading;
                mode = Mode.HOLD;
            } else {

                lastOut = rateOutput(0.0, 0.0);
                return lastOut;
            }
        }

        double error = normalizeError(targetHeading - heading);
        lastError = error;
        if (Math.abs(error) <= holdDeadbandDeg) {

            lastOut = clamp(-kdHold * rate, -holdMaxOut, holdMaxOut);
            return lastOut;
        }
        lastOut = clamp(kpHold * error - kdHold * rate, -holdMaxOut, holdMaxOut);
        return lastOut;
    }

    private double rateOutput(double ffTurnOut, double targetRate) {
        double trim = clamp(kpRate * (targetRate - rate), -rateTrimMax, rateTrimMax);
        lastError = targetRate - rate;
        return clamp(ffTurnOut + trim, -1.0, 1.0);
    }

    // 角度の±180deg跨ぎを補正し、異常な差分を除いてローパス処理する。
    private void updateRate(double heading, double dt) {
        if (!hasPrev || dt <= 1e-4) {
            prevHeading = heading;
            hasPrev = true;
            rate = 0.0;
            return;
        }
        double raw = normalizeError(heading - prevHeading) / dt;
        prevHeading = heading;
        if (Math.abs(raw) <= rateSanityMax) {
            rate += rateFilterAlpha * (raw - rate);
        } else {
            spikes++;
        }
    }

    private static double normalizeError(double deg) {
        while (deg >  180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    private static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    public Mode   getMode()    { return mode; }

    public String getModeText() {
        switch (mode) {
            case HOLD:     return "方位保持";
            case RATE:     return "角速度制御";
            case SETTLING: return "停止待ち";
            default:       return "補助なし";
        }
    }

    public double getTargetHeading() { return targetHeading; }

    public double getRate()   { return rate; }

    public double getHeading() { return prevHeading; }

    public int getModeCode() {
        switch (mode) {
            case SETTLING: return 1;
            case HOLD:     return 2;
            case RATE:     return 3;
            default:       return 0;
        }
    }

    public double getError()  { return lastError; }

    public double getOutput() { return lastOut; }

    public int    getSpikes() { return spikes; }

    public boolean isActive() { return mode != Mode.OFF; }
}
