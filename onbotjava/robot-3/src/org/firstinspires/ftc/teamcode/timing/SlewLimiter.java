// ============================================================================
// 最終変更: 2026-09-23 15:18:10 JST (UTC+09:00)
// 変更内容: Blue計測専用パッケージへ隔離。通常Blue/Redの依存クラスとは共有しない。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode.timing;

// 入力値の毎秒変化量を制限する。ratePerSec<=0は無効、dtは秒。
public class SlewLimiter {

    public double ratePerSec;

    // 同符号で絶対値を減らす場合は即応する。
    public boolean limitOnlyIncreasing = true;

    private double value = 0.0;
    private boolean first = true;

    public SlewLimiter(double ratePerSec) {
        this.ratePerSec = ratePerSec;
    }

    // 初回は入力をそのまま返す。初回から制限したい場合は事前にreset(0)を呼ぶ。
    public double calculate(double target, double dt) {
        if (first) {
            first = false;
            value = target;
            return value;
        }
        if (ratePerSec <= 0) {
            value = target;
            return value;
        }
        if (dt <= 0 || dt > 0.5) dt = 0.02;

        if (limitOnlyIncreasing && Math.abs(target) <= Math.abs(value)
                && target * value >= 0) {
            value = target;
            return value;
        }

        double maxStep = ratePerSec * dt;
        double diff = target - value;
        if (diff >  maxStep) diff =  maxStep;
        if (diff < -maxStep) diff = -maxStep;
        value += diff;
        return value;
    }

    public void reset(double to) {
        value = to;
        first = false;
    }

    public double getValue() { return value; }
}
