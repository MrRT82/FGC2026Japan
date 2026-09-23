// ============================================================================
// 最終変更: 2026-09-23 15:18:10 JST (UTC+09:00)
// 変更内容: Blue計測専用パッケージへ隔離。通常Blue/Redの依存クラスとは共有しない。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode.timing;

import com.qualcomm.robotcore.hardware.DcMotorEx;

// 通常は回収、手動指令で逆転。射撃中のみ低速継続を検知して出力を抑える。
public class IntakeController {
    public DcMotorEx Intake;

    private CachedPower intakeOut;

    public double power = 1.0;

    // 射撃中の回収出力。FeedControllerの射撃出力とは独立。
    public double powerShoot = 0.5;

    public double stallBackoffPower = 0.15;

    public double stallVelThresh = 100.0;

    // 速度100ticks/s未満が400ms続くと出力を抑え、500ms後に再試行する。
    public long stallDetectMs = 400;

    public long stallRetryMs = 500;

    private long stallStartMs   = 0;
    private long backoffStartMs = 0;
    private boolean backingOff  = false;

    public boolean isStallBackoff() { return backingOff; }

    private boolean shootMode = false;

    private final SensorReads reads;

    public IntakeController(RobotHardware hr) {
        reads = hr.reads;
        Intake = hr.intakeMotor;
        intakeOut = new CachedPower(Intake, 0.02);
    }

    public void setShootMode(boolean on) {
        shootMode = on;
    }

    public double getSkipRate(){ return intakeOut == null ? 0.0 : intakeOut.getSkipRate(); }

    public double getPowerLevel() {
        return shootMode ? powerShoot : power;
    }

    // 射撃時の抑制は逆転ではなく低出力化。通常時の詰まり逆転はメインが担当。
    private double applyStallBackoff(double p) {
        if (!shootMode || stallBackoffPower <= 0.0) {
            stallStartMs = 0;
            backingOff = false;
            return p;
        }
        long now = System.currentTimeMillis();

        if (backingOff) {
            if (now - backoffStartMs >= stallRetryMs) {
                backingOff = false;
                stallStartMs = 0;
            } else {
                double lim = stallBackoffPower;
                return (p >= 0) ? Math.min(p, lim) : Math.max(p, -lim);
            }
        }

        double v = Math.abs(reads.velocity(Intake, SensorTiming.VELOCITY_INTAKE));
        if (v < stallVelThresh) {
            if (stallStartMs == 0) {
                stallStartMs = now;
            } else if (now - stallStartMs >= stallDetectMs) {
                backingOff = true;
                backoffStartMs = now;
                double lim = stallBackoffPower;
                return (p >= 0) ? Math.min(p, lim) : Math.max(p, -lim);
            }
        } else {
            stallStartMs = 0;
        }
        return p;
    }

    public void Intake_Forward() {
        intakeOut.set(applyStallBackoff(getPowerLevel()));
    }

    // 逆転には停滞抑制を適用しない。
    public void Intake_Reverse() {
        intakeOut.set(-getPowerLevel());
    }

    public void Intake_Stop(){
        intakeOut.set(0);
    }
}
