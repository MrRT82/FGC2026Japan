// ============================================================================
// 最終変更: 2026-09-23 14:48:47 JST (UTC+09:00)
// 変更内容: センサーAPI所要時間と制御周期の計測を追加。取得頻度・制御設定は維持。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

// 同軸の左右シューターを増分PID+FFで制御する。速度ticks/s、dt秒。
public class ShooterController {

    public double targetSpeed = 0.0;

    private double targetRamped = 0.0;

    public double kp = 0.001;

    public double ki = 0.0006;
    public double kd = 0.00005;

    public double kf = 0.000474;

    // 0では発射負荷の追加FFなし。有効化時は実測で調整する。
    public double loadFeedforward = 0.0;

    public double loadFeedforwardRampSec = 0.15;

    private boolean firing = false;
    private double loadFFApplied = 0.0;

    public void setFiring(boolean on) {
        firing = on;
    }

    public double getLoadFF() {
        return loadFFApplied;
    }

    // 低電圧時のみFFを増やし、倍率は最大1.35に制限する。
    public boolean compensateVoltage = true;
    private static final double NOMINAL_VOLTAGE = 12.0;

    private static final double COMP_MAX = 1.35;

    private VoltageReader voltageReader = null;

    public interface VoltageReader {
        double getVoltage();
    }

    public void setVoltageReader(VoltageReader r) {
        voltageReader = r;
    }

    // 目標速度の増加率。減速側にはtargetSlewDownPerSecを使う。
    public double targetSlewPerSec = 700.0;

    public double targetSlewDownPerSec = 500.0;

    public double outputAlpha = 0.5;

    // 左右の出力配分を調整する。片側故障は速度一致だけでは検出できない。
    public double balanceTrim = -0.018;

    public double balanceKi = 0.0;

    private static final double TRIM_LIMIT = 0.15;
    private double autoTrim = 0.0;

    public double lastCurL = 0.0, lastCurR = 0.0;

    private double prevError = 0.0;
    private double prevPrevError = 0.0;
    private double currentOutput = 0.0;
    private double outputFiltered = 0.0;

    public double lastCommanded = 0.0;

    public double lastError    = 0.0;
    public double lastIntegral = 0.0;
    public double lastFF       = 0.0;
    public double lastDt       = 0.0;
    public boolean lastRamping = false;

    public double lastSpeed = 0.0;

    public double feedStopSpeed  = 1100.0;

    public double feedStartSpeed = 1130.0;
    private boolean feedAllowed = false;

    // 送り停止閾値=目標×0.75、開始はさらに30ticks/s上。目標1200なら900/930。
    public double feedStopRatio = 900.0 / 1200.0;

    public double feedGateHysteresis = 30.0;

    private static final double FEED_GATE_MIN_STOP = 1.0;

    // 目標速度を変更したら呼び、送り許可のヒステリシスを追従させる。
    public void setFeedGateForTarget(double shootSpeed) {
        double stop = Math.abs(shootSpeed) * feedStopRatio;
        if (stop < FEED_GATE_MIN_STOP) stop = FEED_GATE_MIN_STOP;
        feedStopSpeed  = stop;
        feedStartSpeed = stop + feedGateHysteresis;
    }

    public boolean isFeedReady() {
        return feedStopSpeed <= 0.0 || feedAllowed;
    }

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double dtActual = 0.020;
    private static final double DT_MIN = 0.002;
    private static final double DT_MAX = 0.200;

    // falseは右側のみ駆動する設定。運転中に切替える停止処理は実装していない。
    public boolean useLeftMotor = true;

    public DcMotorEx leftShooter;
    public DcMotorEx rightShooter;

    private CachedPower leftOut, rightOut;

    private final SensorReads reads;

    public ShooterController(RobotHardware hr) {
        reads = hr.reads;
        leftShooter = hr.left_shooterMotor;
        rightShooter = hr.right_shooterMotor;
        leftOut  = new CachedPower(leftShooter,  0.002);
        rightOut = new CachedPower(rightShooter, 0.002);
    }

    // 最終目標を設定する。実際の制御目標はshooter_loopで徐々に追従する。
    public void setSpeed(double speed) {
        if (targetSpeed != speed) {
            targetSpeed = speed;
            prevError = 0.0;
            prevPrevError = 0.0;
        }
    }

    public double getRampProgress() {
        if (Math.abs(targetSpeed) < 1e-6) return 1.0;
        return Math.min(1.0, Math.abs(targetRamped / targetSpeed));
    }

    public double getSkipRate(){
        if (leftOut == null) return 0.0;
        return (leftOut.getSkipRate() + rightOut.getSkipRate()) / 2.0;
    }

    public double getTargetRamped() {
        return targetRamped;
    }

    public double getTrim() {
        return balanceTrim + autoTrim;
    }

    private double readCurrent(DcMotorEx m) {
        try { return reads.current(m, SensorTiming.OPTIONAL_CURRENT); } catch (Exception e) { return 0.0; }
    }

    // 速度取得→送り許可→目標ランプ→PID/FF→平滑化→左右出力の順。
    public void shooter_loop() {

        double measured = loopTimer.seconds();
        loopTimer.reset();
        if (measured < DT_MIN || measured > DT_MAX) {
            dtActual = 0.020;
        } else {
            dtActual = measured;
        }

        double current = useLeftMotor
                ? (reads.velocity(leftShooter, SensorTiming.VELOCITY_SHOOTER) + reads.velocity(rightShooter, SensorTiming.VELOCITY_SHOOTER)) / 2.0
                : reads.velocity(rightShooter, SensorTiming.VELOCITY_SHOOTER);
        lastSpeed = current;

        if (current <= feedStopSpeed) {
            feedAllowed = false;
        } else if (current >= feedStartSpeed) {
            feedAllowed = true;
        }

        double toGo = targetSpeed - targetRamped;
        boolean slowingDown = Math.abs(targetSpeed) < Math.abs(current);
        double rate = slowingDown ? targetSlewDownPerSec : targetSlewPerSec;
        if (rate <= 0) rate = targetSlewPerSec;
        double step = rate * dtActual;
        if (Math.abs(toGo) <= step) {
            targetRamped = targetSpeed;
        } else {
            targetRamped += (toGo > 0 ? step : -step);
        }

        double error = targetRamped - current;

        double deltaP = error - prevError;

        // ランプ中はIの増分を止める。P/Dは誤差差分から増分計算する。
        boolean ramping = Math.abs(targetSpeed - targetRamped) > 1e-6;
        double deltaI = ramping ? 0.0 : error * dtActual;
        double deltaD = (error - 2.0 * prevError + prevPrevError) / dtActual;

        double deltaOutput = (kp * deltaP) + (ki * deltaI) + (kd * deltaD);

        double lffTarget = (firing && targetRamped > 1.0) ? loadFeedforward : 0.0;
        if (loadFeedforwardRampSec <= 0.0) {
            loadFFApplied = lffTarget;
        } else {
            double lffStep = Math.abs(loadFeedforward) * dtActual / loadFeedforwardRampSec;
            if (loadFFApplied < lffTarget) {
                loadFFApplied = Math.min(lffTarget, loadFFApplied + lffStep);
            } else {
                loadFFApplied = Math.max(lffTarget, loadFFApplied - lffStep);
            }
        }

        double ff = targetRamped * kf;

        if (compensateVoltage && voltageReader != null) {
            double v = voltageReader.getVoltage();
            if (v > 6.0) {
                double comp = NOMINAL_VOLTAGE / v;
                if (comp > COMP_MAX) comp = COMP_MAX;
                if (comp < 1.0) comp = 1.0;
                ff *= comp;
            }
        }

        ff += loadFFApplied;

        currentOutput += deltaOutput;

        // FFを含めた出力の飽和範囲にPID累積値を収める。
        double lo = -1.0 - ff;
        double hi =  1.0 - ff;
        if (currentOutput < lo) currentOutput = lo;
        if (currentOutput > hi) currentOutput = hi;

        if (Math.abs(targetSpeed) < 1e-6 && Math.abs(targetRamped) < 1e-6) {
            currentOutput = 0.0;
        }

        lastError    = error;
        lastIntegral = currentOutput;
        lastFF       = ff;
        lastDt       = dtActual;
        lastRamping  = ramping;

        double finalOutput = currentOutput + ff;
        if (finalOutput < -1.0) finalOutput = -1.0;
        if (finalOutput >  1.0) finalOutput =  1.0;

        outputFiltered += outputAlpha * (finalOutput - outputFiltered);

        // 自動電流バランスは既定無効。有効化すると左右電流の追加取得が発生する。
        if (balanceKi > 0.0) {
            lastCurL = readCurrent(leftShooter);
            lastCurR = readCurrent(rightShooter);
            double diff = lastCurL - lastCurR;
            autoTrim += balanceKi * diff * dtActual;
            if (autoTrim >  TRIM_LIMIT) autoTrim =  TRIM_LIMIT;
            if (autoTrim < -TRIM_LIMIT) autoTrim = -TRIM_LIMIT;
        }
        double trim = balanceTrim + autoTrim;
        if (trim >  TRIM_LIMIT) trim =  TRIM_LIMIT;
        if (trim < -TRIM_LIMIT) trim = -TRIM_LIMIT;

        if (useLeftMotor) {
            lastCommanded = outputFiltered * (1.0 - trim);
            leftOut.set(lastCommanded);
            rightOut.set(outputFiltered * (1.0 + trim));
        } else {
            lastCommanded = outputFiltered;
            rightOut.set(lastCommanded);
        }

        prevPrevError = prevError;
        prevError = error;
    }
}
