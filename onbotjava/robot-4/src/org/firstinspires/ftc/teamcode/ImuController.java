// ============================================================================
// 最終変更: 2026-09-22 11:20:42 JST (UTC+09:00)
// 変更内容: 変更日時ヘッダーを追加。直前までの修正内容を保持。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// 目標方位のPD/PID制御と摩擦補償。角度deg、角速度deg/s、dt秒。
public class ImuController {

    private final IMU imu;

    public double imuTurnOutput = 0;

    public double kp = 0.040;
    public double ki = 0.000;
    public double kd = 0.0040;

    public double kS = 0.100;

    public double kV = 0.000;

    // falseでは目標角度を即時反映。trueでは角速度と目標先行量を制限する。
    public boolean useProfile = false;

    public double maxTurnRate = 180.0;

    public double maxLead = 25.0;

    public double tolerance = 3.0;

    public double finalTolerance = 1.0;

    public double ksFull = 6.0;

    public double ksRateCut = 20.0;

    public double iZone = 10.0;

    public double iMaxContrib = 0.15;

    public double maxTurnPower = 1.0;

    // 0で出力の増加率制限を無効化。減速側の応答は遅らせない。
    public double turnSlewPerSec = 0.0;
    private double turnSlewed = 0.0;

    private double applyTurnSlew(double out) {
        if (turnSlewPerSec <= 0.0) {
            turnSlewed = out;
            return out;
        }
        if (Math.abs(out) <= Math.abs(turnSlewed) || out * turnSlewed < 0.0) {
            turnSlewed = out;
            return out;
        }
        double step = turnSlewPerSec * dt;
        double diff = out - turnSlewed;
        if (diff >  step) diff =  step;
        if (diff < -step) diff = -step;
        turnSlewed += diff;
        return turnSlewed;
    }

    public double rateFilterAlpha = 0.60;

    public double rateSanityMax = 600.0;

    // 既定はヨー角差分を使用。ジャイロ直読みは追加通信と取付軸の検証が必要。
    public boolean useGyroRate = false;

    public boolean readBothRates = false;

    public double gyroSign = 1.0;

    private static final double STOP_THRESH  = 3.5;
    private static final double STOP_RATE    = 8.0;
    private static final long   STOP_TIME_MS = 150;

    private static final double DT_NOMINAL = 0.020;
    private static final double DT_MIN     = 0.001;
    private static final double DT_MAX     = 0.100;

    public  double targetHeading = 0.0;
    private double setpoint      = 0.0;
    private double setpointRate  = 0.0;
    private double integral      = 0.0;
    private double error         = 0.0;
    private double current       = 0.0;
    private double prevCurrent   = 0.0;
    private double rate          = 0.0;
    private double rateDiff      = 0.0;
    private double rateGyro      = 0.0;

    private double pitch = 0.0, roll = 0.0;

    private int rateSpikes = 0;
    private double gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
    private double dt            = DT_NOMINAL;
    private boolean isRunning    = false;
    private boolean firstLoop    = true;
    private long    inToleranceSince = 0;

    public double pTerm = 0, iTerm = 0, dTerm = 0, vTerm = 0, sTerm = 0;

    private final ElapsedTime loopTimer = new ElapsedTime();

    public ImuController(RobotHardware hw) {
        imu = hw.imu;
    }

    public ImuController(IMU imu) {
        this.imu = imu;
    }

    // START時に呼ぶ。現在の機体方向を方位ゼロにする。
    public void reset() {
        imu.resetYaw();
        integral      = 0.0;
        error         = 0.0;
        rate          = 0.0;
        rateDiff      = 0.0;
        rateGyro      = 0.0;
        rateSpikes    = 0;
        setpoint      = 0.0;
        setpointRate  = 0.0;
        prevCurrent   = 0.0;
        imuTurnOutput = 0.0;
        isRunning     = false;
        firstLoop     = true;
        inToleranceSince = 0;
        loopTimer.reset();
    }

    // 目標が変わったときだけ積分と完了判定をリセットする。
    public void setTarget(double degrees) {
        if (targetHeading != degrees) {
            targetHeading = degrees;
            integral      = 0.0;

            setpoint      = getHeading();
            setpointRate  = 0.0;
            inToleranceSince = 0;
            isRunning     = true;
        }

    }

    private boolean cacheEnabled = false;
    private boolean fresh        = false;
    private double cYaw = 0.0, cPitch = 0.0, cRoll = 0.0;

    // メイン周期先頭で呼ぶ。同一周期のgetHeading/updateは同じIMUサンプルを共有する。
    public void beginLoop() {
        cacheEnabled = true;
        fresh = false;
    }

    private void readImuOnce() {
        if (cacheEnabled && fresh) return;
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        cYaw   = ypr.getYaw(AngleUnit.DEGREES);
        cPitch = ypr.getPitch(AngleUnit.DEGREES);
        cRoll  = ypr.getRoll(AngleUnit.DEGREES);
        fresh  = true;
    }

    public double getHeading() {
        readImuOnce();
        return cYaw;
    }

    public boolean isFinished() {
        return !isRunning;
    }

    // 実測dtを使う。長い休止後は角速度履歴を取り直し、差分の跳ねを防ぐ。
    public double update() {

        double measured = loopTimer.seconds();
        loopTimer.reset();
        boolean resumed = false;
        if (measured < DT_MIN) {
            dt = DT_NOMINAL;
        } else if (measured > DT_MAX) {
            dt = DT_MAX;

            resumed = true;
        } else {
            dt = measured;
        }

        readImuOnce();
        current = cYaw;
        pitch   = cPitch;
        roll    = cRoll;

        if (firstLoop || resumed) {
            prevCurrent = current;
            setpoint    = current;
            rateDiff    = 0.0;
            firstLoop   = false;
        } else {
            double rateRaw = normalizeError(current - prevCurrent) / dt;

            if (Math.abs(rateRaw) <= rateSanityMax) {
                rateDiff += rateFilterAlpha * (rateRaw - rateDiff);
            } else {
                rateSpikes++;
            }
            prevCurrent = current;
        }

        if (useGyroRate || readBothRates) {
            AngularVelocity av = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            gyroX = av.xRotationRate;
            gyroY = av.yRotationRate;
            gyroZ = av.zRotationRate;

            rateGyro = gyroSign * gyroZ;
        }

        rate = useGyroRate ? rateGyro : rateDiff;

        // 任意の目標プロファイル。目標が実角度を大きく先行しないよう制限する。
        if (useProfile) {
            double toGo = normalizeError(targetHeading - setpoint);
            double step = clamp(toGo, -maxTurnRate * dt, maxTurnRate * dt);
            setpoint     = normalizeError(setpoint + step);
            setpointRate = step / dt;

            double lead = normalizeError(setpoint - current);
            if (Math.abs(lead) > maxLead) {
                setpoint = normalizeError(current + Math.signum(lead) * maxLead);
            }
        } else {
            setpoint     = targetHeading;
            setpointRate = 0.0;
        }

        error = normalizeError(setpoint - current);
        double finalError = normalizeError(targetHeading - current);

        // 角度誤差と角速度がともに小さい状態の継続で完了とする。
        if (Math.abs(finalError) <= STOP_THRESH && Math.abs(rate) <= STOP_RATE) {
            if (inToleranceSince == 0) {
                inToleranceSince = System.currentTimeMillis();
            } else if (System.currentTimeMillis() - inToleranceSince > STOP_TIME_MS) {
                isRunning = false;
            }
        } else {
            inToleranceSince = 0;
            isRunning = true;
        }

        // 許容帯内ではP/D/摩擦補償を切る。ki=0の現設定では出力0になる。
        if (Math.abs(finalError) <= tolerance && Math.abs(setpointRate) < 1.0) {
            pTerm = vTerm = sTerm = dTerm = 0.0;

            if (Math.abs(finalError) <= finalTolerance) {
                integral      = 0.0;
                iTerm         = 0.0;
                imuTurnOutput = 0.0;
            } else if (Math.abs(rate) > STOP_RATE) {

                iTerm         = 0.0;
                imuTurnOutput = 0.0;
            } else {

                integral += error * dt;
                iTerm = clamp(ki * integral, -iMaxContrib, iMaxContrib);
                if (ki != 0) integral = iTerm / ki;
                imuTurnOutput = clamp(iTerm, -maxTurnPower, maxTurnPower);
            }

            turnSlewed = imuTurnOutput;
            return imuTurnOutput;
        }

        pTerm = kp * error;

        if (Math.abs(error) <= iZone) {
            integral += error * dt;
        } else {
            integral = 0.0;
        }
        iTerm = clamp(ki * integral, -iMaxContrib, iMaxContrib);

        if (ki != 0) integral = iTerm / ki;

        // D項は目標角速度と実角速度の差。摩擦補償は既に動いているときに抑える。
        dTerm = kd * (setpointRate - rate);

        vTerm = kV * setpointRate;

        double ksScale = (Math.abs(finalError) - tolerance) / Math.max(1e-6, ksFull - tolerance);
        ksScale = clamp(ksScale, 0.0, 1.0);

        if (rate * Math.signum(finalError) > ksRateCut) {
            ksScale = 0.0;
        }
        sTerm = kS * ksScale * Math.signum(finalError);

        imuTurnOutput = clamp(pTerm + iTerm + dTerm + vTerm + sTerm, -maxTurnPower, maxTurnPower);
        imuTurnOutput = applyTurnSlew(imuTurnOutput);
        return imuTurnOutput;
    }

    public double getTarget()   { return targetHeading; }
    public double getSetpoint() { return setpoint; }

    // 最後のupdate時点の方位。updateしない手動期間は最新とは限らない。
    public double getCurrent()  { return current; }

    public double getError()    { return normalizeError(targetHeading - current); }

    public double getRate()     { return rate; }

    public double getRateDiff() { return rateDiff; }

    public double getRateGyro() { return rateGyro; }

    public double getPitch()    { return pitch; }
    public double getRoll()     { return roll; }

    public double getGyroX()    { return gyroX; }
    public double getGyroY()    { return gyroY; }
    public double getGyroZ()    { return gyroZ; }

    public int getRateSpikes()  { return rateSpikes; }
    public double getDt()       { return dt; }
    public double getOutput()   { return imuTurnOutput; }

    public void setGains(double kp, double ki, double kd, double kS, double kV) {
        this.kp = kp; this.ki = ki; this.kd = kd; this.kS = kS; this.kV = kV;
    }

    private static double normalizeError(double error) {
        while (error >  180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
