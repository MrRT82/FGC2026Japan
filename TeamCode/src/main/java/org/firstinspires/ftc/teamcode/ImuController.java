package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ImuController {

    private final IMU       imu;
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;

    double output = 0;

    // ===== PIDゲイン =====
    public double kp = 0.00590;
    public double ki = 0.0001;
    public double kd = 0.0005;
    public double kf = 0.3;

    // ===== 定数 =====
    private static final double dt           = 0.02;
    private static final double STOP_THRESH  = 3.0;
    private static final double INTEGRAL_MAX = 1.0;

    // ===== 状態 =====
    public double  targetHeading = 0.0;
    private double integral      = 0.0;
    private double prevError     = 0.0;
    private boolean isRunning    = false;
    private int ffCount = 0;
    private static final int FF_LOOP = 15;

    // =============================================
    // コンストラクタ（RobotHardware版）
    // =============================================
    public ImuController(RobotHardware hw) {
        imu        = hw.imu;
        leftMotor  = hw.leftMotor;
        rightMotor = hw.rightMotor;
    }

    // =============================================
    // コンストラクタ（直接渡す版・test_1向け）
    // =============================================
    public ImuController(IMU imu, DcMotorEx left, DcMotorEx right) {
        this.imu   = imu;
        leftMotor  = left;
        rightMotor = right;
    }

    // =============================================
    // 公開メソッド
    // =============================================

    /** Start後に呼ぶ。現在の向きを0度にリセット */
    public void reset() {
        imu.resetYaw();
        integral  = 0.0;
        prevError = 0.0;
        isRunning = false;
    }

    /** 目標角度を設定して制御開始 */
    public void setTarget(double degrees) {
        targetHeading = degrees;
        integral      = 0.0;
        prevError     = 0.0;
        isRunning     = true;
        ffCount = 0;
    }

    /** 現在のヨー角を取得（度）*/
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /** 目標に到達しているか */
    public boolean isFinished() {
        return !isRunning;
    }

    /** メインループから毎ループ呼ぶ */
    public void update() {
        if (!isRunning) return;

        double current = getHeading();
        double error   = normalizeError(targetHeading - current);

        // ===== 到達判定 =====
        if (Math.abs(error) < STOP_THRESH) {
            leftMotor.setVelocity(0);
            rightMotor.setVelocity(0);
            integral  = 0.0;
            prevError = 0.0;
            isRunning = false;
            return;
        }

        // ===== PI + F =====
        integral += error * dt;
        integral  = Math.max(-INTEGRAL_MAX, Math.min(INTEGRAL_MAX, integral));
        double deriv  = (error - prevError) / dt;   //Dなのでいらない

        double ff = 0.0;
        if (ffCount < FF_LOOP) {
            ff = Math.signum(error) * kf;
            ffCount++;
        }
        output = kp * error + ki * integral + ff;
        output = Math.max(-1.0, Math.min(1.0, output));

        leftMotor.setPower(-output);
        rightMotor.setPower(output);

        prevError = error;
    }
    public double getTarget() {
        return targetHeading;
    }

    // =============================================
    // ユーティリティ
    // =============================================

    private double normalizeError(double error) {
        while (error >  180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }
}