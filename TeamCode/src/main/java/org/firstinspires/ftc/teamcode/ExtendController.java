package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class ExtendController {
    public DcMotorEx extend;
    public AnalogInput pote;

    double pote_offset = 21;
    double pote_Up = 80;
    double pote_Down = -12;
    double targetAngle = 80;
    double current;
    double dt = 0.02;
    double integral = 0;
    double kp = 0.002;
    double ki = 0.00;
    double kd = 0.00;
    double output;
    double prevError = 0;

    private double m = 0.1;
    private double r = 0.29;
    private double g = 10;

    private final double gravity_offset_angle = 17.0;
    private double angle = 0;

    // ===== 詰まり検知 =====
    private static final double STUCK_RANGE_MIN = 10.0;  // 検知範囲（下限）
    private static final double STUCK_RANGE_MAX = 60.0;  // 検知範囲（上限）
    private static final double STUCK_THRESHOLD = 0.5;   // 変化なし判定しきい値（度）
    private static final long   STUCK_TIME_MS   = 500;   // 詰まり判定時間（ms）
    private static final long   REVERSE_TIME_MS = 500;   // 逆転時間（ms）

    private double  prevPoteValue     = 0.0;
    private long    poteStuckStart    = 0;
    private long    stuckReverseStart = 0;
    public  boolean isStuck           = false;

    public ExtendController(RobotHardware hr){
        extend = hr.extendMotor;
        pote   = hr.potentiometer;
    }

    public void extendDown(){
        targetAngle = pote_Down;
        integral    = 0;
    }

    public void extendUp(){
        if (!isStuck && targetAngle < pote_Up) {
            targetAngle = targetAngle + 1;
            integral    = 0;
        }
    }

    public void extend_loop(){
        current = (pote.getVoltage() / pote.getMaxVoltage()) * 270 / 2 - pote_offset;
        angle   = current + gravity_offset_angle;

        // ===== 詰まり逆転中 =====
        if (isStuck) {
            if (System.currentTimeMillis() - stuckReverseStart < REVERSE_TIME_MS) {
                extend.setPower(0.4);
                integral  = 0;
                prevError = 0;
                return;
            } else {
                // 逆転終了
                isStuck        = false;
                poteStuckStart = 0;
                prevPoteValue  = current;
            }
        }

        // ===== 詰まり検知（範囲内のみ）=====
        if (STUCK_RANGE_MIN <= current && current <= STUCK_RANGE_MAX) {
            if (Math.abs(current - prevPoteValue) < STUCK_THRESHOLD) {
                // 変化なし
                if (poteStuckStart == 0) {
                    poteStuckStart = System.currentTimeMillis();
                } else if (System.currentTimeMillis() - poteStuckStart > STUCK_TIME_MS) {
                    // 詰まり判定
                    isStuck           = true;
                    stuckReverseStart = System.currentTimeMillis();
                    poteStuckStart    = 0;
                    return;
                }
            } else {
                // 変化あり → タイマーリセット
                poteStuckStart = 0;
            }
            prevPoteValue = current;
        } else {
            poteStuckStart = 0;
        }

        // ===== 通常PID =====
        double error = targetAngle - current;
        integral += error * dt;
        integral  = Math.min(0.5, Math.max(integral, -0.5));
        double deriv = (error - prevError) / dt;
        output = kp * error + ki * integral + kd * deriv;
        output = Math.max(-1.0, Math.min(1.0, output));
        extend.setPower(-output);
        prevError = error;
    }
}