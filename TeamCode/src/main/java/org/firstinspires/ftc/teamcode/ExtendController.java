package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class ExtendController {
    public DcMotorEx extend;
    public AnalogInput pote;

    double pote_offset = 20;
    double pote_Up = 70;
    double pote_Down = 0;
    double targetAngle = 17;
    double current;
    double dt = 0.02;
    double integral = 0;
    double kp = 0.02;
    double ki = 0.002;
    double kd = 0.001;
    double output;
    double prevError = 0;

    private double m = 0.1;
    private double r = 0.29;
    private double g = 10;

    // 重心のオフセット（current = 0 のときに重心が17度）
    private final double gravity_offset_angle = 17.0;
    private double angle = 0;

    public ExtendController(RobotHardware hr){
        extend = hr.extendMotor;
        pote = hr.potentiometer;
    }

    public void extendDown(){
        targetAngle = pote_Down;
        integral = 0;
    }

    public void extendUp(){
        if(targetAngle < pote_Up){
            targetAngle = targetAngle + 1.4;
            integral = 0;
        }
    }

    public void extend_loop(){
        current = (pote.getVoltage() / pote.getMaxVoltage()) * 270 / 2 - pote_offset;

        // 実際の重心角度 = 現在角度(current) + 重心オフセット(17度)
        angle = current + gravity_offset_angle;

        double error = targetAngle - current;

        integral += error * dt;
        double deriv = (error - prevError) / dt;

        // 重力補償 (F項): 実際の物理的な重心角度(angle)を使って cos を計算
        double feedforward = (m * g * r * Math.cos(Math.toRadians(angle))) / 3.2;

        output = kp * error + ki * integral + kd * deriv + feedforward;

        output = Math.max(-1.0, Math.min(1.0, output));

        extend.setPower(-output);

        prevError = error;
    }
}