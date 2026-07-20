package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ShooterController
{
    public double targetSpeed = 0.0;
    public double kp = 0.0081;
    public double ki = 0.006302;
    public double kd = 0.000088;
    private double integral = 0;
    private double prevError = 0;
    private static final double dt = 0.02;
    private static final double maxSpeed = 1800;
    public RobotHardware hardware;

    public DcMotorEx leftShooter;
    public DcMotorEx rightShooter;

    public ShooterController(RobotHardware hr){
        leftShooter = hr.left_shooterMotor;
        rightShooter = hr.right_shooterMotor;
    }
    public void setSpeed(double speed) {
        targetSpeed = speed;
        integral = 0;
        prevError = 0;
    }
    public void speedUp(){
        targetSpeed += 100;
        targetSpeed = Math.min(maxSpeed,targetSpeed);
        integral = 0;
        prevError = 0;
    }
    public void speedDown(){
        targetSpeed -= 100;
        targetSpeed = Math.max(-maxSpeed,targetSpeed);
        integral = 0;
        prevError = 0;
    }
    public void shooter_loop(){
        double current = (leftShooter.getVelocity() + rightShooter.getVelocity()) / 2;
        double error = targetSpeed - current;

        integral += error * dt;
        double deriv = (error - prevError) / dt;
        double output = kp * error + ki * integral + kd * deriv;

        output = Math.max(-1.0, Math.min(1.0, output));

        leftShooter.setPower(output);
        rightShooter.setPower(output);

        prevError = error;
    }
}
