package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ClimbController {

    public DcMotor climb;
    public Servo rightUp;
    public Servo rightDown;
    public Servo leftUp;
    public Servo leftDown;

    public double targetSpeed = 0.0;
    public double kp = 0.0081;
    public double ki = 0.006302;
    public double kd = 0.000088;
    private double integral = 0;
    private double prevError = 0;
    private static final double dt = 0.02;
    private static final double maxSpeed = 1800;
    public RobotHardware hardware;


    private DcMotorEx Climb_Roller;

    public ClimbController(RobotHardware hr) {
        rightUp = hr.rightUp_Servo;
        rightDown = hr.rightDown_Servo;
        leftUp = hr.leftUp_Servo;
        leftDown = hr.leftDown_Servo;
        Climb_Roller = hr.climbMotor;
    }
    public void climbUp(){
        rightUp.setPosition(1.0);
        rightDown.setPosition(1.0);
        leftUp.setPosition(1.0);
        leftDown.setPosition(1.0);
    }
    public void climbDown(){
        rightUp.setPosition(0.0);
        rightDown.setPosition(0.0);
        leftUp.setPosition(0.0);
        leftDown.setPosition(0.0);
    }
    public void climbStop(){
        rightUp.setPosition(0.5);
        rightDown.setPosition(0.5);
        leftUp.setPosition(0.5);
        leftDown.setPosition(0.5);
    }


    public void Climb_Roller_Forward() {
        Climb_Roller.setPower(1.0);
    }

    public void Climb_Roller_Reverse() {
        Climb_Roller.setPower(-1.0);
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
        double current = Climb_Roller.getVelocity();
        double error = targetSpeed - current;
        integral += error * dt;
        double deriv = (error - prevError) / dt;
        double output = kp * error + ki * integral + kd * deriv;
        output = Math.max(-1.0, Math.min(1.0, output));
        Climb_Roller.setPower(output);
        prevError = error;
    }
}