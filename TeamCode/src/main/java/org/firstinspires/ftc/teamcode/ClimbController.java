package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class ClimbController {

    public DcMotor climb;
    public Servo rightUp;
    public Servo rightDown;
    public Servo leftUp;
    public Servo leftDown;
    public TouchSensor upperSensor;
    public TouchSensor bottomSensor;

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
        upperSensor = hr.touchSensor;
        bottomSensor = hr.limitSwitch;
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

    public boolean isUpperLimit(){
        return upperSensor.isPressed();
    }
    public boolean isBottomLimit(){
        return bottomSensor.isPressed();
    }

    public void Climb_Roller_Forward() {
        Climb_Roller.setPower(1.0);
    }

    public void Climb_Roller_Reverse() {
        Climb_Roller.setPower(-1.0);
    }
}