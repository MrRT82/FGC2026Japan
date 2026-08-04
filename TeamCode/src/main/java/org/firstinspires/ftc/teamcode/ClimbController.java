package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class ClimbController {
    public Servo rightUp;
    public Servo rightDown;
    public Servo leftUp;
    public Servo leftDown;
    public Servo rightLock;
    public Servo leftLock;
    public TouchSensor bottomSensor;
    public RobotHardware hardware;

    private DcMotorEx Climb_Roller;

    public ClimbController(RobotHardware hr) {
        rightUp = hr.rightUp_Servo;
        rightDown = hr.rightDown_Servo;
        leftUp = hr.leftUp_Servo;
        leftDown = hr.leftDown_Servo;
        rightLock = hr.rightLock_Servo;
        leftLock = hr.leftLock_Servo;
        Climb_Roller = hr.climbMotor;
        bottomSensor = hr.limitSwitch;
    }
    public void climbOpen(){
        rightLock.setPosition(0.5);//あとでなおす
        leftLock.setPosition(0.5);//あとでなおす
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