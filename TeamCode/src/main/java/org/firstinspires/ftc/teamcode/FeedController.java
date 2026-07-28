package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class FeedController
{
    private DcMotor feed;
    public FeedController(RobotHardware hr){
        feed = hr.feedMotor;
    }
    public void Feed_turn_right(){
        feed.setPower(1);
    }
    public void Feed_turn_left(){
        feed.setPower(-1);
    }
    public void Feed_turn_stop(){
        feed.setPower(0);
    }
}