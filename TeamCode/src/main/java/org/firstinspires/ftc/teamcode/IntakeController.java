package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeController {
    public DcMotorEx Intake;

    public IntakeController(RobotHardware hr) {
        Intake = hr.intakeMotor;
    }

    public void Intake_Forward() {
        Intake.setPower(1.0);
    }

    public void Intake_Reverse() {
        Intake.setPower(-1.0);
    }
}