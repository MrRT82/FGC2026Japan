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

    public static class IntakeController {
        DcMotor intakeMotor;
        ColorSensor colorSensor;
        private static final double INTAKE_POWER = 0.8;

        public IntakeController(RobotHardware hw) {
            intakeMotor = hw.intake;
            colorSensor = hw.color;
        }

        public void startIntake() {
            intakeMotor.setPower(INTAKE_POWER);
        }

        public void stopIntake() {
            intakeMotor.setPower(0);
        }

        public boolean hasBall() {
            if (colorSensor.red() > 150 || colorSensor.blue() > 150) {
                return true;
            }
            else{
                return false;
            }
        }
        public String getBallColor(){
            if(colorSensor.red() > 150){
                return "Red";
            }
            else if(colorSensor.blue() > 150){
                return "Blue";
            }
            else{
                return "None";
            }
        }
    }
}