package org.firstinspires.ftc.teamcode;

public class DriveController {
    public RobotHardware hardware;

    // ----- ドライブ固有定数 -----
    public static final double maxSpeed    = 5000;
    public static final double DRIVE_CORRECTION_GAIN = 0.05;
    public static final double DRIVE_INPUT_EXPONENT  = 1.5;
    public static final double BRAKE_STRENGTH        = 0.2;
    public static final double JOYSTICK_DEADZONE     = 0.05;
    public static final double STOP_THRESHOLD        = 30.0;
    public static final double TURN_NORMAL           = 0.7;
    public static final double ALIGN_KP              = 0.001;

    public DriveController(RobotHardware hr) {
        hardware = hr;
    }

    public void driveNormal(double forwardRaw, double turnRaw) {
        double forward = applyInputCurve(forwardRaw, DRIVE_INPUT_EXPONENT);
        double turn    = -applyInputCurve(turnRaw,    DRIVE_INPUT_EXPONENT);

        if (Math.abs(forwardRaw) < JOYSTICK_DEADZONE &&
                Math.abs(turnRaw)    < JOYSTICK_DEADZONE) {

            double leftVel  = hardware.leftMotor.getVelocity();
            double rightVel = hardware.rightMotor.getVelocity();
            double avgVel   = (Math.abs(leftVel) + Math.abs(rightVel)) / 2.0;

            if (avgVel > STOP_THRESHOLD) {
                double leftTarget  = -Math.signum(leftVel)  * Math.abs(leftVel)  * BRAKE_STRENGTH;
                double rightTarget = -Math.signum(rightVel) * Math.abs(rightVel) * BRAKE_STRENGTH;

                hardware.leftMotor.setVelocity(leftTarget);
                hardware.rightMotor.setVelocity(rightTarget);
            } else {
                hardware.leftMotor.setVelocity(0);
                hardware.rightMotor.setVelocity(0);
            }

        } else {
            double leftVelTarget  = (forward - TURN_NORMAL * turn) * maxSpeed;
            double rightVelTarget = (forward + TURN_NORMAL * turn) * maxSpeed;

            double velError = hardware.leftMotor.getVelocity() - hardware.rightMotor.getVelocity();
            leftVelTarget  -= velError * DRIVE_CORRECTION_GAIN;
            rightVelTarget += velError * DRIVE_CORRECTION_GAIN;

            hardware.leftMotor.setVelocity(leftVelTarget);
            hardware.rightMotor.setVelocity(rightVelTarget);
        }
    }

    public static double applyInputCurve(double input, double exponent) {
        return Math.signum(input) * Math.pow(Math.abs(input), exponent);
    }

    public static double applyDeadzone(double value) {
        return (Math.abs(value) < JOYSTICK_DEADZONE) ? 0 : value;
    }
}