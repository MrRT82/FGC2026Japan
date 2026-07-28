package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ShooterController {
    public double targetSpeed = 0.0;

    // 速度型PIDのゲイン
    public double kp = 0.001;
    public double ki = 0.0001;
    public double kd = 0.00005;
    public double kf = 0.0005; // Feedforward 係数

    private double prevError = 0.0;     // e(k-1)
    private double prevPrevError = 0.0; // e(k-2)
    private double currentOutput = 0.0; // 現在の出力パワー u(k-1)

    private static final double dt = 0.02;
    private static final double maxSpeed = 1800;

    public DcMotorEx leftShooter;
    public DcMotorEx rightShooter;

    public ShooterController(RobotHardware hr) {
        leftShooter = hr.left_shooterMotor;
        rightShooter = hr.right_shooterMotor;
    }

    public void setSpeed(double speed) {
        targetSpeed = speed;
        if (speed == 0) {
            resetPID();
        }
    }

    public void speedUp() {
        setSpeed(Math.min(maxSpeed, targetSpeed + 100));
    }

    public void speedDown() {
        setSpeed(Math.max(-maxSpeed, targetSpeed - 100));
    }

    private void resetPID() {
        prevError = 0.0;
        prevPrevError = 0.0;
        currentOutput = 0.0;
    }

    public void shooter_loop() {
        targetSpeed = Math.max(0, targetSpeed);

        if (targetSpeed == 0) {
            leftShooter.setPower(0);
            rightShooter.setPower(0);
            resetPID();
            return;
        }

        double current = (leftShooter.getVelocity() + rightShooter.getVelocity()) / 2.0;
        double error = targetSpeed - current;

        double deltaP = error - prevError;

        double deltaI = error * dt;

        double deltaD = (error - 2.0 * prevError + prevPrevError) / dt;

        double deltaOutput = (kp * deltaP) + (ki * deltaI) + (kd * deltaD);

        double ff = targetSpeed * kf;
        currentOutput += deltaOutput;

        double finalOutput = Math.max(-1.0, Math.min(1.0, currentOutput + ff));

        leftShooter.setPower(finalOutput);
        rightShooter.setPower(finalOutput);

        prevPrevError = prevError;
        prevError = error;
    }
}