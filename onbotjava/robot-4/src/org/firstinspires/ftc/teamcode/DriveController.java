// ============================================================================
// 最終変更: 2026-10-02 JST
// 変更内容: 撤去した測距センサーへの依存と測距処理を削除。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode;

// 前後・旋回の要求を左右出力へ変換する。モーターへの書き込みはメインが担当。
public class DriveController {
    public RobotHardware hardware;

    public static final double maxSpeed              = 1.0;

    // 旧補正は指令値の左右差を使うため既定0。実際の方位補正はDriveAssistが担当。
    public static final double DRIVE_CORRECTION_GAIN = 0.0;
    public static final double DRIVE_INPUT_EXPONENT  = 1.2;
    public static final double BRAKE_STRENGTH         = 0.05;
    public static final double JOYSTICK_DEADZONE      = 0.05;
    public static final double STOP_THRESHOLD         = 0.02;
    public static final double TURN_NORMAL            = 0.7;


    public DriveController(RobotHardware hr) {
        hardware = hr;
    }

    public static class DriveOutput {
        public final double leftPower;
        public final double rightPower;

        public DriveOutput(double leftPower, double rightPower) {
            this.leftPower  = leftPower;
            this.rightPower = rightPower;
        }
    }

    private CachedPower leftOut = null, rightOut = null;

    public void setOutputCache(CachedPower left, CachedPower right) {
        leftOut  = left;
        rightOut = right;
    }

    private double lastLeftPower() {
        return (leftOut  != null) ? leftOut.getLast()  : hardware.leftMotor.getPower();
    }

    private double lastRightPower() {
        return (rightOut != null) ? rightOut.getLast() : hardware.rightMotor.getPower();
    }

    // 入力曲線を適用して合成する。中立時は前回指令の5%を逆向きに返して減速する。
    public DriveOutput driveManual(double forwardRaw, double turnRaw) {
        double forward = applyInputCurve(forwardRaw, DRIVE_INPUT_EXPONENT);
        double turn    = -applyInputCurve(turnRaw,   DRIVE_INPUT_EXPONENT);

        if (Math.abs(forwardRaw) < JOYSTICK_DEADZONE &&
                Math.abs(turnRaw)    < JOYSTICK_DEADZONE) {

            double leftPower  = lastLeftPower();
            double rightPower = lastRightPower();
            double avgPower   = (Math.abs(leftPower) + Math.abs(rightPower)) / 2.0;

            if (avgPower > STOP_THRESHOLD) {
                double leftTarget  = -Math.signum(leftPower)  * Math.abs(leftPower)  * BRAKE_STRENGTH;
                double rightTarget = -Math.signum(rightPower) * Math.abs(rightPower) * BRAKE_STRENGTH;
                return new DriveOutput(leftTarget, rightTarget);
            } else {
                return new DriveOutput(0, 0);
            }

        } else {
            double leftPowerTarget  = (forward - TURN_NORMAL * turn) * maxSpeed;
            double rightPowerTarget = (forward + TURN_NORMAL * turn) * maxSpeed;

            double maxVal = Math.max(Math.abs(leftPowerTarget), Math.abs(rightPowerTarget));
            if (maxVal > maxSpeed) {
                leftPowerTarget  = leftPowerTarget  / maxVal * maxSpeed;
                rightPowerTarget = rightPowerTarget / maxVal * maxSpeed;
            }

            if (Math.abs(turnRaw) < JOYSTICK_DEADZONE) {

                double powerError = lastLeftPower() - lastRightPower();
                leftPowerTarget  -= powerError * DRIVE_CORRECTION_GAIN;
                rightPowerTarget += powerError * DRIVE_CORRECTION_GAIN;
            }

            return new DriveOutput(
                    clamp(leftPowerTarget,  -1.0, 1.0),
                    clamp(rightPowerTarget, -1.0, 1.0)
            );
        }
    }

    // 手動旋回の符号・入力曲線をDriveAssistと共有する。
    public double manualTurnOutput(double turnRaw) {
        double turn = -applyInputCurve(turnRaw, DRIVE_INPUT_EXPONENT);
        return TURN_NORMAL * turn * maxSpeed;
    }

    // 手動前後とIMU旋回を合成し、左右比率を保って最大値を1へ収める。
    public DriveOutput driveWithImuCorrection(double forwardRaw, double imuTurnOutput) {
        double forward = applyInputCurve(forwardRaw, DRIVE_INPUT_EXPONENT) * maxSpeed;

        double left  = forward - imuTurnOutput;
        double right = forward + imuTurnOutput;

        return normalize(left, right);
    }

    private static DriveOutput normalize(double left, double right) {
        double maxVal = Math.max(Math.abs(left), Math.abs(right));
        if (maxVal > 1.0) {
            left  /= maxVal;
            right /= maxVal;
        }
        return new DriveOutput(left, right);
    }

    public static double applyInputCurve(double input, double exponent) {
        return Math.signum(input) * Math.pow(Math.abs(input), exponent);
    }

    public static double applyDeadzone(double value) {
        return (Math.abs(value) < JOYSTICK_DEADZONE) ? 0 : value;
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
