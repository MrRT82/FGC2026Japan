package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** Independent robot: three motors, no servos or travel-limit logic. */
@TeleOp(name = "Simple_Drive_Climb", group = "OTHER_TEAM")
public class SimpleDriveClimb extends LinearOpMode {
    // Change these three names to the actual Robot Configuration names.
    static final String LEFT_NAME = "left_drive";
    static final String RIGHT_NAME = "right_drive";
    static final String CLIMB_NAME = "climb";

    // Starting values only: motor speed, load, gearing and polarity are unverified.
    static final double DRIVE_MAX_TICKS_PER_SEC = 1000.0;
    static final double CLIMB_POWER = 0.30;
    static final double HOLD_KP = 0.005; // Same as Robot 4 ClimbController.ROLLER_KP (power/tick)
    static final double TRIGGER_THRESHOLD = 0.20;
    static final DcMotorSimple.Direction LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    static final DcMotorSimple.Direction RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    static final DcMotorSimple.Direction CLIMB_DIRECTION = DcMotorSimple.Direction.FORWARD;

    private DcMotorEx left, right, climb;

    @Override public void runOpMode() throws InterruptedException {
        try {
            left = hardwareMap.get(DcMotorEx.class, LEFT_NAME);
            right = hardwareMap.get(DcMotorEx.class, RIGHT_NAME);
            climb = hardwareMap.get(DcMotorEx.class, CLIMB_NAME);
            left.setPower(0); right.setPower(0); climb.setPower(0);
            left.setDirection(LEFT_DIRECTION);
            right.setDirection(RIGHT_DIRECTION);
            climb.setDirection(CLIMB_DIRECTION);
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            climb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addLine("右Y:前後 / 左X:旋回");
            telemetry.addLine("RT:上昇 / LT:下降 / 同じトリガー再押し:位置保持");
            telemetry.addLine("両トリガー:位置保持 / 画面STOP:全出力停止");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            ClimbControl control = new ClimbControl(climb.getCurrentPosition());
            while (opModeIsActive()) {
                double forward = -gamepad1.right_stick_y;
                double turn = gamepad1.left_stick_x;
                double[] speeds = driveMix(forward, turn);
                left.setVelocity(speeds[0] * DRIVE_MAX_TICKS_PER_SEC);
                right.setVelocity(speeds[1] * DRIVE_MAX_TICKS_PER_SEC);

                int position = climb.getCurrentPosition();
                control.update(gamepad1.right_trigger > TRIGGER_THRESHOLD,
                        gamepad1.left_trigger > TRIGGER_THRESHOLD, position);
                climb.setPower(control.power(position));

                telemetry.addData("Climb", control.mode);
                telemetry.addData("Position / hold", "%d / %d", position, control.holdPosition);
                telemetry.update();
                sleep(20);
            }
        } finally {
            // Each motor is stopped even if another motor's stop command throws.
            try { if (left != null) left.setPower(0); }
            finally {
                try { if (right != null) right.setPower(0); }
                finally { if (climb != null) climb.setPower(0); }
            }
        }
    }

    // Normalize the pair together so combined forward/turn never exceeds the speed setting.
    static double[] driveMix(double forward, double turn) {
        double l = forward + turn, r = forward - turn;
        double scale = Math.max(1.0, Math.max(Math.abs(l), Math.abs(r)));
        return new double[]{l / scale, r / scale};
    }

    enum Mode { HOLD, UP, DOWN }

    static class ClimbControl {
        Mode mode = Mode.HOLD;
        int holdPosition;
        boolean previousRight, previousLeft, simultaneous;

        ClimbControl(int position) { holdPosition = position; }

        void hold(int position) {
            if (mode != Mode.HOLD) holdPosition = position;
            mode = Mode.HOLD;
        }

        void update(boolean rightPressed, boolean leftPressed, int position) {
            if (rightPressed && leftPressed) {
                hold(position);
                simultaneous = true;
            } else if (simultaneous) {
                // After a simultaneous stop, release both before accepting a new command.
                if (!rightPressed && !leftPressed) simultaneous = false;
            } else if (rightPressed && !previousRight) {
                if (mode == Mode.UP) hold(position);
                else mode = Mode.UP;
            } else if (leftPressed && !previousLeft) {
                if (mode == Mode.DOWN) hold(position);
                else mode = Mode.DOWN;
            }
            previousRight = rightPressed;
            previousLeft = leftPressed;
        }

        double power(int position) {
            if (mode == Mode.UP) return CLIMB_POWER;
            if (mode == Mode.DOWN) return -CLIMB_POWER;
            // Only P control; clipping is the SDK's valid power range, not an angle limit.
            double error = (double) holdPosition - position;
            return Math.max(-1.0, Math.min(1.0, HOLD_KP * error));
        }
    }
}
