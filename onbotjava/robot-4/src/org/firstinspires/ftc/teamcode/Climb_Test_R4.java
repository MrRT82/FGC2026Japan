// ============================================================================
// 最終変更: 2026-10-09 JST (UTC+09:00)
// 変更内容: 4号機のクライムサーボとローラーだけを本番と同じ段階・時間で動かす試験用OpModeを追加。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Climb_Test_R4", group = "9_TEST")
public class Climb_Test_R4 extends LinearOpMode {
    private Servo rightUp;
    private Servo rightDown;
    private Servo leftUp;
    private Servo leftDown;
    private Servo rightLock;
    private Servo leftLock;
    private DcMotorEx climbMotor;

    @Override
    public void runOpMode() {
        RobotHardware climbHardware = mapClimbHardwareOnly();
        ClimbController climbController = new ClimbController(climbHardware);
        ClimbSequence climbSequence = new ClimbSequence();
        ElapsedTime sequenceTimer = new ElapsedTime();

        double rollerPower = 1.0;
        telemetry.addLine("4号機 クライム単体試験");
        telemetry.addLine("開始前: D-pad上=ローラー+ / D-pad下=ローラー-");
        telemetry.addLine("開始後: 左トリガーを押すたびに本番の次段階へ進む");
        telemetry.addLine("B=即時停止してOpMode終了");
        telemetry.update();

        while (opModeInInit() && !isStopRequested()) {
            if (gamepad1.dpad_up) rollerPower = 1.0;
            if (gamepad1.dpad_down) rollerPower = -1.0;
            telemetry.addData("ローラー方向", rollerPower > 0 ? "+1.0" : "-1.0");
            telemetry.addLine("INIT中はクライム出力をすべて停止しています");
            telemetry.update();
            idle();
        }

        waitForStart();
        if (isStopRequested()) {
            stopAll();
            return;
        }

        sequenceTimer.reset();
        try {
            while (opModeIsActive()) {
                if (gamepad1.b) {
                    stopAll();
                    requestOpModeStop();
                    break;
                }

                double nowMs = sequenceTimer.milliseconds();
                // この試験では展開を動かさないため、展開準備済みとして0degを渡す。
                int stage = climbSequence.update(gamepad1.left_trigger, nowMs, 0.0);
                // IMUは使わず、開始前に選んだローラー方向を表側出力として固定する。
                climbController.update(stage, nowMs, climbSequence.stageElapsedMs(nowMs),
                        0.0, 0.0, rollerPower);

                telemetry.addData("段階", stageName(stage));
                telemetry.addData("段階経過", "%.1f 秒", climbSequence.stageElapsedMs(nowMs) / 1000.0);
                telemetry.addData("ローラー指令", "%.3f", climbController.getRollerCommand());
                telemetry.addData("ローラー位置", climbController.getRollerPosition());
                telemetry.addData("ローラー電流", "%.2f A", climbMotor.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("位置保持", climbController.isHolding()
                        ? "有効 / 目標 " + climbController.getHoldTarget() : "無効");
                telemetry.addData("下端センサー", "左=%s 右=%s",
                        climbHardware.leftTouch.isPressed(), climbHardware.rightTouch.isPressed());
                telemetry.addLine("左トリガー=次段階 / B=即時停止");
                telemetry.update();
                idle();
            }
        } finally {
            stopAll();
        }
    }

    private RobotHardware mapClimbHardwareOnly() {
        RobotHardware hardware = new RobotHardware();
        rightUp = hardwareMap.get(Servo.class, "rightInside");
        rightDown = hardwareMap.get(Servo.class, "rightOutside");
        leftUp = hardwareMap.get(Servo.class, "leftInside");
        leftDown = hardwareMap.get(Servo.class, "leftOutside");
        rightLock = hardwareMap.get(Servo.class, "rightLock");
        leftLock = hardwareMap.get(Servo.class, "leftLock");
        climbMotor = hardwareMap.get(DcMotorEx.class, "climb");
        TouchSensor leftTouch = hardwareMap.get(TouchSensor.class, "leftTouch");
        TouchSensor rightTouch = hardwareMap.get(TouchSensor.class, "rightTouch");

        rightUp.setDirection(Servo.Direction.REVERSE);
        rightDown.setDirection(Servo.Direction.FORWARD);
        leftUp.setDirection(Servo.Direction.FORWARD);
        leftDown.setDirection(Servo.Direction.REVERSE);
        rightLock.setDirection(Servo.Direction.FORWARD);
        leftLock.setDirection(Servo.Direction.REVERSE);
        climbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopAll();

        hardware.rightUp_Servo = rightUp;
        hardware.rightDown_Servo = rightDown;
        hardware.leftUp_Servo = leftUp;
        hardware.leftDown_Servo = leftDown;
        hardware.rightLock_Servo = rightLock;
        hardware.leftLock_Servo = leftLock;
        hardware.climbMotor = climbMotor;
        hardware.leftTouch = leftTouch;
        hardware.rightTouch = rightTouch;
        return hardware;
    }

    private void stopAll() {
        if (rightUp != null) rightUp.setPosition(0.5);
        if (rightDown != null) rightDown.setPosition(0.5);
        if (leftUp != null) leftUp.setPosition(0.5);
        if (leftDown != null) leftDown.setPosition(0.5);
        if (rightLock != null) rightLock.setPosition(0.5);
        if (leftLock != null) leftLock.setPosition(0.5);
        if (climbMotor != null) climbMotor.setPower(0.0);
    }

    private static String stageName(int stage) {
        switch (stage) {
            case ClimbSequence.WAIT: return "待機";
            case ClimbSequence.LOWER: return "展開待ち省略";
            case ClimbSequence.UP: return "ロック上昇 (10.5秒)";
            case ClimbSequence.DOWN: return "下降 (6秒後ローラー開始 / 7秒まで)";
            case ClimbSequence.ROLL: return "ローラー回転";
            case ClimbSequence.HOLD: return "ローラー位置保持";
            default: return "不明(" + stage + ")";
        }
    }
}
