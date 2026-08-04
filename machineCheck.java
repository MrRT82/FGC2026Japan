package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="HardwareCheck", group="Test")
public class HardwareCheck extends LinearOpMode {

    // ===== モーター =====
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private DcMotorEx left_shooterMotor;
    private DcMotorEx right_shooterMotor;
    private DcMotorEx intakeMotor;
    private DcMotorEx extendMotor;
    private DcMotorEx climbMotor;
    private DcMotor   feedMotor;

    // ===== サーボ =====
    private Servo rightUp_Servo;
    private Servo rightDown_Servo;
    private Servo leftUp_Servo;
    private Servo leftDown_Servo;

    // ===== センサー =====
    private AnalogInput potentiometer;
    private TouchSensor touchSensor;
    private TouchSensor limitSwitch;
    private IMU imu;

    // ===== OK判定フラグ（モーター）=====
    private boolean okLeftMotor         = false;
    private boolean okRightMotor        = false;
    private boolean okLeftShooter       = false;
    private boolean okRightShooter      = false;
    private boolean okIntake            = false;
    private boolean okExtend            = false;
    private boolean okClimb             = false;

    // ===== OK判定フラグ（センサー）=====
    private boolean okPote              = false;
    private boolean okTouchSensor       = false;
    private boolean okLimitSwitch       = false;
    private boolean okImu               = false;

    // ===== サーボテスト用 =====
    private boolean prevA    = false;
    private boolean prevB    = false;

    // ===== IMU基準値 =====
    private double imuBaseline = 0;

    // ===== ポテンショ基準値 =====
    private double poteBaseline = 0;

    // ===== 速度検知しきい値 =====
    private static final double VEL_THRESHOLD  = 50.0;  // ticks/sec
    private static final double POTE_THRESHOLD = 0.05;  // V
    private static final double IMU_THRESHOLD  = 5.0;   // 度

    @Override
    public void runOpMode() {

        // ===== 初期化 =====
        leftMotor          = hardwareMap.get(DcMotorEx.class, "leftmotor");
        rightMotor         = hardwareMap.get(DcMotorEx.class, "rightmotor");
        left_shooterMotor  = hardwareMap.get(DcMotorEx.class, "right_shooter");
        right_shooterMotor = hardwareMap.get(DcMotorEx.class, "left_shooter");
        intakeMotor        = hardwareMap.get(DcMotorEx.class, "intakemotor");
        extendMotor        = hardwareMap.get(DcMotorEx.class, "extendmotor");
        climbMotor         = hardwareMap.get(DcMotorEx.class, "climb");
        feedMotor          = hardwareMap.get(DcMotor.class,   "feed");

        rightUp_Servo   = hardwareMap.get(Servo.class, "rightUp");
        rightDown_Servo = hardwareMap.get(Servo.class, "rightDown");
        leftUp_Servo    = hardwareMap.get(Servo.class, "leftUp");
        leftDown_Servo  = hardwareMap.get(Servo.class, "leftDown");

        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        touchSensor   = hardwareMap.get(TouchSensor.class, "touch_sensor");
        limitSwitch   = hardwareMap.get(TouchSensor.class, "limit_switch");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        // ===== RunMode設定 =====
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ===== サーボ初期位置 =====
        rightUp_Servo.setPosition(0.5);
        rightDown_Servo.setPosition(0.5);
        leftUp_Servo.setPosition(0.5);
        leftDown_Servo.setPosition(0.5);

        imu.resetYaw();
        imuBaseline  = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        poteBaseline = potentiometer.getVoltage();

        telemetry.addData("操作方法", "");
        telemetry.addData("モーター類", "手で回してOKを確認");
        telemetry.addData("A",          "サーボ正転テスト");
        telemetry.addData("B",          "サーボ停止");
        telemetry.addData("センサー類", "押す・動かしてOKを確認");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ===== モーターOK判定（一度でも反応があればOK）=====
            if (!okLeftMotor    && Math.abs(leftMotor.getVelocity())          > VEL_THRESHOLD) okLeftMotor    = true;
            if (!okRightMotor   && Math.abs(rightMotor.getVelocity())         > VEL_THRESHOLD) okRightMotor   = true;
            if (!okLeftShooter  && Math.abs(left_shooterMotor.getVelocity())  > VEL_THRESHOLD) okLeftShooter  = true;
            if (!okRightShooter && Math.abs(right_shooterMotor.getVelocity()) > VEL_THRESHOLD) okRightShooter = true;
            if (!okIntake       && Math.abs(intakeMotor.getVelocity())        > VEL_THRESHOLD) okIntake       = true;
            if (!okExtend       && Math.abs(extendMotor.getVelocity())        > VEL_THRESHOLD) okExtend       = true;
            if (!okClimb        && Math.abs(climbMotor.getVelocity())         > VEL_THRESHOLD) okClimb        = true;

            // ===== センサーOK判定 =====
            // ポテンショ：基準値から変化があればOK
            if (!okPote && Math.abs(potentiometer.getVoltage() - poteBaseline) > POTE_THRESHOLD) {
                okPote = true;
            }
            // タッチセンサー：一度でも押されればOK
            if (!okTouchSensor && touchSensor.isPressed()) okTouchSensor = true;
            // リミットスイッチ：一度でも押されればOK
            if (!okLimitSwitch && limitSwitch.isPressed()) okLimitSwitch = true;
            // IMU：基準値から5度以上変化があればOK
            double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            if (!okImu && Math.abs(currentYaw - imuBaseline) > IMU_THRESHOLD) okImu = true;

            // ===== サーボテスト =====
            if (gamepad1.a && !prevA) {
                // 正転（連続回転モード）
                rightUp_Servo.setPosition(1.0);
                rightDown_Servo.setPosition(1.0);
                leftUp_Servo.setPosition(1.0);
                leftDown_Servo.setPosition(1.0);
            }
            if (gamepad1.b && !prevB) {
                // 停止
                rightUp_Servo.setPosition(0.5);
                rightDown_Servo.setPosition(0.5);
                leftUp_Servo.setPosition(0.5);
                leftDown_Servo.setPosition(0.5);
            }

            // ===== telemetry =====
            telemetry.addData("== モーター ==", "（手で回してOKを確認）");
            telemetry.addData("左ドライブ",   okLeftMotor    ? "✅ OK" : "❌ 未確認");
            telemetry.addData("右ドライブ",   okRightMotor   ? "✅ OK" : "❌ 未確認");
            telemetry.addData("左シューター", okLeftShooter  ? "✅ OK" : "❌ 未確認");
            telemetry.addData("右シューター", okRightShooter ? "✅ OK" : "❌ 未確認");
            telemetry.addData("インテーク",   okIntake       ? "✅ OK" : "❌ 未確認");
            telemetry.addData("展開",         okExtend       ? "✅ OK" : "❌ 未確認");
            telemetry.addData("クライム",     okClimb        ? "✅ OK" : "❌ 未確認");

            telemetry.addData("== サーボ ==", "（A:正転 B:停止）");
            telemetry.addData("rightUp",   rightUp_Servo.getPosition()   > 0.6 ? "▶ 正転中" : "■ 停止");
            telemetry.addData("rightDown", rightDown_Servo.getPosition() > 0.6 ? "▶ 正転中" : "■ 停止");
            telemetry.addData("leftUp",    leftUp_Servo.getPosition()    > 0.6 ? "▶ 正転中" : "■ 停止");
            telemetry.addData("leftDown",  leftDown_Servo.getPosition()  > 0.6 ? "▶ 正転中" : "■ 停止");

            telemetry.addData("== センサー ==", "（押す・動かしてOKを確認）");
            telemetry.addData("ポテンショ",       okPote        ? "✅ OK" : "❌ 未確認");
            telemetry.addData("タッチセンサー",   okTouchSensor ? "✅ OK" : "❌ 未確認");
            telemetry.addData("リミットスイッチ", okLimitSwitch ? "✅ OK" : "❌ 未確認");
            telemetry.addData("IMU",              okImu         ? "✅ OK" : "❌ 未確認");

            // ===== 全体の合否 =====
            boolean allOk = okLeftMotor && okRightMotor && okLeftShooter && okRightShooter
                    && okIntake && okExtend && okClimb
                    && okPote && okTouchSensor && okLimitSwitch && okImu;
            telemetry.addData("================", "");
            telemetry.addData("総合判定", allOk ? "✅ 全て OK！" : "⚠️ 未確認項目あり");
            telemetry.update();

            prevA = gamepad1.a;
            prevB = gamepad1.b;
        }
    }
}