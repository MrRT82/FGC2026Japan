// ============================================================================
// 最終変更: 2026-10-02 JST
// 変更内容: 撤去した測距センサーへの依存と測距処理を削除。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.TouchSensor;

// 設定名と実機の対応を集約する。方向や取付姿勢の変更は全OpModeに影響する。
public class RobotHardware {
    public DcMotorEx leftMotor;
    public DcMotorEx rightMotor;

    public DcMotorEx left_shooterMotor;
    public DcMotorEx right_shooterMotor;

    public DcMotorEx intakeMotor;

    public DcMotorEx extendMotor;
    public DcMotor feedMotor;
    public DcMotorEx climbMotor;

    public Servo rightUp_Servo;
    public Servo rightDown_Servo;
    public Servo leftUp_Servo;
    public Servo leftDown_Servo;
    public Servo rightLock_Servo;
    public Servo leftLock_Servo;
    public AnalogInput potentiometer;

    public IMU imu;

    public TouchSensor rightTouch;
    public TouchSensor leftTouch;


    public void init(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftmotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightmotor");

        // 左右シューターは同じ軸を駆動する。物理配線と設定名の対応を確認する。
        left_shooterMotor  = hardwareMap.get(DcMotorEx.class, "left_shooter");
        right_shooterMotor = hardwareMap.get(DcMotorEx.class, "right_shooter");
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakemotor");
        extendMotor = hardwareMap.get(DcMotorEx.class,"extendmotor");
        feedMotor = hardwareMap.get(DcMotor.class, "feed");
        climbMotor = hardwareMap.get(DcMotorEx.class,"climb");
        potentiometer = hardwareMap.get(AnalogInput.class,"potentiometer");
        leftTouch  = hardwareMap.get(TouchSensor.class, "leftTouch");
        rightTouch = hardwareMap.get(TouchSensor.class, "rightTouch");

        imu = hardwareMap.get(IMU.class, "imu");

        rightUp_Servo = hardwareMap.get(Servo.class,"rightInside");
        rightDown_Servo = hardwareMap.get(Servo.class,"rightOutside");
        leftUp_Servo = hardwareMap.get(Servo.class,"leftInside");
        leftDown_Servo = hardwareMap.get(Servo.class,"leftOutside");
        rightLock_Servo = hardwareMap.get(Servo.class,"rightLock");
        leftLock_Servo = hardwareMap.get(Servo.class,"leftLock");

        // 走行はBRAKE、回収・展開はFLOAT。BRAKEは位置や方位を保証する保持制御ではない。
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        left_shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        right_shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        extendMotor.setDirection(DcMotor.Direction.FORWARD);
        rightUp_Servo.setDirection(Servo.Direction.FORWARD);
        rightDown_Servo.setDirection(Servo.Direction.REVERSE);
        leftUp_Servo.setDirection(Servo.Direction.REVERSE);
        leftDown_Servo.setDirection(Servo.Direction.FORWARD);
        rightLock_Servo.setDirection(Servo.Direction.FORWARD);
        leftLock_Servo.setDirection(Servo.Direction.REVERSE);

        // 初期化時に登坂サーボを中立、ロックを初期位置へ指令する。
        rightUp_Servo.setPosition(0.5);
        rightDown_Servo.setPosition(0.5);
        leftUp_Servo.setPosition(0.5);
        leftDown_Servo.setPosition(0.5);
        rightLock_Servo.setPosition(0.3);
        leftLock_Servo.setPosition(0.15);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // 走行はHub内の速度制御。シューター・展開は独自制御のためRUN_WITHOUT_ENCODER。
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 実機取付：ロゴLEFT、USB BACKWARD。取付変更時は必ず更新する。
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        ));
    }
}
