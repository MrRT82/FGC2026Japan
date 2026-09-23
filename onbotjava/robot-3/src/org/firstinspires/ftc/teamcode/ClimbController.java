// ============================================================================
// 最終変更: 2026-09-22 11:20:42 JST (UTC+09:00)
// 変更内容: 変更日時ヘッダーを追加。直前までの修正内容を保持。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

// 登坂サーボの0/0.5/1指令とロックを管理する。動作段階はメインのclimbEnumが担当。
public class ClimbController {
    public Servo rightUp;
    public Servo rightDown;
    public Servo leftUp;
    public Servo leftDown;
    public Servo rightLock;
    public Servo leftLock;
    public TouchSensor bottomSensor;
    public TouchSensor BottomSensor;
    public RobotHardware hardware;

    private DcMotorEx Climb_Roller;

    private CachedPower rollerOut;

    private CachedServo rightUpC, rightDownC, leftUpC, leftDownC, rightLockC, leftLockC;

    public ClimbController(RobotHardware hr) {
        rightUp = hr.rightUp_Servo;
        rightDown = hr.rightDown_Servo;
        leftUp = hr.leftUp_Servo;
        leftDown = hr.leftDown_Servo;
        rightLock = hr.rightLock_Servo;
        leftLock = hr.leftLock_Servo;
        Climb_Roller = hr.climbMotor;
        rollerOut = new CachedPower(Climb_Roller, 0.02);
        bottomSensor = hr.leftTouch;
        BottomSensor = hr.rightTouch;

        rightUpC   = new CachedServo(rightUp);
        rightDownC = new CachedServo(rightDown);
        leftUpC    = new CachedServo(leftUp);
        leftDownC  = new CachedServo(leftDown);
        rightLockC = new CachedServo(rightLock);
        leftLockC  = new CachedServo(leftLock);
    }

    public double getSkipRate(){
        if (rightUpC == null) return 0.0;
        return (rightUpC.getSkipRate() + rightDownC.getSkipRate()
                + leftUpC.getSkipRate() + leftDownC.getSkipRate()) / 4.0;
    }
    // ロック解除位置。実機の取付に合わせて校正する。
    public void climbOpen(){
        rightLockC.set(0.2);
        leftLockC.set(0);
    }
    public void climbUp(){
        rightUpC.set(1.0);
        rightDownC.set(1.0);
        leftUpC.set(1.0);
        leftDownC.set(1.0);
    }
    public void climbDown(){
        rightUpC.set(0.0);
        rightDownC.set(0.0);
        leftUpC.set(0.0);
        leftDownC.set(0.0);
    }
    public void climbStop(){
        rightUpC.set(0.5);
        rightDownC.set(0.5);
        leftUpC.set(0.5);
        leftDownC.set(0.5);
    }
    // 左右いずれかの下端センサーで下降を止める。
    public boolean isBottomLimit(){
        return (bottomSensor.isPressed() || BottomSensor.isPressed());
    }

    public void Climb_Roller_Forward() {
        rollerOut.set(1.0);
    }

    public void Climb_Roller_Reverse() {
        rollerOut.set(-1.0);
    }
}
