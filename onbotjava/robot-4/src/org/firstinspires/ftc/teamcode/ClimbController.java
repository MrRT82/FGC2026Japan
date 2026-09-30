// ============================================================================
// 最終変更: 2026-09-30 JST (UTC+09:00)
// 変更内容: ロックサーボ左右を無限回転にし、登坂サーボと連動し、登坂の1/2の速さで回す方式へ変更。上昇・下降を時間で停止。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

// 登坂サーボとロック(連動回転)の0/0.5/1指令を管理する。動作段階はメインのclimbEnumが担当。
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
    // ロック機構は無限回転サーボで、登坂サーボと連動して回る。0.5で停止。
    // 速さは登坂サーボの1/2。速さは0.5からの差で0〜0.5(0.5が全速)。
    // 回す向きが逆なら、LOCK_DIRを-1にする。
    private static final double LOCK_STOP = 0.5;
    private static final double CLIMB_SPEED = 0.5;          // climbUp/Downの1.0/0.0に相当
    private static final double LOCK_SPEED_RATIO = 0.5;     // 登坂に対するロックの速さ
    private static final double LOCK_UP_SPEED = CLIMB_SPEED * LOCK_SPEED_RATIO;
    private static final double LOCK_DOWN_SPEED = CLIMB_SPEED * LOCK_SPEED_RATIO;
    private static final double LOCK_DIR = 1;
    private static final double LOCK_UP_OUT = LOCK_STOP + LOCK_DIR * LOCK_UP_SPEED;
    private static final double LOCK_DOWN_OUT = LOCK_STOP - LOCK_DIR * LOCK_DOWN_SPEED;

    // 左右に同じ値を同時に送る。左右はRobotHardwareで向きを逆に設定しているため、左右は逆向きに同期して回る。
    private void setLock(double out){
        rightLockC.set(out);
        leftLockC.set(out);
    }
    // 旧解錠段階。連動方式では解錠動作がないため、ロックは停止のまま。
    public void climbOpen(){
        setLock(LOCK_STOP);
    }
    // 上昇・下降は時間で止める。実機に合わせてCLIMB_UP_MSとCLIMB_DOWN_MSを校正する(仮値)。
    // 計時は向きが上昇⇔下降で切り替わった時だけ始め直す。途中のclimbStop()や
    // 下端センサーの断続では延長しない。
    private static final long CLIMB_UP_MS = 2000, CLIMB_DOWN_MS = 2000;
    private int motionDir = 0;          // 1: 上昇, -1: 下降, 0: 未動作
    private long motionStartMs;

    // 指定方向の計時中ならtrue。向きが変わった時に計時を始める。
    private boolean withinTime(int dir, long limitMs){
        long now = System.currentTimeMillis();
        if (motionDir != dir) { motionDir = dir; motionStartMs = now; }
        return now - motionStartMs < limitMs;
    }
    public void climbUp(){
        if (!withinTime(1, CLIMB_UP_MS)) { stopAll(); return; }
        setLock(LOCK_UP_OUT);
        rightUpC.set(1.0);
        rightDownC.set(1.0);
        leftUpC.set(1.0);
        leftDownC.set(1.0);
    }
    public void climbDown(){
        if (!withinTime(-1, CLIMB_DOWN_MS)) { stopAll(); return; }
        setLock(LOCK_DOWN_OUT);
        rightUpC.set(0.0);
        rightDownC.set(0.0);
        leftUpC.set(0.0);
        leftDownC.set(0.0);
    }
    public void climbStop(){
        stopAll();
    }
    private void stopAll(){
        setLock(LOCK_STOP);
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
