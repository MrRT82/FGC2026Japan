// ============================================================================
// 最終変更: 2026-10-02 JST (UTC+09:00)
// 変更内容: Blue計測専用パッケージへ隔離(通常Blue/Redとは共有しない)。上昇はロックのみ全速、下降は登坂とロックを全速。上昇・下降を時間で停止。下降後に進入方位でローラーを回す。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode.timing;

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

    private final SensorReads reads;

    public ClimbController(RobotHardware hr) {
        reads = hr.reads;
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
    // 速さは0.5からの差で0〜0.5(0.5が全速)。上昇・下降とも全速で回す。
    // 回す向きが逆なら、LOCK_DIRを-1にする。
    private static final double LOCK_STOP = 0.5;
    private static final double LOCK_UP_SPEED = 0.5;
    private static final double LOCK_DOWN_SPEED = 0.5;
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
        // 上昇中はロックだけを回し、登坂サーボは止める。
        setLock(LOCK_UP_OUT);
        rightUpC.set(0.5);
        rightDownC.set(0.5);
        leftUpC.set(0.5);
        leftDownC.set(0.5);
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
        rollerOut.set(0.0);
    }

    // 下降が終わったら(時間切れ、または下端センサー)ローラーを回し始める。
    // 向きは下降終了時の方位で決める: 登坂の基準方位から±90°以内なら表側。
    // frontRollerPowerは表側の出力(Blue +1.0、Red -1.0)。裏側はその逆。
    // ローラーは次のボタン(ClimbSequenceのDONE)でclimbStop()が呼ばれるまで回り続ける。
    private boolean downFinished = false;
    private double rollerPower = 0.0;
    public void climbDownThenRoller(double heading, double climbTarget, double frontRollerPower){
        if (!downFinished) {
            if (!isBottomLimit() && withinTime(-1, CLIMB_DOWN_MS)) { climbDown(); return; }
            downFinished = true;
            boolean front = Math.abs(normalize(heading - climbTarget)) <= 90.0;
            rollerPower = front ? frontRollerPower : -frontRollerPower;
        }
        stopAll();
        rollerOut.set(rollerPower);
    }
    public double getRollerPower(){ return downFinished ? rollerPower : 0.0; }
    private static double normalize(double deg){
        double a = deg % 360.0;
        if (a > 180.0) a -= 360.0;
        if (a < -180.0) a += 360.0;
        return a;
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
        return (reads.touch(bottomSensor) || reads.touch(BottomSensor));
    }

    public void Climb_Roller_Forward() {
        rollerOut.set(1.0);
    }

    public void Climb_Roller_Reverse() {
        rollerOut.set(-1.0);
    }
}
