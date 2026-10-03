// ============================================================================
// 最終変更: 2026-10-03 JST (UTC+09:00)
// 変更内容: Blue計測専用パッケージへ隔離(通常Blue/Redとは共有しない)。上昇10.5秒(ロックのみ)→下降7秒(登坂とロック)→下降開始6秒後にローラー全力→押すごとに停止(P保持)/回転。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode.timing;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

// 登坂サーボ・ロック・ローラーの出力を、段階(ClimbSequence)と経過時間から決める。
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
    // ローラー: 回す時は全力(±1.0、電池電圧をそのままかける)。止める時は止めた瞬間の
    // エンコーダー位置を目標にP制御で保持する。ROLLER_KPとROLLER_HOLD_MAXは仮値(実機で調整)。
    public static final double ROLLER_KP = 0.005;       // 出力 / エンコーダー1カウント
    public static final double ROLLER_HOLD_MAX = 1.0;
    private double rollerFullPower = 0.0;   // 向き込みの全力出力。最初に回した瞬間に決め、以後固定。
    private boolean rollerDirDecided = false;
    private boolean holding = false;
    private int holdTarget = 0, rollerPosition = 0;
    private double rollerCmd = 0.0;
    private double downStartMs = Double.NaN;

    // 毎ループ1回呼ぶ。stageとstageElapsedMsはClimbSequence、headingはIMU方位[deg]。
    // frontRollerPowerは表側のローラー出力(Blue +1.0、Red -1.0)。裏側はその逆。
    // 表裏は最初にローラーを回す瞬間の方位で決める: 登坂の基準方位から±90°以内なら表側。
    public void update(int stage, double nowMs, double stageElapsedMs, double heading,
                       double climbTarget, double frontRollerPower){
        // 下降はDOWN_MSのあいだ続ける。6秒でローラーへ進み、その後に停止へ進んでも7秒までは下降する。
        if (stage == ClimbSequence.DOWN && Double.isNaN(downStartMs)) downStartMs = nowMs;
        boolean descending = !Double.isNaN(downStartMs) && nowMs - downStartMs < ClimbSequence.DOWN_MS;
        if (stage == ClimbSequence.UP && stageElapsedMs < ClimbSequence.UP_MS) servosUp();
        else if (descending) servosDown();
        else servosStop();

        rollerPosition = Climb_Roller.getCurrentPosition();
        boolean spin = stage == ClimbSequence.ROLL
                || (stage == ClimbSequence.DOWN && stageElapsedMs >= ClimbSequence.ROLLER_DELAY_MS);
        if (spin) {
            if (!rollerDirDecided) {
                boolean front = Math.abs(normalize(heading - climbTarget)) <= 90.0;
                rollerFullPower = front ? frontRollerPower : -frontRollerPower;
                rollerDirDecided = true;
            }
            holding = false;
            rollerCmd = rollerFullPower;
        } else if (stage == ClimbSequence.HOLD) {
            if (!holding) { holding = true; holdTarget = rollerPosition; }
            rollerCmd = Math.max(-ROLLER_HOLD_MAX,
                    Math.min(ROLLER_HOLD_MAX, ROLLER_KP * (holdTarget - rollerPosition)));
        } else {
            holding = false;
            rollerCmd = 0.0;
        }
        rollerOut.set(rollerCmd);
    }
    public double getRollerCommand(){ return rollerCmd; }
    public int getRollerPosition(){ return rollerPosition; }
    public boolean isHolding(){ return holding; }
    public int getHoldTarget(){ return holdTarget; }
    // 最初の回転で決めた全力出力の符号(+1/-1)。まだ決まっていなければ0。
    public int getRollerDirection(){ return rollerDirDecided ? (int) Math.signum(rollerFullPower) : 0; }

    private static double normalize(double deg){
        double a = deg % 360.0;
        if (a > 180.0) a -= 360.0;
        if (a < -180.0) a += 360.0;
        return a;
    }
    // 上昇はロックだけを全速で回し、登坂サーボは止める。
    private void servosUp(){
        setLock(LOCK_UP_OUT);
        setClimbServos(0.5);
    }
    // 下降は登坂サーボもロックも全速(上昇と逆向き)。
    private void servosDown(){
        setLock(LOCK_DOWN_OUT);
        setClimbServos(0.0);
    }
    private void servosStop(){
        setLock(LOCK_STOP);
        setClimbServos(0.5);
    }
    private void setClimbServos(double out){
        rightUpC.set(out);
        rightDownC.set(out);
        leftUpC.set(out);
        leftDownC.set(out);
    }    // 左右いずれかの下端センサーで下降を止める。
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
