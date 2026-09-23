// ============================================================================
// 最終変更: 2026-09-23 15:18:10 JST (UTC+09:00)
// 変更内容: Blue計測専用パッケージへ隔離。通常Blue/Redの依存クラスとは共有しない。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode.timing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

// ポテンショ角度のPID+重力/摩擦補償。角度deg、速度deg/s、dt秒。
public class ExtendController {
    public DcMotorEx extend;

    private CachedPower extendOut;
    public AnalogInput pote;

    // ポテンショ換算のオフセットと機構の目標位置。機械変更時は校正する。
    double pote_offset = 21;

    double pote_Middle = 10;
    double pote_Up = 76;
    double pote_Down = -16;
    double targetAngle = 76;
    double current;

    double integral = 0;
    double kp = 0.0175;
    double ki = 0.005;
    double kd = 0.000;
    double output;
    double error;
    double prevError = 0;
    boolean reverse = false;

    private static final double TARGET_MIN_DEG = -20.0;
    private static final double TARGET_MAX_DEG = 120.0;

    // 重力FFと摩擦FFを使用。初動の100%上書きは削除済み。
    public boolean ffEnabled = true;

    private static final double KG_AMP   = 0.0638;
    private static final double KG_CONST = 0.0694;
    private static final double KS_FF    = 0.123;

    private static final double FF_DEADBAND_DEG = 2.0;

    // 目標方向へ十分動いているときは摩擦FFを切り、加速し過ぎを防ぐ。
    private static final double KS_RATE_CUT_DPS = 15.0;

    private static final double VEL_ALPHA = 0.35;

    private final double gravity_offset_angle = 17.0;
    private double angle = 0;

    public double ffOut = 0;
    public double pidOut = 0;

    private static final double TARGET_SLEW_DPS      = 50.0;
    private static final double TARGET_SLEW_FAST_DPS = 100.0;
    private double slewRate   = TARGET_SLEW_DPS;

    private static final double EXTEND_UP_DPS = 50.0;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double dtActual = 0.020;
    private static final double DT_MIN = 0.002;
    private static final double DT_MAX = 0.100;
    private double targetGoal = 80;
    private boolean slewActive = false;

    // falseでは初期目標76degへ向かう。trueなら初回の現在角度を保持する。
    private static final boolean SNAP_TARGET_ON_FIRST_LOOP = false;
    private boolean targetInitialized = false;

    private static final double STUCK_RANGE_MIN = 15.0;
    private static final double STUCK_RANGE_MAX = 70.0;

    private static final double STUCK_RATE_DPS  = 5.0;
    private static final long   STUCK_CHECK_MS  = 100;
    private static final long   STUCK_TIME_MS   = 500;
    private static final long   REVERSE_TIME_MS = 500;

    private static final double REVERSE_DUTY   = 0.55;

    // 0は電流制限無効。センサー異常・押し込み過ぎ保護とは独立。
    public double currentLimit = 0.0;
    private static final long   CUR_READ_MS = 200;
    private long   lastCurReadMs = 0;
    private double lastMotorCur  = 0.0;
    private double curScale      = 1.0;

    public double getMotorCurrent() { return lastMotorCur; }

    private static final double STUCK_ERROR_MIN = 3.0;

    private double  prevPoteValue     = 0.0;
    private long    prevStuckCheckMs  = 0;
    private double  prevStuckAngle    = 0.0;

    public  double  poteRate          = 0.0;

    public  double  poteVel           = 0.0;
    private double  prevAngleForVel   = 0.0;
    private boolean velInitialized    = false;
    private long    poteStuckStart    = 0;
    private long    stuckReverseStart = 0;
    public  boolean isStuck           = false;

    // 最近詰まり、シューター速度が不足した場合にメインが設定する。
    public  boolean yieldCurrent      = false;

    public  boolean yielding          = false;

    private long    lastJamMs         = 0;

    public  long    jamMemoryMs       = 1500;

    public boolean isJamRecent(){
        return lastJamMs != 0 && System.currentTimeMillis() - lastJamMs < jamMemoryMs;
    }

    // 校正範囲外のポテンショ値で出力を停止する。
    private static final double POTE_ANGLE_MIN = -20.8;
    private static final double POTE_ANGLE_MAX = 120.0;
    public boolean poteFault = false;

    // 収納側目標で、一定出力なのに1deg未満の動きが1500ms続くと停止保護をラッチする。
    private static final double STALL_DUTY_MIN   = 0.25;
    private static final double STALL_MOVE_DEG   = 1.0;
    private static final long   STALL_TIME_MS    = 1500;

    private static final double STALL_TARGET_MIN_DEG = 50.0;
    private double stallRefAngle = 0;
    private long   stallSince    = 0;
    public boolean stallFault    = false;

    private double stallStartAngle = 0.0;

    private final SensorReads reads;

    public ExtendController(RobotHardware hr){
        reads = hr.reads;
        extend = hr.extendMotor;
        extendOut = new CachedPower(extend, 0.005);
        pote   = hr.potentiometer;
    }

    public double readAngle(){
        return reads.angle(pote, pote_offset);
    }

    private static double clampTarget(double deg){
        return Math.max(TARGET_MIN_DEG, Math.min(TARGET_MAX_DEG, deg));
    }

    public void setTarget(double deg){
        setTargetSlew(deg, TARGET_SLEW_DPS);
    }

    public void setTargetFast(double deg){
        setTargetSlew(deg, TARGET_SLEW_FAST_DPS);
    }

    // 最終目標へ指定deg/sで追従する。
    public void setTargetSlew(double deg, double slewDegPerSec){
        targetGoal  = clampTarget(deg);
        slewRate    = Math.max(1.0, slewDegPerSec);
        slewActive  = true;
        commandDestination(targetGoal);
    }

    public void snapTarget(double deg){
        targetAngle = clampTarget(deg);
        targetGoal  = targetAngle;
        slewActive  = false;
        commandDestination(targetGoal);
    }

    public void extendDown(){
        targetAngle = pote_Down;
        targetGoal  = targetAngle;
        slewActive  = false;
        commandDestination(pote_Down);
    }
    public void extendMiddle(){
        targetAngle = pote_Middle;
        targetGoal  = targetAngle;
        slewActive  = false;
        commandDestination(pote_Middle);
    }

    // 収納目標を即時設定する。出力自体は通常のPID+FFで決まる。
    public void extendUpNow(){
        targetAngle = pote_Up;
        targetGoal  = targetAngle;
        slewActive  = false;
        commandDestination(pote_Up);
    }

    public void extendUp(){
        if (!isStuck && targetAngle < pote_Up) {

            targetAngle = Math.min(pote_Up, targetAngle + EXTEND_UP_DPS * dtActual);
            targetGoal  = targetAngle;
            slewActive  = false;

            commandDestination(pote_Up);
        }
    }

    private void clearFaults(){
        stallFault = false;
        stallSince = 0;
    }

    // 右バンパーの押し直しなど、明示的な再試行で停止保護を解除する。
    public void clearStallFault(){
        clearFaults();
    }

    private double lastGoalForFault = Double.NaN;

    // 同じ行き先の連続指令では積分と保護を維持。行き先が0.5deg超変わったときだけリセット。
    private void commandDestination(double dest){

        if (Double.isNaN(lastGoalForFault) || Math.abs(dest - lastGoalForFault) > 0.5) {
            clearFaults();
            integral = 0.0;
            prevError = targetAngle - current;
            lastGoalForFault = dest;
        }
    }

    // 目標の移動量は指定速度×実測dt。
    private void slewTarget(){
        if (!slewActive) return;

        double step = slewRate * dtActual;
        double d = targetGoal - targetAngle;
        if (Math.abs(d) <= step) {
            targetAngle = targetGoal;
            slewActive  = false;
        } else {
            targetAngle += (d > 0 ? step : -step);
        }
    }

    private double gravityFF(double angleDeg){
        return KG_AMP * Math.cos(Math.toRadians(angleDeg)) + KG_CONST;
    }

    private double frictionFF(double error){
        if (poteVel * Math.signum(error) > KS_RATE_CUT_DPS) return 0.0;
        double x = error / FF_DEADBAND_DEG;
        if (x >  1.0) x =  1.0;
        if (x < -1.0) x = -1.0;
        return KS_FF * x;
    }

    // 保護停止はキャッシュの許容差を迂回し、必ず0を送る。
    private void stopOutput(){
        ffOut  = 0;
        pidOut = 0;
        output = 0;
        integral = 0;
        prevError = 0;
        extendOut.setNow(0.0);
    }

    public double getSkipRate(){ return extendOut == null ? 0.0 : extendOut.getSkipRate(); }

    // 角度取得→異常/逆転/電力譲渡→詰まり判定→PID+FF→出力の順。
    public void extend_loop(){

        yielding = false;

        double measured = loopTimer.seconds();
        loopTimer.reset();
        if (measured < DT_MIN || measured > DT_MAX) {
            dtActual = (measured > DT_MAX) ? DT_MAX : 0.020;
        } else {
            dtActual = measured;
        }

        current = reads.angle(pote, pote_offset);
        angle   = current + gravity_offset_angle;

        if (!velInitialized) {
            prevAngleForVel = current;
            poteVel = 0.0;
            velInitialized = true;
        } else {
            double velRaw = (current - prevAngleForVel) / dtActual;
            poteVel += VEL_ALPHA * (velRaw - poteVel);
            prevAngleForVel = current;
        }

        poteFault = (current < POTE_ANGLE_MIN) || (current > POTE_ANGLE_MAX);
        if (poteFault) {
            stopOutput();
            return;
        }

        if (SNAP_TARGET_ON_FIRST_LOOP && !targetInitialized) {
            targetInitialized = true;
            targetAngle   = clampTarget(current);
            targetGoal    = targetAngle;
            slewActive    = false;
            integral      = 0;
            prevError     = 0;
            prevPoteValue = current;
            stallRefAngle = current;
        }

        // 詰まり後は500ms逆転する。逆転処理は電力譲渡より優先。
        if (isStuck) {
            if (System.currentTimeMillis() - stuckReverseStart < REVERSE_TIME_MS) {
                reverse = true;
                ffOut  = 0;
                pidOut = -REVERSE_DUTY;
                output = -REVERSE_DUTY;
                extendOut.set(output);
                integral  = 0;
                prevError = 0;
                return;
            } else {

                reverse = false;
                isStuck        = false;
                poteStuckStart = 0;
                prevPoteValue  = current;

                prevStuckCheckMs = System.currentTimeMillis();
                prevStuckAngle   = current;
            }
        }

        // 電力譲渡中は出力と積分をリセットし、停止を新しい詰まりと誤認しない。
        if (yieldCurrent) {
            yielding = true;
            stopOutput();
            integral  = 0;
            prevError = 0;
            poteStuckStart   = 0;
            prevStuckCheckMs = System.currentTimeMillis();
            prevStuckAngle   = current;
            prevPoteValue    = current;
            stallSince       = 0;
            stallRefAngle    = current;
            stallStartAngle  = current;
            return;
        }

        slewTarget();

        long nowMs = System.currentTimeMillis();
        if (prevStuckCheckMs == 0) {
            prevStuckCheckMs = nowMs;
            prevStuckAngle   = current;
        }
        // 100msごとに移動速度を確認。角度範囲と位置誤差の条件付きで詰まりを検知。
        if (nowMs - prevStuckCheckMs >= STUCK_CHECK_MS) {
            double elapsed = (nowMs - prevStuckCheckMs) / 1000.0;
            poteRate = Math.abs(current - prevStuckAngle) / elapsed;

            boolean tryingToMove = Math.abs(targetAngle - current) > STUCK_ERROR_MIN;
            boolean inRange = STUCK_RANGE_MIN <= current && current <= STUCK_RANGE_MAX;

            if (tryingToMove && inRange && poteRate < STUCK_RATE_DPS) {
                if (poteStuckStart == 0) {
                    poteStuckStart = nowMs;
                } else if (nowMs - poteStuckStart > STUCK_TIME_MS) {

                    isStuck           = true;
                    lastJamMs         = nowMs;
                    stuckReverseStart = nowMs;
                    poteStuckStart    = 0;
                    prevStuckCheckMs  = nowMs;
                    prevStuckAngle    = current;
                    stopOutput();
                    return;
                }
            } else {
                poteStuckStart = 0;
            }
            prevStuckCheckMs = nowMs;
            prevStuckAngle   = current;
            prevPoteValue    = current;
        }

        if (Math.abs(current - stallRefAngle) > STALL_MOVE_DEG) {
            stallRefAngle = current;
            stallSince    = 0;
        }

if (targetAngle >= STALL_TARGET_MIN_DEG) {

    if (Math.abs(output) >= STALL_DUTY_MIN) {
        if (stallSince == 0) {
            stallSince      = System.currentTimeMillis();
            stallStartAngle = current;
        } else if (Math.abs(current - stallStartAngle) >= STALL_MOVE_DEG) {

            stallSince      = System.currentTimeMillis();
            stallStartAngle = current;
        } else if (System.currentTimeMillis() - stallSince > STALL_TIME_MS) {
            stallFault = true;
            lastJamMs  = System.currentTimeMillis();
        }
    } else {
        stallSince = 0;
    }

    if (stallFault) {
        stopOutput();
        return;
    }

} else {

    stallSince = 0;
}

        error = targetAngle - current;
        integral += error * dtActual;
        // 積分を±0.5へ制限する。現在ki=0.005なので出力寄与は最大±0.0025。
        integral  = Math.min(0.5, Math.max(integral, -0.5));
        double deriv = (error - prevError) / dtActual;

        ffOut  = ffEnabled ? (gravityFF(angle) + frictionFF(error)) : 0.0;
        pidOut = kp * error + ki * integral + kd * deriv;

        output = ffOut + pidOut;
        output = Math.max(-1.0, Math.min(1.0, output));

        if (currentLimit > 0) {
    long nowCur = System.currentTimeMillis();
    if (nowCur - lastCurReadMs >= CUR_READ_MS) {
        lastCurReadMs = nowCur;
        try { lastMotorCur = reads.current(extend, SensorTiming.OPTIONAL_CURRENT); }
        catch (Exception ignored) { }
        if (lastMotorCur > currentLimit) curScale -= 0.06;
        else if (curScale < 1.0)         curScale += 0.03;
        if (curScale < 0.4) curScale = 0.4;
        if (curScale > 1.0) curScale = 1.0;
    }
    output *= curScale;
}

extendOut.set(output);
prevError = error;
    }
}
