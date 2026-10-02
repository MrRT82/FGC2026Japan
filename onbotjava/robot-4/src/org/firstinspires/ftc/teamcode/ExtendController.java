// ============================================================================
// 最終変更: 2026-10-02 JST
// 変更内容: 四号機の方向別PIDFと非対称出力スルーを通常Blue/Redへ統合。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode;

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
    double pote_offset = 0;

    double pote_Middle = 38;
    double pote_Up = 75;
    double pote_Down = 1;
    double targetAngle = 75;
    double current;

    double integral = 0;
    private static final double KP_UP = 0.07, KI_UP = 0.005, KD_UP = 0.0015;
    private static final double KP_DOWN = 0.07, KI_DOWN = 0.005, KD_DOWN = 0.0025;
    double kp = KP_UP;
    double ki = KI_UP;
    double kd = KD_UP;
    double output;
    double error;
    double prevError = 0;
    boolean reverse = false;

    private static final double TARGET_MIN_DEG = -2;
    private static final double TARGET_MAX_DEG = 88;

    // ManualTuneと同じ12.6V基準のduty値。実測電圧による追加補正は行わない。
    public boolean ffEnabled = true;

    private static final double FF_REFERENCE_V = 12.6;
    private static final double KG = 1.9969002748 / FF_REFERENCE_V;
    private static final double KV = 0.0404679237 / FF_REFERENCE_V;
    private static final double FC_UP = 0.9482379897 / FF_REFERENCE_V;
    private static final double FC_DOWN = 1.9725096967 / FF_REFERENCE_V;
    private static final double FS_UP = 0.9624524741 / FF_REFERENCE_V;
    private static final double FS_DOWN = 2.5390997778 / FF_REFERENCE_V;

    private static final double FF_DEADBAND_DEG = 2.0;
    private static final double STATIC_TARGET_VELOCITY_DPS = 0.5;
    private static final double DRIVE_SLEW_PER_20MS = 0.03;
    private static final double BRAKE_SLEW_PER_20MS = 0.08;
    private static final double MOTION_THRESHOLD_DPS = 2.0;

    private static final double VEL_ALPHA = 0.35;

    // Robot 4 survey: raw sensor angle 20 deg is physical horizontal (gravity angle 0 deg).
    private final double gravity_offset_angle = -20.0;
    private double angle = 0;

    public double ffOut = 0;
    public double pidOut = 0;
    public double targetVelocity = 0;
    public double requestedOutput = 0;
    public double slewLimit = 0;
    public int pidDirection = 0; // 保持中は最後の目標プロファイル方向を維持。

    private static final double TARGET_SLEW_DPS      = 25.0;
    private static final double TARGET_SLEW_FAST_DPS = 100.0;
    private static final double PROFILE_ACCEL_DPS2 = 100.0;
    private double slewRate   = TARGET_SLEW_DPS;
    private double profileVelocity = 0;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double dtActual = 0.020;
    private static final double DT_MIN = 0.002;
    private static final double DT_MAX = 0.100;
    private double targetGoal = 75;
    private boolean slewActive = false;
    private boolean targetInitialized = false;
    private boolean profileNeedsSync = true;

    private static final double STUCK_RANGE_MIN = 15.0;
    private static final double STUCK_RANGE_MAX = 70.0;

    private static final double STUCK_RATE_DPS  = 5.0;
    private static final long   STUCK_CHECK_MS  = 100;
    private static final long   STUCK_TIME_MS   = 500;
    private static final long   REVERSE_TIME_MS = 1000;

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
    private static final double POTE_ANGLE_MIN = -5;
    private static final double POTE_ANGLE_MAX = 90;
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

    public ExtendController(RobotHardware hr){
        extend = hr.extendMotor;
        extendOut = new CachedPower(extend, 0.005);
        pote   = hr.potentiometer;
    }

    public double readAngle(){
        double voltage = pote.getVoltage();
        double maxVoltage = pote.getMaxVoltage();
        if (!finite(voltage) || !finite(maxVoltage) || maxVoltage <= 0) return Double.NaN;
        return voltage / maxVoltage * 135.0 - pote_offset;
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
        if (!finite(deg) || !finite(slewDegPerSec)) {
            throw new IllegalArgumentException("Non-finite extension target/rate");
        }
        targetGoal  = clampTarget(deg);
        slewRate    = Math.max(1.0, slewDegPerSec);
        slewActive  = true;
        commandDestination(targetGoal);
    }

    public void snapTarget(double deg){
        if (!finite(deg)) throw new IllegalArgumentException("Non-finite extension target");
        targetAngle = clampTarget(deg);
        targetGoal  = targetAngle;
        slewActive  = false;
        profileVelocity = targetVelocity = 0;
        commandDestination(targetGoal);
    }

    public void extendDown(){
        setTarget(pote_Down);
    }
    public void extendMiddle(){
        setTarget(pote_Middle);
    }

    // 既存呼出名を維持。収納の最終目標を設定し、25deg/s・100deg/s^2で追従する。
    public void extendUpNow(){
        setTarget(pote_Up);
    }

    public void extendUp(){
        if (!isStuck) setTarget(pote_Up);
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

    // 目標までの残距離で減速し、到達時は目標を越えない。
    private void slewTarget(){
        targetVelocity = 0;
        if (!slewActive) return;
        double oldTarget = targetAngle;
        double remaining = targetGoal - oldTarget;
        double maxDelta = PROFILE_ACCEL_DPS2 * dtActual;
        // 毎周期の速度を積分する離散系の停止距離。sqrt(2*a*d)だけでは
        // 最後の周期に減速が間に合わず、目標到達時の速度が急変する。
        double fullSteps = Math.floor((Math.sqrt(1.0
                + 8.0 * Math.abs(remaining) / (maxDelta * dtActual)) - 1.0) / 2.0);
        double arrivalSpeed = Math.abs(remaining) / ((fullSteps + 1.0) * dtActual)
                + maxDelta * fullSteps / 2.0;
        double desiredVelocity = Math.signum(remaining)
                * Math.min(slewRate, arrivalSpeed);
        profileVelocity += Math.max(-maxDelta, Math.min(maxDelta, desiredVelocity - profileVelocity));
        double step = profileVelocity * dtActual;
        if (remaining == 0 || (step * remaining > 0 && Math.abs(step) >= Math.abs(remaining))) {
            targetAngle = targetGoal;
            profileVelocity = 0;
            slewActive = false;
        } else {
            // 初期実角度が目標範囲外でも、一度に境界へ飛ばさず範囲内へ戻る。
            double low = Math.min(TARGET_MIN_DEG, oldTarget);
            double high = Math.max(TARGET_MAX_DEG, oldTarget);
            targetAngle = Math.max(low, Math.min(high, oldTarget + step));
            if (targetAngle != oldTarget + step) profileVelocity = 0;
        }
        targetVelocity = (targetAngle - oldTarget) / dtActual;
    }

    private static boolean finite(double value){
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    // 逆転/停止から復帰するときは最終行き先を保ち、現在位置から再出発する。
    private void syncProfile(){
        targetAngle = current;
        profileVelocity = targetVelocity = integral = prevError = 0;
        slewActive = targetGoal != targetAngle;
        profileNeedsSync = false;
        targetInitialized = true;
        stallSince = poteStuckStart = 0;
        stallRefAngle = stallStartAngle = prevStuckAngle = current;
        prevStuckCheckMs = System.currentTimeMillis();
    }

    private double gravityFF(double angleDeg){
        return KG * Math.cos(Math.toRadians(angleDeg));
    }

    private double frictionFF(double error){
        boolean moving = Math.abs(targetVelocity) >= STATIC_TARGET_VELOCITY_DPS;
        double direction = Math.signum(moving ? targetVelocity : error);
        double magnitude = direction >= 0 ? (moving ? FC_UP : FS_UP)
                : (moving ? FC_DOWN : FS_DOWN);
        double fade = moving ? 1.0 : Math.min(1.0, Math.abs(error) / FF_DEADBAND_DEG);
        return direction * magnitude * fade;
    }

    private void selectPidDirection(){
        int direction = targetVelocity > 0 ? 1 : targetVelocity < 0 ? -1 : pidDirection;
        if (direction != pidDirection) integral = 0;
        pidDirection = direction;
        kp = direction < 0 ? KP_DOWN : KP_UP;
        ki = direction < 0 ? KI_DOWN : KI_UP;
        kd = direction < 0 ? KD_DOWN : KD_UP;
    }

    // 本番25ms周期を維持。20ms基準の変化率を実測dtへ換算する。
    private double slewOutput(double requested){
        double desired = Math.max(-1.0, Math.min(1.0, requested));
        // キャッシュが送信を省略した場合も、最後に実際に送った出力を基準にする。
        double previousSent = extendOut.getLast();
        double delta = desired - previousSent;
        boolean braking = Math.abs(poteVel) >= MOTION_THRESHOLD_DPS && delta * poteVel < 0;
        slewLimit = (braking ? BRAKE_SLEW_PER_20MS : DRIVE_SLEW_PER_20MS) * dtActual / 0.020;
        return Math.max(previousSent - slewLimit, Math.min(previousSent + slewLimit, desired));
    }

    // 保護停止はキャッシュの許容差を迂回し、必ず0を送る。
    private void stopOutput(){
        ffOut  = 0;
        pidOut = 0;
        output = 0;
        integral = 0;
        prevError = 0;
        requestedOutput = slewLimit = targetVelocity = profileVelocity = 0;
        profileNeedsSync = true;
        extendOut.setNow(0.0);
    }

    // OpMode停止・非常停止/deadmanを持つ呼出元向け。センサーやスルーを介さず0。
    public void stop(){
        stopOutput();
        isStuck = reverse = false;
        poteStuckStart = 0;
    }


    public double getSkipRate(){ return extendOut == null ? 0.0 : extendOut.getSkipRate(); }

    // 角度取得→異常/逆転/電力譲渡→詰まり判定→PID+FF→出力の順。
    public void extend_loop(){

        yielding = false;

        double measured = loopTimer.seconds();
        loopTimer.reset();
        if (!targetInitialized) {
            // INIT/START待ち時間を初回のプロファイル・出力スルーへ加算しない。
            dtActual = 0.025;
        } else if (measured < DT_MIN || measured > DT_MAX) {
            dtActual = (measured > DT_MAX) ? DT_MAX : 0.020;
        } else {
            dtActual = measured;
        }

        try {
            current = readAngle();
        } catch (RuntimeException e) {
            poteFault = true;
            velInitialized = false;
            stopOutput();
            throw e;
        }
        angle   = current + gravity_offset_angle;

        // 無効値をEMAへ入れず、回復時もNaNを残さない。
        poteFault = !finite(current) || current < POTE_ANGLE_MIN || current > POTE_ANGLE_MAX;
        if (poteFault) {
            velInitialized = false;
            stopOutput();
            return;
        }

        if (!velInitialized) {
            prevAngleForVel = current;
            poteVel = 0.0;
            velInitialized = true;
        } else {
            double velRaw = (current - prevAngleForVel) / dtActual;
            poteVel += VEL_ALPHA * (velRaw - poteVel);
            prevAngleForVel = current;
        }

        // 詰まり後は従来どおり1000ms、-0.55で逆転。PIDF/スルーは迂回する。
        if (isStuck) {
            if (System.currentTimeMillis() - stuckReverseStart < REVERSE_TIME_MS) {
                reverse = true;
                ffOut  = 0;
                pidOut = -REVERSE_DUTY;
                output = -REVERSE_DUTY;
                extendOut.set(output);
                requestedOutput = output;
                targetVelocity = profileVelocity = slewLimit = 0;
                profileNeedsSync = true;
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
                profileNeedsSync = true;
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

        // 停止ラッチ中はプロファイルを進めず、0出力を維持する。
        if (stallFault) {
            stopOutput();
            return;
        }
        if (!targetInitialized || profileNeedsSync) syncProfile();
        slewTarget();
        selectPidDirection();

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
                    stallSince = System.currentTimeMillis();
                    stallStartAngle = current;
                } else if (Math.abs(current - stallStartAngle) >= STALL_MOVE_DEG) {
                    stallSince = System.currentTimeMillis();
                    stallStartAngle = current;
                } else if (System.currentTimeMillis() - stallSince > STALL_TIME_MS) {
                    stallFault = true;
                    lastJamMs = System.currentTimeMillis();
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
        double deriv = targetVelocity - poteVel;

        ffOut  = ffEnabled ? (gravityFF(angle) + KV * targetVelocity + frictionFF(error)) : 0.0;
        pidOut = kp * error + ki * integral + kd * deriv;

        requestedOutput = ffOut + pidOut;
        output = Math.max(-1.0, Math.min(1.0, requestedOutput));

        if (currentLimit > 0) {
            long nowCur = System.currentTimeMillis();
            if (nowCur - lastCurReadMs >= CUR_READ_MS) {
                lastCurReadMs = nowCur;
                try { lastMotorCur = extend.getCurrent(CurrentUnit.AMPS); }
                catch (Exception ignored) { }
                if (lastMotorCur > currentLimit) curScale -= 0.06;
                else if (curScale < 1.0) curScale += 0.03;
                if (curScale < 0.4) curScale = 0.4;
                if (curScale > 1.0) curScale = 1.0;
            }
            output *= curScale;
        }

        if (!finite(output)) {
            stopOutput();
            return;
        }
        output = slewOutput(output);
        extendOut.set(output);
        output = extendOut.getLast();
        prevError = error;
    }
}
