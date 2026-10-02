// ============================================================================
// 最終変更: 2026-10-02 JST
// 変更内容: 撤去した測距センサーへの依存と測距処理を削除。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode.timing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;

// 入力・制御状態・速度・電流・処理時間をCSVへ記録する。
public class RobotLogger {

    private static final String DIR = "/sdcard/FIRST/logs/";

    // 記録50ms、flush1000ms。書き込みは現在メインスレッド上で行う。
    private static final long LOG_INTERVAL_MS = 50;

    private static final long FLUSH_INTERVAL_MS = 1000;

    private static final long CURRENT_INTERVAL_MS = 200;
    private int curIndex = 0;
    private double cVolt = 0.0;

    private final RobotHardware hw;
    private final HardwareMap hardwareMap;
    private final ImuController imu;
    private final ExtendController extend;
    private final ShooterController shooter;

    private IntakeController intake = null;

    private DriveAssist assist = null;

    private LynxModule[] hubs;
    private BufferedWriter out;
    private String path = "(未作成)";
    private long startMs = 0, lastLogMs = 0, lastFlushMs = 0, prevLogMs = 0;

    private double cDrvL = 0, cDrvR = 0, cShL = 0, cShR = 0;
    private double cInt = 0, cExt = 0, cFeed = 0, cClimb = 0, cHub = 0;
    private boolean failed = false;
    private PowerTrace powerTrace;
    private boolean powerFailed;
    private int rows = 0;

    public RobotLogger(RobotHardware hw, HardwareMap map, ImuController imu,
                       ExtendController extend, ShooterController shooter) {
        this.hw = hw;
        this.hardwareMap = map;
        this.imu = imu;
        this.extend = extend;
        this.shooter = shooter;
    }

    public void setDriveAssist(DriveAssist a) { this.assist = a; }

    private double phBulk = 0, phImu = 0, phCtrl = 0, phMode = 0, phTelem = 0, phWait = 0;
    private double phLog = 0;

    private double periodMs = 0, periodMax = 0;
    private int    ovrCount = 0, loopCount = 0;

    // 区間時間はms。待機時間は前周期分、phLogは直前の記録処理時間。
    public void setPhases(double bulk, double imu, double ctrl,
                          double mode, double telem, double waitMs) {
        phBulk  = bulk;
        phImu   = imu;
        phCtrl  = ctrl;
        phMode  = mode;
        phTelem = telem;
        phWait  = waitMs;
    }

    public void setPeriod(double period, int overruns, int loops) {
        periodMs  = period;
        if (period > periodMax) periodMax = period;
        ovrCount  = overruns;
        loopCount = loops;
    }

    private double pmLogic = 0, pmImu = 0, pmCalc = 0, pmWrite = 0;

    public void setModeParts(double logic, double imu, double calc, double write) {
        pmLogic = logic;
        pmImu   = imu;
        pmCalc  = calc;
        pmWrite = write;
    }

    // 10項目を1周期1項目ずつ取得。各値の取得時刻は異なり、同時測定ではない。
    public void readOneCurrent() {
        if (failed) return;
        switch (curIndex) {
            case 0: cDrvL  = cur(hw.leftMotor, SensorTiming.DIAG_DRIVE_L);         break;
            case 1: cDrvR  = cur(hw.rightMotor, SensorTiming.DIAG_DRIVE_R);        break;
            case 2: cShL   = cur(shooter.leftShooter, SensorTiming.DIAG_SHOOTER_L);  break;
            case 3: cShR   = cur(shooter.rightShooter, SensorTiming.DIAG_SHOOTER_R); break;
            case 4: cInt   = cur(hw.intakeMotor, SensorTiming.DIAG_INTAKE);       break;
            case 5: cExt   = cur(extend.extend, SensorTiming.DIAG_EXTEND);        break;
            case 6: cFeed  = curDc(hw.feedMotor);       break;
            case 7: cClimb = cur(hw.climbMotor, SensorTiming.DIAG_CLIMB);        break;
            case 8: cHub   = hubCurrent();              break;
            default: cVolt = voltage();                 break;
        }
        curIndex++;
        if (curIndex > 9) curIndex = 0;
    }

    public double getShooterCurL() { return cShL; }
    public double getShooterCurR() { return cShR; }

    public void setIntake(IntakeController i) {
        intake = i;
    }

    // ファイル名は秒単位の時刻。初期値として電流・電圧を取得する。
    public void init() {
        startMs = System.currentTimeMillis();
        lastLogMs = 0;
        lastFlushMs = startMs;
        prevLogMs = startMs;

        List<LynxModule> list = hardwareMap.getAll(LynxModule.class);
        hubs = list.toArray(new LynxModule[0]);

        try { powerTrace = new PowerTrace(new File(DIR)); }
        catch (Exception e) { powerFailed = true; }

        cDrvL  = cur(hw.leftMotor, SensorTiming.DIAG_DRIVE_L);
        cDrvR  = cur(hw.rightMotor, SensorTiming.DIAG_DRIVE_R);
        cShL   = cur(shooter.leftShooter, SensorTiming.DIAG_SHOOTER_L);
        cShR   = cur(shooter.rightShooter, SensorTiming.DIAG_SHOOTER_R);
        cInt   = cur(hw.intakeMotor, SensorTiming.DIAG_INTAKE);
        cExt   = cur(extend.extend, SensorTiming.DIAG_EXTEND);
        cFeed  = curDc(hw.feedMotor);
        cClimb = cur(hw.climbMotor, SensorTiming.DIAG_CLIMB);
        cHub   = hubCurrent();
        cVolt  = voltage();

        try {
            File dir = new File(DIR);
            if (!dir.exists()) dir.mkdirs();
            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            File f = new File(dir, "robot_blue_timing_" + stamp + ".csv");
            out = new BufferedWriter(new FileWriter(f), 16384);
            path = f.getAbsolutePath();
            out.write(csvHeader());
        } catch (Exception e) {
            failed = true;
            path = "ERROR: " + e.getMessage();
        }
    }

    static String csvHeader() {
        return "t_ms,loop_ms,"
                + "gp1_lx,gp1_ly,gp1_rx,gp1_lt,gp1_rt,gp1_btn,gp2_lt,gp2_btn,"
                + "shoot_mode,climb_mode,discharge_mode,climb_enum,"
                + "drv_l_out,drv_r_out,drv_l_vel,drv_r_vel,drv_l_cur,drv_r_cur,"
                + "imu_target,imu_cur,imu_err,imu_rate,imu_out,imu_p,imu_i,imu_d,imu_s,imu_dt_ms,imu_spikes,"
                + "ext_target,ext_cur,ext_vel,ext_out,ext_ff,ext_pid,ext_cur_A,ext_stuck,ext_reverse,"

                + "sht_target,sht_ramped,sht_vel,sht_vel_l,sht_vel_r,sht_out,"
                + "sht_err,sht_int,sht_ff,sht_dt,sht_rmping,"
                + "sht_cur_l,sht_cur_r,int_cur,int_vel,int_stall,hub_cur,"
                + "feed_cur,climb_cur,cur_sum,cur_other,"

                + "ph_bulk,ph_imu,ph_ctrl,ph_mode,ph_telem,ph_wait,ph_log,period_ms,period_max,ovr_pct,heap_kb,"
                + "pm_logic,pm_imu,pm_calc,pm_write,volt,"

                + "as_mode,as_head,as_tgt,as_rate,as_err,as_out,"

                + "build,"

                + "ext_stall,ext_pote,ext_yield"
                + SensorTiming.csvHeader() + "\n";
    }

    // 間引き後に1行記録。loop_msはログ行間隔、period_msが制御周期。
    public void update(Gamepad gp1, Gamepad gp2,
                       double driveLeft, double driveRight,
                       boolean shootMode, boolean climbMode, boolean dischargeMode,
                       int climbEnum) {
        if (failed || out == null) return;
        long now = System.currentTimeMillis();
        if (now - lastLogMs < LOG_INTERVAL_MS) return;

        long wt0 = System.nanoTime();

        long t = now - startMs;
        long loop = now - prevLogMs;
        lastLogMs = now;
        prevLogMs = now;

        StringBuilder sb = new StringBuilder(2048);
        sb.append(t).append(',').append(loop).append(',');

        ap2(sb, gp1.left_stick_x);  sb.append(',');
        ap2(sb, gp1.left_stick_y);  sb.append(',');
        ap2(sb, gp1.right_stick_x); sb.append(',');
        ap2(sb, gp1.left_trigger);  sb.append(',');
        ap2(sb, gp1.right_trigger); sb.append(',');
        sb.append(buttons(gp1)).append(',');
        ap2(sb, gp2.left_trigger);  sb.append(',');
        sb.append(buttons(gp2)).append(',');

        sb.append(shootMode ? 1 : 0).append(',')
          .append(climbMode ? 1 : 0).append(',')
          .append(dischargeMode ? 1 : 0).append(',')
          .append(climbEnum).append(',');

        ap3(sb, driveLeft);  sb.append(',');
        ap3(sb, driveRight); sb.append(',');
        ap1(sb, hw.reads.velocity(hw.leftMotor, SensorTiming.VELOCITY_DRIVE));  sb.append(',');
        ap1(sb, hw.reads.velocity(hw.rightMotor, SensorTiming.VELOCITY_DRIVE)); sb.append(',');
        ap2(sb, cDrvL); sb.append(',');
        ap2(sb, cDrvR); sb.append(',');

        ap2(sb, imu.getTarget());   sb.append(',');
        ap2(sb, imu.getCurrent());  sb.append(',');
        ap2(sb, imu.getError());    sb.append(',');
        ap1(sb, imu.getRate());     sb.append(',');
        ap3(sb, imu.getOutput());   sb.append(',');
        ap3(sb, imu.pTerm);         sb.append(',');
        ap3(sb, imu.iTerm);         sb.append(',');
        ap3(sb, imu.dTerm);         sb.append(',');
        ap3(sb, imu.sTerm);         sb.append(',');
        ap1(sb, imu.getDt() * 1000.0); sb.append(',');
        sb.append(imu.getRateSpikes()).append(',');

        ap2(sb, extend.targetAngle); sb.append(',');
        ap2(sb, extend.current);     sb.append(',');
        ap1(sb, extend.poteVel);     sb.append(',');
        ap3(sb, extend.output);      sb.append(',');
        ap3(sb, extend.ffOut);       sb.append(',');
        ap3(sb, extend.pidOut);      sb.append(',');

        ap2(sb, cExt);  sb.append(',');
        sb.append(extend.isStuck ? 1 : 0).append(',')
          .append(extend.reverse ? 1 : 0).append(',');

        ap1(sb, shooter.targetSpeed); sb.append(',');

        ap1(sb, shooter.getTargetRamped()); sb.append(',');

        double shVL = hw.reads.velocity(shooter.leftShooter, SensorTiming.VELOCITY_SHOOTER);
        double shVR = hw.reads.velocity(shooter.rightShooter, SensorTiming.VELOCITY_SHOOTER);
        ap1(sb, (shVL + shVR) / 2.0); sb.append(',');
        ap1(sb, shVL); sb.append(',');
        ap1(sb, shVR); sb.append(',');

        ap3(sb, shooter.lastCommanded); sb.append(',');

        ap1(sb, shooter.lastError);    sb.append(',');
        ap3(sb, shooter.lastIntegral); sb.append(',');
        ap3(sb, shooter.lastFF);       sb.append(',');
        ap3(sb, shooter.lastDt);       sb.append(',');
        sb.append(shooter.lastRamping ? 1 : 0).append(',');

        ap2(sb, cShL); sb.append(',');
        ap2(sb, cShR); sb.append(',');
        ap2(sb, cInt); sb.append(',');

        ap1(sb, hw.reads.velocity(hw.intakeMotor, SensorTiming.VELOCITY_INTAKE)); sb.append(',');
        sb.append(intake != null && intake.isStallBackoff() ? 1 : 0); sb.append(',');
        ap2(sb, cHub); sb.append(',');

        // 非同時サンプルの合計。瞬間的な電力収支の評価には使わない。
        double sum = cDrvL + cDrvR + cShL + cShR + cInt + cExt + cFeed + cClimb;
        ap2(sb, cFeed);         sb.append(',');
        ap2(sb, cClimb);        sb.append(',');
        ap2(sb, sum);           sb.append(',');
        ap2(sb, cHub - sum);    sb.append(',');


        ap2(sb, phBulk);  sb.append(',');
        ap2(sb, phImu);   sb.append(',');
        ap2(sb, phCtrl);  sb.append(',');
        ap2(sb, phMode);  sb.append(',');
        ap2(sb, phTelem); sb.append(',');
        ap2(sb, phWait);  sb.append(',');
        ap2(sb, phLog);   sb.append(',');
        ap2(sb, periodMs);  sb.append(',');
        ap2(sb, periodMax); sb.append(',');
        periodMax = 0;
        ap2(sb, loopCount > 0 ? 100.0 * ovrCount / loopCount : 0.0); sb.append(',');

        Runtime rt = Runtime.getRuntime();
        ap1(sb, (rt.totalMemory() - rt.freeMemory()) / 1024.0); sb.append(',');
        ap2(sb, pmLogic); sb.append(',');
        ap2(sb, pmImu);   sb.append(',');
        ap2(sb, pmCalc);  sb.append(',');
        ap2(sb, pmWrite); sb.append(',');
        ap2(sb, cVolt); sb.append(',');

        if (assist != null) {
            sb.append(assist.getModeCode()).append(',');
            ap2(sb, assist.getHeading());       sb.append(',');
            ap2(sb, assist.getTargetHeading()); sb.append(',');
            ap1(sb, assist.getRate());          sb.append(',');
            ap2(sb, assist.getError());         sb.append(',');
            ap3(sb, assist.getOutput());
        } else {
            sb.append("0,0,0,0,0,0");
        }
        sb.append(',').append(BuildInfo.STAMP);
        sb.append(',').append(extend.stallFault ? 1 : 0)
          .append(',').append(extend.poteFault  ? 1 : 0)
          .append(',').append(extend.yielding   ? 1 : 0);
        hw.timing.recordSince(SensorTiming.LOG_BUILD, wt0, false);
        hw.timing.appendWindow(sb);
        sb.append('\n');

        try {

            long writeStart = System.nanoTime();
            boolean writeFailed = true;
            try {
                out.write(sb.toString());
                writeFailed = false;
            } finally { hw.timing.recordSince(SensorTiming.LOG_WRITE, writeStart, writeFailed); }
            rows++;
            if (now - lastFlushMs >= FLUSH_INTERVAL_MS) {
                long flushStart = System.nanoTime();
                boolean flushFailed = true;
                try {
                    out.flush();
                    flushFailed = false;
                } finally { hw.timing.recordSince(SensorTiming.LOG_FLUSH, flushStart, flushFailed); }
                lastFlushMs = now;
            }
            phLog = (System.nanoTime() - wt0) / 1e6;
        } catch (Exception e) {
            failed = true;
        }
    }

    // 通常終了時に呼び、バッファを保存してファイルを閉じる。
    public void close() {
        if (powerTrace != null) { powerTrace.close(); powerTrace = null; }
        if (out == null) return;
        try { out.flush(); } catch (Exception ignored) { }
        try { out.close(); } catch (Exception ignored) { }
        out = null;
    }

    public String getPath() { return path; }
    public int getRows()    { return rows; }
    public boolean isFailed() { return failed || powerFailed || (powerTrace != null && !powerTrace.healthy()); }
    public String getPowerPath() { return powerTrace == null ? "(unavailable)" : powerTrace.path; }

    private static int buttons(Gamepad g) {
        int b = 0;
        if (g.a) b |= 1 << 0;
        if (g.b) b |= 1 << 1;
        if (g.x) b |= 1 << 2;
        if (g.y) b |= 1 << 3;
        if (g.dpad_up)    b |= 1 << 4;
        if (g.dpad_down)  b |= 1 << 5;
        if (g.dpad_left)  b |= 1 << 6;
        if (g.dpad_right) b |= 1 << 7;
        if (g.left_bumper)  b |= 1 << 8;
        if (g.right_bumper) b |= 1 << 9;
        if (g.left_stick_button)  b |= 1 << 10;
        if (g.right_stick_button) b |= 1 << 11;
        if (g.back)  b |= 1 << 12;
        if (g.start) b |= 1 << 13;
        return b;
    }

    // 値の列では例外も0。対応するtm_*_errorsで同じ集計窓の取得失敗を確認する。
    private double cur(DcMotorEx m, int metric) {
        long start = System.nanoTime(); double value = Double.NaN;
        try { value = hw.reads.current(m, metric); return value; }
        catch (Exception e) { return 0.0; }
        finally { trace(motorName(metric), value, start); }
    }

    private static String motorName(int metric) {
        switch (metric) {
            case SensorTiming.DIAG_DRIVE_L: return "drive_left_A";
            case SensorTiming.DIAG_DRIVE_R: return "drive_right_A";
            case SensorTiming.DIAG_SHOOTER_L: return "shooter_left_A";
            case SensorTiming.DIAG_SHOOTER_R: return "shooter_right_A";
            case SensorTiming.DIAG_INTAKE: return "intake_A";
            case SensorTiming.DIAG_EXTEND: return "extend_A";
            case SensorTiming.DIAG_FEED: return "feed_A";
            default: return "climb_A";
        }
    }
    private void trace(String label, double value, long start) {
        if (powerTrace != null) powerTrace.record(label, value, start,
            !Double.isNaN(value) && !Double.isInfinite(value) && value >= 0, extend, periodMs);
    }
    // Called by the existing shooter voltage acquisition; it does not read the sensor again.
    public void traceVoltage(LynxModule hub, double value, long start) {
        trace("hub_" + hub.getModuleAddress() + "_" + hub.getSerialNumber() + "_V", value, start);
    }

    private double curDc(com.qualcomm.robotcore.hardware.DcMotor m) {
        if (m instanceof DcMotorEx) return cur((DcMotorEx) m, SensorTiming.DIAG_FEED);
        trace("feed_A", Double.NaN, System.nanoTime());
        return 0.0;
    }

    private double hubCurrent() {
        double sum = 0.0;
        if (hubs != null) {
            for (LynxModule h : hubs) {
                long start = System.nanoTime(); double value = Double.NaN;
                try { value = hw.reads.hubCurrent(h, SensorTiming.DIAG_HUB); sum += value; }
                catch (Exception ignored) { }
                finally { trace("hub_" + h.getModuleAddress() + "_" + h.getSerialNumber() + "_A", value, start); }
            }
        }
        return sum;
    }

    private double voltage() {
        double v = Double.POSITIVE_INFINITY;
        if (hubs != null) {
            for (LynxModule h : hubs) {
                long start = System.nanoTime(); double x = Double.NaN;
                try {
                    x = hw.reads.voltage(h, SensorTiming.DIAG_VOLTAGE);
                    if (x > 0) v = Math.min(v, x);
                } catch (Exception ignored) { }
                finally { traceVoltage(h, x, start); }
            }
        }
        return (v == Double.POSITIVE_INFINITY) ? 0.0 : v;
    }

    private static void ap1(StringBuilder sb, double x) { apFix(sb, x, 10L); }
    private static void ap2(StringBuilder sb, double x) { apFix(sb, x, 100L); }
    private static void ap3(StringBuilder sb, double x) { apFix(sb, x, 1000L); }

    private static void apFix(StringBuilder sb, double x, long scale) {
        if (Double.isNaN(x) || Double.isInfinite(x)) { sb.append("NaN"); return; }
        long v = Math.round(x * scale);
        if (v < 0) { sb.append('-'); v = -v; }
        sb.append(v / scale).append('.');
        long f = v % scale;
        for (long d = scale / 10; d > 1; d /= 10) {
            if (f < d) sb.append('0');
        }
        sb.append(f);
    }
}
