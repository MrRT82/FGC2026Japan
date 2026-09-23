// ============================================================================
// 最終変更: 2026-09-22 11:20:42 JST (UTC+09:00)
// 変更内容: 変更日時ヘッダーを追加。直前までの修正内容を保持。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// 射撃時の前後距離制御。距離cm、角度deg、dt秒。センサー取得と制御更新を分離する。
public class Distance {
    private final DistanceSensor sensor;
    private final ImuController imuController;

    public double kp = 0.01;
    public double ki = 0.000;

    public double kd = 0.000;

    // 正面距離の目標。斜めの測距はIMUの方位誤差でcos補正する。
    private double targetDistance = 120.0;

    public double validMin = 3.0;
    public double validMax = 200.0;

    // 接近は残距離に応じて減速。後退側の上限は別設定。
    public double maxApproach = 0.42;

    public double maxRetreat = 0.60;

    public double approachScale = 50.0;

    public double tolerance = 3.0;

    // 角度不適合の猶予。無効サンプル時の出力はupdate内で0にする。
    private static final long FAIL_CUT_MS = 120;
    private long failStartMs = 0;

    public double maxAngleForValid = 12.0;

    // 有効な新規サンプルだけを5点中央値へ追加する。
    private final double[] medBuf = new double[5];
    private int medIdx = 0, medCount = 0;
    private int totalSamples = 0, totalFails = 0;

    private double distanceOutput = 0.0;

    private double integral = 0.0;
    private double prevError = 0.0;

    private double dt = 0.02;
    private static final double DT_MIN = 0.002;
    private static final double DT_MAX = 0.200;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private static final double INTEGRAL_MAX = 1.0;

    private double lastDistance      = 0.0;
    private double lastFrontDistance = 0.0;
    private double lastAngleError    = 0.0;
    private int    failCount         = 0;
    private boolean lastValid        = false;

    // 測距スレッドからメインへ結果と時刻を公開する。
    private volatile double readerValue = 0.0;
    private volatile long   readerStamp = 0;
    private volatile boolean readerRun  = false;
    private volatile int    readerErrors = 0;
    private Thread readerThread = null;
    private long   lastUsedStamp = 0;

    // 停止処理が漏れた場合の上限15分。通常はstopReader()で終了させる。
    private static final long READER_MAX_LIFE_MS = 15 * 60 * 1000L;

    // 測定終了後に40ms待機。センサー測定時間もサンプル間隔へ加算される。
    private static final long READER_GAP_MS = 40;

    private static final long READER_IDLE_MS = 25;

    private volatile boolean readerActive = true;

    public void setReaderActive(boolean on) {
        if (readerActive != on) {
            clearFilter();
            // 再開直後は停止前のサンプルを再利用せず、新しい測定を待つ。
            lastUsedStamp = readerStamp;
        }
        readerActive = on;
    }

    // 150msより古いデータは制御に使わない。
    private static final long SAMPLE_STALE_MS = 150;

    public long getSampleAgeMs() {
        long st = readerStamp;
        return (st == 0) ? -1 : (System.currentTimeMillis() - st);
    }

    public double getSensorRaw() { return readerValue; }

    private volatile long readerStartedAtMs = 0;

    // 遅い測距APIを別スレッドで実行する。Hubの通信負荷そのものは残る。
    public void startReader() {
        if (readerThread != null) return;
        readerRun = true;
        readerStartedAtMs = System.currentTimeMillis();
        final long deadline = System.currentTimeMillis() + READER_MAX_LIFE_MS;
        readerThread = new Thread(new Runnable() {
            @Override public void run() {
                while (readerRun && System.currentTimeMillis() < deadline) {
                    long gap;
                    if (readerActive) {
                        try {
                            double v = sensor.getDistance(DistanceUnit.CM);
                            readerValue = v;
                            readerStamp = System.currentTimeMillis();
                        } catch (Exception e) {
                            readerErrors++;
                        }
                        gap = READER_GAP_MS;
                    } else {

                        gap = READER_IDLE_MS;
                    }
                    try {
                        Thread.sleep(gap);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        return;
                    }
                }
            }
        }, "DistanceReader");
        readerThread.setDaemon(true);
        readerThread.start();
    }

    // 割り込みと最大200msのjoinで終了を待つ。
    public void stopReader() {
        readerRun = false;
        Thread t = readerThread;
        readerThread = null;
        if (t != null) {
            t.interrupt();
            try { t.join(200); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
        }
    }

    public boolean isReaderRunning() {
        Thread t = readerThread;
        return t != null && t.isAlive();
    }

    public int getReaderErrors() { return readerErrors; }

    public Distance(DistanceSensor sensor, ImuController imuController) {
        this.sensor = sensor;
        this.imuController = imuController;
    }

    private static double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // センサーへのアクセスは行わず、最後の中央値を返す。
    public double getDistance() {
        return lastDistance;
    }

    public double getAngleError() {
        return lastAngleError;
    }

    public double getFrontDistance() {
        return lastFrontDistance;
    }

    private void clearFilter() {
        medCount = 0;
        medIdx = 0;
        lastValid = false;
        lastDistance = 0.0;
        lastFrontDistance = 0.0;
        distanceOutput = 0.0;
        integral = 0.0;
        prevError = 0.0;
    }

    private double median() {
        double[] a = new double[medCount];
        for (int i = 0; i < medCount; i++) a[i] = medBuf[i];

        for (int i = 1; i < a.length; i++) {
            double v = a[i];
            int j = i - 1;
            while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
            a[j + 1] = v;
        }
        return (a.length == 0) ? 0.0 : a[a.length / 2];
    }

    public double getFailRate() {
        return (totalSamples > 0) ? 100.0 * totalFails / totalSamples : 0.0;
    }

    public boolean isValid() {
        return lastValid;
    }

    public int getFailCount() {
        return failCount;
    }

    // 履歴、サンプル鮮度、失敗継続時間で距離制御の可否を判定する。
    public boolean isUsable() {

        if (!readerActive || medCount == 0) return false;

        if (readerStamp == 0
                || System.currentTimeMillis() - readerStamp > SAMPLE_STALE_MS) return false;

        return failStartMs == 0
                || (System.currentTimeMillis() - failStartMs) < FAIL_CUT_MS;
    }

    public void setTargetDistance(double distance) {
        targetDistance = distance;
        integral = 0.0;
        prevError = 0.0;
    }

    public double getTargetDistance() {
        return targetDistance;
    }

    // IMUのupdate後に呼ぶ。新しい測距サンプルが来たときだけ制御を更新する。
    public void update() {

        if (!readerActive) return;

        if (readerThread == null && !readerRun) startReader();

        long stamp = readerStamp;

        if (stamp == 0) return;

        if (System.currentTimeMillis() - stamp > SAMPLE_STALE_MS) {
            clearFilter();
            return;
        }
        if (stamp == lastUsedStamp) return;
        if (lastUsedStamp != 0 && stamp - lastUsedStamp > SAMPLE_STALE_MS) {
            clearFilter();
        }
        lastUsedStamp = stamp;
        long nowMs = stamp;

        double measured = loopTimer.seconds();
        loopTimer.reset();
        if (measured < DT_MIN || measured > DT_MAX) {
            dt = 0.033;
        } else {
            dt = measured;
        }

        double raw = readerValue;
        totalSamples++;

        lastAngleError = Math.abs(normalizeAngle(
                imuController.getTarget() - imuController.getCurrent()));

        // NaN・無限大・距離範囲外・方位誤差過大を除外する。
        boolean rangeOk = !Double.isNaN(raw) && !Double.isInfinite(raw)
                && raw >= validMin && raw <= validMax;
        boolean angleOk = lastAngleError <= maxAngleForValid;
        lastValid = rangeOk && angleOk;

        if (lastValid) {
            failCount = 0;
            failStartMs = 0;

            medBuf[medIdx] = raw;
            medIdx = (medIdx + 1) % medBuf.length;
            if (medCount < medBuf.length) medCount++;
            lastDistance = median();
        } else {
            failCount++;
            totalFails++;
            if (failStartMs == 0) failStartMs = nowMs;

            clearFilter();
        }

        double angleRad = Math.toRadians(Math.min(lastAngleError, 89.0));
        lastFrontDistance = lastDistance * Math.cos(angleRad);

        // 無効な測距では前後出力と積分をゼロにする。
        if (medCount == 0 || !lastValid) {
            distanceOutput = 0.0;
            integral = 0.0;
            prevError = 0.0;
            return;
        }

        if (failStartMs != 0 && (nowMs - failStartMs) >= FAIL_CUT_MS) {
            distanceOutput = 0.0;
            integral = 0.0;
            return;
        }

        double error = targetDistance - lastFrontDistance;

        if (Math.abs(error) <= tolerance) {
            distanceOutput = 0.0;
            integral = 0.0;
            prevError = error;
            return;
        }

        integral += error * dt;
        integral = clamp(integral, -INTEGRAL_MAX, INTEGRAL_MAX);

        double deriv = (error - prevError) / dt;

        double out = kp * error + ki * integral + kd * deriv;

        // 目標へ近づくほど接近側出力を制限し、行き過ぎを抑える。
        double remain = lastFrontDistance - targetDistance;
        double approachCap = maxApproach * clamp(remain / approachScale, 0.0, 1.0);

        if (out < 0) {
            out = Math.max(out, -approachCap);

            if (!lastValid) out = 0.0;
        } else {
            out = Math.min(out, maxRetreat);

            if (!lastValid) out = 0.0;
        }

        distanceOutput = clamp(out, -1.0, 1.0);
        prevError = error;
    }

    public double getOutput() {
        return distanceOutput;
    }

}
