// ============================================================================
// 最終変更: 2026-10-02 JST
// 変更内容: 撤去した測距センサーへの依存と測距処理を削除。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode.timing;

// API待ち時間を含む実経過時間。重なる計測区間の時間は加算しない。
// 計測中は記録時にオブジェクトを生成せず、CSV出力時だけ文字列化する。
public final class SensorTiming {
    public static final int IMU_YPR = 0;
    public static final int IMU_GYRO = 1;
    public static final int VELOCITY_SHOOTER = 2;
    public static final int VELOCITY_DRIVE = 3;
    public static final int VELOCITY_INTAKE = 4;
    public static final int POT_ANGLE = 5;
    public static final int TOUCH = 6;
    public static final int DIAG_DRIVE_L = 7;
    public static final int DIAG_DRIVE_R = 8;
    public static final int DIAG_SHOOTER_L = 9;
    public static final int DIAG_SHOOTER_R = 10;
    public static final int DIAG_INTAKE = 11;
    public static final int DIAG_EXTEND = 12;
    public static final int DIAG_FEED = 13;
    public static final int DIAG_CLIMB = 14;
    public static final int DIAG_HUB = 15;
    public static final int DIAG_VOLTAGE = 16;
    public static final int FF_VOLTAGE = 17;
    public static final int CACHE_CLEAR = 18;
    public static final int LOG_BUILD = 19;
    public static final int LOG_WRITE = 20;
    public static final int LOG_FLUSH = 21;
    public static final int LOOP_PERIOD = 22;
    public static final int LOOP_WORK = 23;
    public static final int LOOP_OVERRUN = 24;
    public static final int WORK_MANUAL = 25;
    public static final int WORK_SHOOT = 26;
    public static final int WORK_FIRE = 27;
    public static final int WORK_CLIMB = 28;
    public static final int WORK_DISCHARGE = 29;
    public static final int OPTIONAL_CURRENT = 30;
    private static final String[] NAMES = {"imu_ypr","imu_gyro","velocity_shooter","velocity_drive","velocity_intake","pot_angle","touch","diag_drive_l","diag_drive_r","diag_shooter_l","diag_shooter_r","diag_intake","diag_extend","diag_feed","diag_climb","diag_hub","diag_voltage","ff_voltage","cache_clear","log_build","log_write","log_flush","loop_period","loop_work","loop_overrun","work_manual","work_shoot","work_fire","work_climb","work_discharge","optional_current"};
    private final Counter[] counters = new Counter[NAMES.length];
    private volatile long startNs = Long.MAX_VALUE;

    public SensorTiming() {
        for (int i = 0; i < counters.length; i++) counters[i] = new Counter();
    }
    public void start() { startAt(System.nanoTime()); }
    void startAt(long now) { startNs = now; }
    public void recordSince(int id, long start, boolean failed) {
        record(id, start, System.nanoTime(), failed);
    }
    void record(int id, long start, long end, boolean failed) {
        if (start < startNs || end < start) return;
        counters[id].add(end - start, failed);
    }
    public static String csvHeader() {
        StringBuilder sb = new StringBuilder();
        for (String name : NAMES) {
            sb.append(",tm_").append(name).append("_n")
              .append(",tm_").append(name).append("_total_us")
              .append(",tm_").append(name).append("_max_us")
              .append(",tm_").append(name).append("_errors");
        }
        return sb.toString();
    }
    // 前回出力以降の全呼出しを集約。各カウンタを短時間だけロックし、I/Oはしない。
    public void appendWindow(StringBuilder sb) {
        for (Counter counter : counters) counter.appendAndReset(sb);
    }
    private static final class Counter {
        private long count, totalNs, maxNs, errors;
        synchronized void add(long ns, boolean failed) {
            count++;
            totalNs += ns;
            maxNs = Math.max(maxNs, ns);
            if (failed) errors++;
        }
        synchronized void appendAndReset(StringBuilder sb) {
            sb.append(',').append(count).append(',').append(totalNs / 1000)
              .append(',').append(maxNs / 1000).append(',').append(errors);
            count = totalNs = maxNs = errors = 0;
        }
    }
}
