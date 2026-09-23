// ============================================================================
// 最終変更: 2026-09-22 11:20:42 JST (UTC+09:00)
// 変更内容: 変更日時ヘッダーを追加。直前までの修正内容を保持。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

// 位置差がepsilon未満なら送信を省略し、250msごとに再送する。
public class CachedServo {

    private static final long REFRESH_MS = 250;

    private final Servo servo;
    private final double epsilon;

    private double last = Double.NaN;

    private long lastWriteMs = 0;

    public int skipped = 0;

    public int written = 0;

    public CachedServo(Servo s) {
        this(s, 0.002);
    }

    public CachedServo(Servo s, double eps) {
        servo   = s;
        epsilon = eps;
    }

    public void set(double position) {
        long now = System.currentTimeMillis();
        boolean changed = Double.isNaN(last) || Math.abs(position - last) >= epsilon;
        boolean stale   = (now - lastWriteMs) >= REFRESH_MS;
        if (!changed && !stale) {
            skipped++;
            return;
        }
        servo.setPosition(position);
        last        = position;
        lastWriteMs = now;
        written++;
    }

    // 中立や保護指令など、即時書き込みを保証したいときに使う。
    public void setNow(double position) {
        servo.setPosition(position);
        last        = position;
        lastWriteMs = System.currentTimeMillis();
        written++;
    }

    public double getLast() {
        return Double.isNaN(last) ? 0.0 : last;
    }

    public double getSkipRate() {
        int total = skipped + written;
        return total == 0 ? 0.0 : 100.0 * skipped / total;
    }
}
