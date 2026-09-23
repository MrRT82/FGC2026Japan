// ============================================================================
// 最終変更: 2026-09-22 11:20:42 JST (UTC+09:00)
// 変更内容: 変更日時ヘッダーを追加。直前までの修正内容を保持。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

// 出力差がepsilon未満なら送信を省略し、250msごとに再送する。
public class CachedPower {

    private static final long REFRESH_MS = 250;

    private final DcMotor motor;
    private final double epsilon;

    private double last = Double.NaN;

    private long lastWriteMs = 0;

    public int skipped = 0;

    public int written = 0;

    public CachedPower(DcMotor m, double eps) {
        motor   = m;
        epsilon = eps;
    }

    // 通常指令。0も許容差の対象になるため、即時停止が必要な箇所はsetNow(0)を使う。
    public void set(double power) {
        long now = System.currentTimeMillis();
        boolean changed = Double.isNaN(last) || Math.abs(power - last) >= epsilon;
        boolean stale   = (now - lastWriteMs) >= REFRESH_MS;
        if (!changed && !stale) {
            skipped++;
            return;
        }
        motor.setPower(power);
        last        = power;
        lastWriteMs = now;
        written++;
    }

    // 保護停止などでキャッシュを迂回し、必ず書き込む。
    public void setNow(double power) {
        motor.setPower(power);
        last        = power;
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
