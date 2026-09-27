// ============================================================================
// 最終変更: 2026-09-27 11:20:42 JST (UTC+09:00)
// 変更内容: 変更日時ヘッダーを追加。直前までの修正内容を保持。
//          　getpowerlevelの正負を逆にした(栗原)
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

// 搬送の方向と出力を設定する。射撃の速度ゲートはメインが担当。
public class FeedController
{
    private DcMotor feed;

    private CachedPower feedOut;

    public double power = 1.0;

    // 現在は通常時と同じ1.0。射撃モードにしてもFeed自体の出力は下がらない。
    public double powerShoot = 1.0;

    private boolean shootMode = false;

    // 0は電流制限無効。有効化すると200msごとに電流を取得する。
    public double currentLimit = 0.0;
    private static final long   CURRENT_READ_MS = 200;
    private static final double SCALE_MIN = 0.35;
    private static final double SCALE_DOWN = 0.06;
    private static final double SCALE_UP   = 0.03;
    private long   lastCurMs = 0;
    private double lastCur = 0.0;
    private double scale = 1.0;

    public double getCurrent() { return lastCur; }

    public double getLimitScale() { return scale; }

    private void updateLimit() {
        if (currentLimit <= 0) { scale = 1.0; return; }
        long now = System.currentTimeMillis();
        if (now - lastCurMs >= CURRENT_READ_MS) {
            lastCurMs = now;
            try {
                if (feed instanceof DcMotorEx) {
                    lastCur = ((DcMotorEx) feed).getCurrent(CurrentUnit.AMPS);
                }
            } catch (Exception ignored) { }
            if (lastCur > currentLimit) {
                scale -= SCALE_DOWN;
            } else if (scale < 1.0) {
                scale += SCALE_UP;
            }
            if (scale < SCALE_MIN) scale = SCALE_MIN;
            if (scale > 1.0) scale = 1.0;
        }
    }

    public FeedController(RobotHardware hr){
        feed = hr.feedMotor;
        feedOut = new CachedPower(feed, 0.02);
    }

    public void setShootMode(boolean on){
        shootMode = on;
    }

    public double getSkipRate(){ return feedOut == null ? 0.0 : feedOut.getSkipRate(); }

    public double getPowerLevel(){
        return shootMode ? powerShoot : power;
    }

    public void Feed_turn_right(){
        updateLimit();
        feedOut.set(-getPowerLevel() * scale);
    }
    public void Feed_turn_left(){
        updateLimit();
        feedOut.set(getPowerLevel() * scale);
    }
    public void Feed_turn_stop(){
        feedOut.set(0);
    }
}
