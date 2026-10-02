// ============================================================================
// 最終変更: 2026-10-02 JST (UTC+09:00)
// 変更内容: Blue計測専用パッケージへ隔離(通常Blue/Redとは共有しない)。登坂を「押す→上昇(時間)→押す→下降(時間)→ローラー→押す→終了」の流れへ変更。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode.timing;

// 登坂段階。押す(または自動)→上昇(時間で停止)→押す→下降(時間/下端センサーで停止)→ローラー→押す→終了。
// 段階番号はメインのswitchに合わせる: 0=待機, 4=上昇, 2=下降→ローラー, 3=終了。上昇・下降の時間停止は
// ClimbControllerが行う。下降中は下端センサーでも止まる。
public final class ClimbSequence {
    public static final int WAIT = 0, UP = 4, DOWN = 2, DONE = 3;
    private int stage = WAIT;
    private boolean primaryHeld, secondaryHeld;
    private double lastAdvanceMs = Double.NEGATIVE_INFINITY;

    // cancelは使わない(引数はメインとの互換のため残す)。
    public int update(double primary, double secondary, boolean cancel,
                      boolean autoUnlockDue, double nowMs) {
        boolean primaryNow = primary > (primaryHeld ? 0.02 : 0.05);
        boolean secondaryNow = secondary > (secondaryHeld ? 0.02 : 0.05);
        boolean primaryPressed = primaryNow && !primaryHeld;
        boolean secondaryPressed = secondaryNow && !secondaryHeld;
        primaryHeld = primaryNow;
        secondaryHeld = secondaryNow;

        // gp2は最初の上昇から操作できる。gp1は上昇後の下降だけ操作できる。
        boolean pressed = (secondaryPressed || (primaryPressed && stage != WAIT))
                && nowMs - lastAdvanceMs >= 300.0;
        if (pressed) lastAdvanceMs = nowMs;
        if (stage == WAIT && (pressed || autoUnlockDue)) stage = UP;
        else if (stage == UP && pressed) stage = DOWN;
        else if (stage == DOWN && pressed) stage = DONE;
        // DONEの後は段階が進まず、サーボもローラーも止まったまま。
        return stage;
    }
}
