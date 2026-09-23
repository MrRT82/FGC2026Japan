// ============================================================================
// 最終変更: 2026-09-22 11:20:42 JST (UTC+09:00)
// 変更内容: 変更日時ヘッダーを追加。直前までの修正内容を保持。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode;

// 登坂段階。押し直しで1段進み、キャンセルは必ず停止段階へ移る。
public final class ClimbSequence {
    private int stage = 0;
    private boolean primaryHeld, secondaryHeld;
    private double lastAdvanceMs = Double.NEGATIVE_INFINITY;

    public int update(double primary, double secondary, boolean cancel,
                      boolean autoUnlockDue, double nowMs) {
        boolean primaryNow = primary > (primaryHeld ? 0.02 : 0.05);
        boolean secondaryNow = secondary > (secondaryHeld ? 0.02 : 0.05);
        boolean primaryPressed = primaryNow && !primaryHeld;
        boolean secondaryPressed = secondaryNow && !secondaryHeld;
        primaryHeld = primaryNow;
        secondaryHeld = secondaryNow;

        if (cancel) {
            if (stage == 1 || stage == 2) stage = 3;
            else if (stage == 4) stage = 5;
            return stage;
        }
        // gp2は最初の解錠も可能。gp1は解錠後のみ操作できる。
        if ((secondaryPressed || (primaryPressed && stage != 0))
                && nowMs - lastAdvanceMs >= 300.0) {
            stage = stage >= 5 ? 2 : stage + 1;
            lastAdvanceMs = nowMs;
        }
        if (stage == 0 && autoUnlockDue) stage = 1;
        return stage;
    }
}
