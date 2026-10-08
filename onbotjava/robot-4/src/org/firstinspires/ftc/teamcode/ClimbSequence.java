// ============================================================================
// 最終変更: 2026-10-07 JST (UTC+09:00)
// 変更内容: クライム時の展開目標50degに合わせ、上昇開始判定を51deg以下へ変更。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode;

// 登坂段階。gamepad1の左トリガーだけで進める。
//   待機 →(押す)→ 展開を50へ下げ、EXTEND_READY_DEG以下になったら → 上昇(UP_MS)
//   →(上昇後に押す)→ 下降(DOWN_MS)。下降開始からROLLER_DELAY_MSで ローラーが全力で回り始める →(回り始めた後に押す)→ 停止(P保持) →(押す)→ 回転 →(押す)→ 停止 …
// 展開を下げている間、上昇中、下降開始からローラーが回り始めるまでは、押しても無視する。
// 押した後(LOWER以降)は展開を50以下に保つ(limitsExtend)。上昇開始後(hasRisen)は登坂モード中に自動排出する。
// 出力はClimbControllerが段階と経過時間から決める。
public final class ClimbSequence {
    public static final int WAIT = 0, UP = 1, DOWN = 2, ROLL = 3, HOLD = 4, LOWER = 5;
    public static final long UP_MS = 10_500, DOWN_MS = 7_000, ROLLER_DELAY_MS = 6_000;
    // 展開をこの角度以下にしてから上昇を始める(目標は50、制御が50付近で止まっても進めるよう1°の余裕)。
    public static final double EXTEND_READY_DEG = 51.0;
    private int stage = WAIT;
    private boolean held;
    private double stageStartMs = 0.0;
    private double lastAdvanceMs = Double.NEGATIVE_INFINITY;

    // extendAngleDegは展開のポテンショメーター角度(ExtendController.current)。
    public int update(double trigger, double nowMs, double extendAngleDeg) {
        boolean now = trigger > (held ? 0.02 : 0.05);
        boolean pressed = now && !held;
        held = now;
        if (pressed && nowMs - lastAdvanceMs >= 300.0 && accepts(nowMs - stageStartMs)) {
            stage = next(stage);
            stageStartMs = nowMs;
            lastAdvanceMs = nowMs;
        }
        // 展開が下がりきったら上昇を始める。角度が読めない(NaN)間は始めない。
        if (stage == LOWER && extendAngleDeg <= EXTEND_READY_DEG) {
            stage = UP;
            stageStartMs = nowMs;
        }
        return stage;
    }

    // 左トリガーを押した後は、展開を50以下に保つ。
    public boolean limitsExtend() { return stage != WAIT; }
    // 上昇を始めた後(展開を下げている間は含まない)。
    public boolean hasRisen() { return stage != WAIT && stage != LOWER; }

    public double stageElapsedMs(double nowMs) { return nowMs - stageStartMs; }

    private boolean accepts(double elapsedMs) {
        if (stage == LOWER) return false;
        if (stage == UP) return elapsedMs >= UP_MS;
        if (stage == DOWN) return elapsedMs >= ROLLER_DELAY_MS;
        return true;
    }

    private static int next(int stage) {
        switch (stage) {
            case WAIT: return LOWER;
            case UP:   return DOWN;
            case HOLD: return ROLL;
            default:   return HOLD;   // DOWN(ローラー回転中)とROLLは停止へ
        }
    }
}
