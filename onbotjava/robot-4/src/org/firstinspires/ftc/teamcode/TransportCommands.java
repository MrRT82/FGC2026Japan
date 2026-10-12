// ============================================================================
// 最終変更: 2026-09-23 JST (UTC+09:00)
// 変更内容: 変更日時ヘッダーを追加。直前までの修正内容を保持。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode;

// 回収・送りの指令を一箇所で決定。方向は+1正転、-1逆転、0停止。
public final class TransportCommands {
    public final int intakeDirection, feedDirection;
    public final boolean raiseExtend, shootPower;

    private TransportCommands(int intake, int feed, boolean raise, boolean reducedPower) {
        intakeDirection = intake;
        feedDirection = feed;
        raiseExtend = raise;
        shootPower = reducedPower;
    }

    public static TransportCommands resolve(boolean cancel, boolean manualDischarge,
            boolean dischargeMode, boolean shooting, boolean fireHeld,
            boolean feedReady, boolean jamReverse) {
        if (cancel) return new TransportCommands(0, 0, false, false);
        // BとX排出は速度ゲート・自動逆転より優先する。
        if (manualDischarge || dischargeMode) {
            return new TransportCommands(-1, -1, true, false);
        }
        if (shooting) {
            boolean feed = fireHeld && feedReady;
            return new TransportCommands(1, feed ? -1 : 0, feed, true);
        }
        if (jamReverse) return new TransportCommands(-1, 0, false, false);
        return new TransportCommands(1, 1, false, false);
    }
}
