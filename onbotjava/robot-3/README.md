# 3号機 OnBot Java

専用ブランチ: `codex/robot-3`
専用作業フォルダー: `github_FGC2026Japan_robot3`
ソース: `src/org/firstinspires/ftc/teamcode/`
対象Control Hub: `1f4562e7e0deafa4`

2026-09-23にユーザーが現在接続中のHubを3号機として指定。端末のシリアルを照合済み。
3号機と4号機は別々のHubを使用する。4号機タスクで `dddad6c23c4eeb3b` を4号機として確認済み。3号機からは書き込まない。
開発ルールはリポジトリ直下と本ディレクトリのAGENTS.mdを参照。

## 3つのメインOpMode

| 表示名 | グループ | 内容 |
|---|---|---|
| MainClass_Blue | 0_MATCH | 計測追加前の通常Blue |
| MainClass_Red | 0_MATCH | 計測追加前の通常Red |
| MainClass_Blue_Timing | 1_DIAG | Blueのセンサー取得時間・展開負荷計測版 |

通常22ファイルは計測前のコミット `9157b47` と完全一致する。計測専用22ファイルはtimingパッケージへ隔離し、通常版に計測処理を追加しない。
計測項目と解析方法は [TIMING.md](TIMING.md) を参照。
既存Android Studio用TeamCodeとは独立しており、これらはGradleビルド対象外。
Hubへの反映結果は [DEPLOYMENT.md](DEPLOYMENT.md) に記録する。

展開負荷の試験手順は [EXTEND_LOAD.md](EXTEND_LOAD.md) を参照。load_v3はPC検証済み、Hubへの配布は未実施。
