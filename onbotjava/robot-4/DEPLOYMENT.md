# 4号機 Blue計測版の配布記録

2026-09-23 JST、ユーザーの「今接続している4号機のHubに書き込んでほしい」に基づき実施。

- 対象Hub: dddad6c23c4eeb3b。ADBの唯一の接続端末で、ro.serialnoも一致。この確認により4号機の対応を確定。
- Robot Controller: 11.1。
- 配布コード: codex/robot-4 の 180bbfdc678e0f6fb286fff066f5a73ab6d760c7。
- 配布前に /sdcard/FIRST/java 全体をローカルへバックアップ。
- 通常22JavaはGitチェックアウトの改行差を除き一致したため、転送せず保持。
- timingパッケージ21Javaだけを追加。読み戻しSHA-256は配布元と全21ファイル一致。
- 既存22Javaはバックアップとバイト単位で一致。配布対象43Javaすべての内容一致を確認（通常版は改行差を正規化）。
- 初回ビルドは成功表示だったが、生成JARに計測クラスが含まれていなかった。一意の新しいbuildRequestで再ビルドし、最新JARに timing/MainClass_Blue_Timing.class があることを確認。初回未収録の原因は未確定。
- 2回目のbuildStarted/buildSuccessful/buildComplete更新とSUCCESSFULログを確認。buildLogは空。
- 通常MainClass_Blue、MainClass_Red、追加MainClass_Blue_Timingの登録を確認。既存controllerChecker、pot_angle_testも登録済み。
- 計測版の識別子: 20260923_154923_robot4_blue_timing_v2。
- 通常版の識別子: 20260922_112042。
- INIT/START・走行・計測データ収集は行っていない。RC再起動も行っていない。

バックアップ・読み戻し・ハッシュ一覧・ビルド状態・登録ログは作業領域の hub_robot4_timing_20260923_155417 に保存。GitHubへ実機ログやバックアップは追加しない。

Hubの時計は1970年を表示していたため、ビルドの更新順序・生成物・再登録で完了を判定。時計設定は変更していない。
