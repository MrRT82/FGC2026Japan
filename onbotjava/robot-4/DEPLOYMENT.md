# 4号機 Blue計測版の配布記録

## 2026-09-29 Tune v3 静止判定修正

- 実機BREAKAWAY_POSは角度幅0.082度・誤差最大0.082度で保持していたが、ポテンショメーターの1カウント揺れを微分した速度が-1.59～+1.16度/秒となり、旧0.5度/秒条件によりNOT_SETTLEDとなった。
- 静止判定を500ms内の角度幅0.15度以内・誤差0.3度以内・非飽和へ変更。基準電圧固定後も変位0.15度以内で判定する。15度/秒の過速度、最大3度移動、電流・電圧、誤方向などの保護は維持。
- 実測の41.973度／42.055度交互入力を再現するテストを追加。PCのJDK17とFTC SDK JARで全58 JavaをJava 8向けにコンパイル成功。ExtendTuneCoreTest 21項目、ExtendIdentificationTest 30項目、ExtendConfigCompatibilityTest 8項目成功。
- 初回PCコンパイルは引数ファイルの相対パス生成不備でソース未検出。ASCII相対パスへ修正後に成功。識別テスト初回は新テストの挿入位置により既存状態を上書きして1項目失敗し、独立した位置へ移して成功。
- 4号機Hub `dddad6c23c4eeb3b`へExtendTuneCore.javaとExtend_Tune_R4.javaだけを転送。読み戻しSHA-256一致、OnBotJavaビルド成功、空のbuildLog、既存OpMode再登録を確認。
- Hub生成JARのBUILDは`20260929_robot4_extend_tune_v3`。設定はmotion_enabled=true、BREAKAWAY_POS、train、motor_hub_index=0を保持。INIT／STARTは配布作業では行っていない。
- 実機でのv3 BREAKAWAY_POS再試験は未実施。バックアップ、ログ、生成JARは作業領域`robot4_breakaway_fix_20260929`に保存し、Gitには含めない。

## 2026-09-29 Tune v2・現地追加機能との統合

- 対象は4号機Hub `dddad6c23c4eeb3b`。転送前にADBの`ro.serialno`一致を確認。
- 現地追加のAuto／Sweep／SysId 9 Javaと旧ExtendTuneConfigをそのままリポジトリへ収録。v2はExtendIdentificationConfigへ分離し、既存設定検証とAutoの試験時間を保持。
- Hubへ新規ExtendIdentificationConfigと更新ExtendTuneCore、Extend_Tune_R4の3 Javaだけを転送。TuneCsvWriterは既存とSHA-256一致のため転送せず。
- 実設定はmotion_enabledをtrueからfalseへ変更しただけ。ゲイン、角度範囲、電流・電圧閾値、Auto設定を保持。共通設定を読むTuneとAutoは動作無効となる。別設定のSweep／SysIdの有効状態は変更していない。
- PC: Windows、JDK17、FTC SDKのonbotjava-classes.jarとandroid.jarで、実Hubの全ソースに更新を重ねた57 Javaをコンパイル。`javac -encoding UTF-8 -source 8 -target 8 -cp '../hub_compile_libs/onbotjava-classes.jar;../hub_compile_libs/android.jar' -d classes @sources.txt`成功（Java8 bootstrap警告1件）。引数ファイルは相対パス一覧。
- PC: `javac -encoding UTF-8 -cp classes -d classes <test.java>`、`java -cp classes org.firstinspires.ftc.teamcode.tuning.<test>`でExtendTuneCoreTest 21項目、ExtendIdentificationTest 29項目、ExtendConfigCompatibilityTest 8項目成功。制御停止・不正設定・試験上限と、旧Autoがv2の8秒制限や追加検証に影響されないことを確認。
- 検証中の失敗: 初回javacはUTF-8引数ファイル内の日本語絶対パスの読み取りでMalformedInputException。ASCII相対パスに修正して成功。別の制限環境ではコンパイル済みクラスが見つからず互換性テストのコンパイルが失敗。同じ昇格環境で再実行して成功。javapも制限環境ではJAR終了時AccessDeniedExceptionとなり、昇格環境でエラーなし・終了コード0を確認。
- ADB読み戻しとPowerShell Get-FileHashによるSHA-256照合で、全57 Javaが配布予定と一致。対象外54 Javaは配布前とバイト一致。設定も配布元と一致。
- 一意のbuildRequest.txtでHubビルド。buildStarted／buildSuccessful／buildComplete更新、空のbuildLog、`onBotJava build finished successfully`を確認。
- Hub生成OnBotJava.jarを`jar tf`および`javap -constants`で確認。新設定クラス、Tune／Auto／Sweep／SysIdを収録し、TuneのBUILDは`20260927_robot4_extend_tune_v2`。
- ビルド完了後のOpmodeRegistrationログでTune、Auto、Sweep、SysId、通常Blue／Red、Blue_Timing、既存診断の登録を確認。
- INIT／START、機構の動作、実CSV収集は未実施。実機の制御性能・停止閾値の妥当性・再起動原因は未検証。新しいゲインは算出・適用していない。
- バックアップと証拠は作業領域`hub_robot4_tune_20260929`に保存。実設定・機器ログ・生成JARはGitへ含めない。Hub時計は1970年のため、日時の大小ではなく更新内容・生成JAR・ビルド後登録で判断した。

## 2026-09-23 Blue計測版

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
