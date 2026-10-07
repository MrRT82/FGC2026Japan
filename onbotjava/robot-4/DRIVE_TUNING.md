# 4号機 足回りギヤ変更後の調整

2026-10-07。ユーザー申告で、足回りのUltraPlanetaryを 5:1+5:1（公称25:1、実際27.36:1）から
5:1+3:1（公称15:1、実際15.15:1）へ変更した。実比はREVの公表値（5:1は5.23、3:1は2.89）による。

## 換算の考え方

走行モーターは `RUN_USING_ENCODER`。`setPower(p)` は「設定されたモーター種類の最大tick/s × p」の速度指令になる
（SDK 11.0の`LynxDcMotorController`で確認）。エンコーダーはモーター軸にあるため、ギヤを変えても同じ`p`で
モーター軸の速度は変わらず、車輪の速度だけが **27.36 / 15.15 = 1.806倍** になる。

SDK 11.0のREV HD Hex系モーター種類（20:1、40:1、UltraPlanetary）はいずれも最大2800 ticks/s、到達率0.85なので、
Robot Configurationでどれを選んでいても出力1.0はモーター軸5100rpmの指令になる。車輪直径90mm（ユーザー申告）では
出力1.0の指令速度は約1.59m/s（旧ギヤ約0.88m/s）。実際に出る速度は負荷で下がり得るので計測の`per_power`で確認する。

したがって「出力 ÷ 角度」「出力 ÷ 角速度」の単位を持つゲインは、旧値 ÷ 1.806 で角度あたりの効きが旧ギヤと同じになる。
本体と`timing/`の両方を換算した。**実機では未検証。**

| 場所 | 項目 | 旧 | 新 |
| --- | --- | --- | --- |
| ImuController | kp | 0.040 | 0.0221 |
| ImuController | kd | 0.0040 | 0.00221 |
| ImuController | kS | 0.100 | 0.0554 |
| ImuController | iMaxContrib（ki=0のため現在未使用） | 0.15 | 0.0831 |
| DriveAssist | kpHold | 0.015 | 0.00831 |
| DriveAssist | kdHold | 0.0020 | 0.00111 |

ユーザー判断で据え置いたもの（速くなった分だけ速く動く）:

- `ImuController.maxTurnPower = 1.0`: 自動方位合わせ（射撃・排出・登坂）の最高旋回速度は約1.8倍。旧ギヤと同じ上限は0.554。
- `DriveController.maxSpeed`、`TURN_NORMAL`: 手動の前後・旋回とも約1.8倍。
- `DriveAssist.holdMaxOut = 0.35`: 方位保持の最大補正速度は約1.8倍。旧ギヤと同じ上限は0.194。

角度・角速度・時間の閾値（`tolerance`、`ksRateCut`、`captureRateDps`、`settleTimeoutMs`など）は変更していない。
旋回の止まりぎわが変わるため、実測で確認する。

## ブレーキと加速の現状

- 手動入力の加速制限は無効（`fwdSlew`/`turnSlew`=0）。IMU出力の増加率制限も無効（`turnSlewPerSec`=0）。
- スティックを離すと、`DriveController`が前回指令の5%を逆向きに1周期だけ出し、その後0にする。
  `RUN_USING_ENCODER`の0は「速度0を目標にHubの速度制御が止めにいく」指令なので、実際には能動的なブレーキになる。
  射撃中にRBを押したときの走行停止も同じ。
- ギヤ変更による理論上の影響: 最高速1.8倍、車輪の力0.55倍、運動エネルギー3.3倍。
  同じ出力から止まる距離は、タイヤが滑る場合で約3.3倍、モーターの力で決まる場合で最大約5.9倍（1.806³）になり得る。
  止まるまでの時間も延び、その間モーターは大電流になりやすい。高負荷時のHub再起動の既知症状と重なる可能性がある。

本番コードのブレーキ・加速は、下の計測で「現行（MATCH）」と「ランプで減速（RAMP）」を比べてから決める。

## 計測OpMode `Drive_Tune_R4`（1_DIAG）

ファイル: `tuning/DriveTuneConfig.java`、`tuning/DriveTuneCore.java`、`tuning/Drive_Tune_R4.java`。
既存の`tuning/TuneCsvWriter.java`と、本体の`ImuController`・`DriveController`（換算版）を使う。
設定は`tools/robot4_drive_tune.properties.example`を参考に`/sdcard/FIRST/robot4_drive_tune.properties`へ置く。

`RobotHardware.init()`は呼ばない（登坂サーボやロックを動かさないため）。走行モーターの向き・BRAKE・`RUN_USING_ENCODER`と
IMUの取付向きは`RobotHardware`と同じ設定を個別に行う。`RobotHardware`を変えたらここも合わせる。

### 安全

- 既定は`motion_enabled=false`で、INITから計測と表示だけを行う。
- 有効化には`gear_reduction`、`wheel_diameter_mm`、`max_power`、`max_travel_m`、`max_turn_deg`が必要。不正ならINITで止まる。
- RBを押し続けている間だけ動く。離すと0（=Hubの能動ブレーキ）。Bはラッチ停止で、再INITまで動かない。
- 自動でブレーキまたは停止する条件: 走行距離、旋回角、`run_ms`、ブレーキ時間切れ、制御周期100ms超、ログ保存失敗、IMU値の異常、
  HEADINGの暴走（目標+60度超え、または逆方向30度）。
- 電流と電圧では止めない（ブレーキ時の値を測るため、記録のみ）。Hubが落ちるまで追い込まない。
- 広い場所で、進行方向に人がいないこと。壁までは`max_travel_m`に停止距離を足した以上の余裕を取る。最初は低い出力から。

### 操作

INIT中とSTART後に選べる（試験中は変更不可）:
十字左右 = 試験（STRAIGHT / TURN / HEADING）、十字上下 = 出力（HEADINGでは角度）、X = 向き、Y = ブレーキ方式（MATCH / RAMP）。
RBを押したままAを新しく押すと開始。

- STRAIGHT: 加速（`accel_per_s`、0なら即時）→ 巡航 → `run_ms`か`max_travel_m`でブレーキ → 車輪が200ms静止で終了。
- TURN: 同じ流れをその場旋回で行い、`max_turn_deg`でブレーキ。
- HEADING: 本番の`ImuController`で「開始時の方位 + 角度」へ`heading_ms`の間合わせる。
  `imu_kp`などを書けば上書きできる（未記入なら本番値）。
- MATCH: 本番と同じ「5%逆向き → 0」。RAMP: `brake_per_s`で出力を下げる（0なら逆向きパルスなしで即0）。

### 進め方の例

1. INITだけ行い、表示（モーター種類、「出力1.0の速度」、電圧）が妥当か確認する。
   「最大 ticks/s」がRobot Configurationのモーター種類で決まる。種類を変えていたらここで分かる。
2. STRAIGHT 0.3をMATCHで前後各2回 → 0.5で同じ → 同じ出力でRAMP。
3. TURN 0.3 / 0.5。
4. HEADING 45 / 90 / 180を両方向。
5. 問題がなければ`max_power`と`powers`を上げて0.7、1.0へ。

### 解析

`python -B tools/analyze_drive_tune.py <CSV> [別CSV...]`（`--json`で機械可読）

| 指標 | 意味 | 判断の例 |
| --- | --- | --- |
| `stop` ms / m | ブレーキ開始から静止までの時間と距離 | 長すぎればブレーキのランプや最高速の制限を検討 |
| `decel_peak`、`pitch` | 最大減速度、ブレーキ中の前後傾き | 傾きが大きければ減速を緩める（RAMP） |
| `I`、`Vmin` | 走行モーター電流とHub電圧の最大・最小（間引き値） | MATCHで電圧が大きく下がるならRAMPを本番へ |
| `t90` | 最高速の90%までの時間 | 電流・電圧と合わせて加速制限の要否を判断 |
| `per_power` | 出力1あたりの速度（巡航300ms以上の試験のみ） | 「出力1.0の速度」より大きく低ければ、その出力でHubの速度制御が追従できていない |
| `overshoot`、`settle` | HEADINGの行き過ぎと、±3度に収まるまでの時間 | 行き過ぎが大きければkdを上げるか`maxTurnPower`を下げる。遅い・手前で止まるならkS/kpを上げる |

電流・電圧は1周期1項目ずつの巡回取得で、短いピークを取り逃す。記録にないことを「起きていない」根拠にしない。

### 送り返すもの

`/sdcard/FIRST/logs/robot4_drive_tune_...csv`と同名の`.properties`、試験の順番、床の種類、電池電圧、可能なら動画。
生ログ・実設定・機器ログはGitHubへ追加しない。

## 注意

- 事前点検`Preflight_R4`は走行モーターを片側ずつ出力0.18で0.55秒回し、2.5Aで中止する。ギヤ変更で車体の動きは約1.8倍になり、
  トルク低下で電流も上がりやすい。誤って中止になる場合は閾値の見直しが必要（今回は未変更）。
- 通常の試合OpModeが書く`robot_*.csv`（`RobotLogger`）には走行出力・速度・IMUが入っている。旧ギヤ時のログがHubに残っていれば、
  新ギヤとの比較に使える。
