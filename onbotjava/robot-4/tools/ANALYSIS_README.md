# 4号機：モデル同定 → PIDチューニング（v2）

## 旧版からの訂正

4885217の「入力－線形減衰＋定数バイアス」モデルと、そのモデルに基づくPDランキングは廃止した。停止中の静止摩擦と動き出しを再現できず、P/Dが不足する摩擦補償を代わりに担う候補を選ぶ問題があった。保持出力をcos＋定数へ当てはめて重力係数として出す旧gravityコマンドも廃止した。静止保持中は摩擦が釣り合う範囲があり、単一の保持出力を重力と同一視しない。

主フローはCAD/実測による角度対応とCSVだけを使う。動画はモデル同定・PID探索の入力でも必須作業でもない。

## 実行

Python 3.10以上、toolsフォルダで実行する。

```powershell
python -m venv .venv
.venv/Scripts/python -m pip install -r analysis_requirements.txt
# 既存CSVの品質確認・電流/電圧/角度のグラフ化
.venv/Scripts/python extend_analysis.py analyze trial.csv --out overview
# 第1段：同定・独立検証。実測に合わせて記入したmanifestを指定
.venv/Scripts/python rebuild_extend_model.py model_manifest.json --out identification_v2
# 第2段：第1段の検証通過モデルのみでPIDと摩擦補償を探索
.venv/Scripts/python tune_extend_pid.py identification_v2/model_report.json trial.properties pid_search.json --out pid_candidates.json
```

旧`extend_analysis.py --fit`（analyzeのオプション）は廃止理由をreport.jsonに書くのみで、旧モデル/ゲインを生成しない。

## 第1段：CSVからモデルを作り、別CSVで検証する

### モデル

単一剛体の回転軸を対象に、物理角度q[deg]と指令相当電圧Vで表す。

- 角度：`q = sensor_scale * angle_deg + sensor_offset_deg`。
- 入力：`V = command_sign * sent_out * motor_Hub_voltage`。出力変更時刻をゼロ次ホールドし、Hub電圧は各測定時刻に復元して、その時点以前のサンプルだけを使う。モーター端子電圧そのものを測っているわけではない。
- 動いている間：`V = kA*q'' + kV*q' + kG*cos(q+phase) + Fc_positive`（正方向）、負方向は最後が`-Fc_negative`。
- 止まっている間：`-Fs_negative <= V - kG*cos(q+phase) <= Fs_positive`なら動かない。これを超えた方向へ動き出し、動摩擦へ切り替わる。
- 摩擦で速度がゼロを跨ぐといったん停止し、次の積分刻みで静止条件を再評価する。単にsign(0)=0として摩擦を消すモデルではない。

Fsは正逆別の静止摩擦/起動閾値、Fcは正逆別の動摩擦。どちらも必須の同定項目。kVは粘性摩擦と逆起電力などをまとめた速度比例項で、純粋な機械摩擦係数とは呼ばない。任意の定数バイアスへ摩擦を押し込めない。

係数の単位はV、V/(deg/s)、V/(deg/s²)。質量・重心・慣性モーメントの独立推定値ではない。モーターのトルク換算・伝達効率等が未確定のため、CSVから制御用の有効係数を推定する。CADの質量/慣性は必須ではない。CAD/実測からは角度対応と重力の角度位相を使う。リンク機構や非線形な角度対応は現実装の対象外で、single_rigid_jointとして偽装せず、角度/重力マップを実装してから使用する。

### データ指定

`model_manifest.example.json`をコピーして編集する。例は形式説明用で、データ数も足りず、そのまま動かす設定ではない。

- geometryのnullをCAD/実測値で埋め、sourceにCADリビジョン/測定根拠を書く。
- motor_hub_voltage_signalは、展開モーターを接続したHubの電圧列を指定する。hub_0とは限らない。
- balls、motor_model、gear_ratioを各CSVと同名の設定スナップショットと一致させる。同じ球数でも配置が違う荷重は混ぜない。
- trainとvalidationを別の記録CSVにする。同じ内容を別名コピーしてもSHA256によって拒否する。
- CSVごとにstart_s/end_s（ログ起点の秒）で区間を指定する。ACTIVE以外・fault・欠損・時刻逆転・100ms超の間隔・古い/不正なHub電圧は拒否する。重複区間を証拠として二重計上しない。
- sliding区間：複数角度・速度・加減速、正逆の走行を用意する。動いている区間のみでkA/kV/kG/Fc正逆を分離する。逆転周辺を回帰から外す。
- breakaway区間：150ms以上の静止から、正または負へ穏やかに入力を増やしたイベント。direction、最後に静止を確認したlast_still_s、ノイズ以上の移動が持続したfirst_motion_sをCSVから指定する。起動時刻の不確かさを閾値の区間として残す。移動前後の情報が不十分なら推定しない。
- trainのbreakawayは正逆各2イベント以上。推定区間が交わらない場合は、角度依存/荷重変化/バックラッシュ等を見直す。FsがFcより小さくなる矛盾も通さない。
- validationには正逆のbreakaway、低速を含むsliding、正逆のreplayを用意する。replayは160ms以上の静止から動き、最後300ms以上停止保持する。

**今のExtend_Tune_R4の±5度操作だけで、このすべての情報が必ず取れるわけではない。** 特に静止摩擦には緩やかな起動入力の記録が必要。現行ログで要件を満たす区間があるか先に確認し、不足時は計測手順/OpModeの追加設計を行う。この変更ではJavaやHubを変更せず、実機試験も開始しない。

静止摩擦の区間は、指定した時刻とeffort_uncertainty_Vを使う実験上の上下限で、統計的信頼区間ではない。モーター電気遅延や位相誤差等を別途評価する。閾値設定例は暫定値で、センサーノイズ実測に合わせて変更する。

### 検証と再作成ループ

出力はmodel_report.json、sliding_residuals.png、replay_N.png。

1. 同定データで動摩擦/重力/加速係数を推定し、静止摩擦を別の起動イベントで推定する。
2. 独立CSVで、電圧残差の方向別・速度帯別評価、角度別残差、動き出し閾値を確認する。
3. 計測した入力を与え、位置を途中で実測へ戻さずに自由再生する。角度誤差、動き出し時刻、停止後のドリフトを評価する。Fs上下限の組み合わせでも再生する。
4. 不一致なら`rebuild_required`、不足なら`needs_data`。次に必要な計測をnext_iterationへ出す。第2段は実行不可。
5. 低速残差が系統的ならStribeck特性、方向反転だけならバックラッシュ/遅延、角度依存なら重力形状や荷重依存摩擦を検討してモデル構造を更新する。現v2は静止/動摩擦切替＋速度比例項を実装しており、連続Stribeck曲線やLuGreを同定するコードは未実装。単に無視してゲイン探索を通すのではなく、低速検証に失敗したモデルを止める。
6. 診断に使ったvalidationで構造を調整した後は、新しい独立検証CSVを追加して最終評価する。改版ごとに別の出力フォルダに保存する。

第1段は常にgain_generation_enabled=false。ゲインを生成する責務は第2段だけにある。第2段への通過条件はstatus=model_validation_passedかつ全validationのpassed=true。

## 第2段：PID＋摩擦補償の候補探索

`pid_search.example.json`をコピーして探索範囲/評価条件を書く。第1段の係数を現行ExtendTuneCoreのシミュレーションへ渡す。

- 探索変数はP、I、D、Javaのfriction。摩擦は固定しない。
- 重力補償は同定したkG/参照電圧とCAD位相から設定し、旧定数バイアスは0にする。
- 現行Javaの誤差微分D、積分上限±0.5、目標10度/秒、8秒試験、出力制限、速度/無動作停止を再現する。無負荷の理想PIDだけを評価するものではない。
- 正逆の起動閾値上下限、学習した電圧範囲の端点、正逆の目標操作を評価する。学習範囲外の角度/指令や保護停止になる候補は棄却する。
- RMSE・終端誤差・最大誤差の条件を通った候補だけを、最悪ケースのRMSE＋終端誤差＋5×飽和率で順位付けする。探索範囲/重みの最適性は保証しない。
- 現行Javaのgravity_cos*cos(sensor_angle+phase)へ厳密に写せるsensor_scale=±1のみ対応する。他の角度倍率では、補償式の対応を先に変更する必要がある。
- **プラントのFs正逆/Fc正逆と、Javaのfriction係数は同じものではない。** 現行Javaのfrictionは誤差/2で立ち上がる1係数。正逆別の静止・動摩擦補償を持つ制御器ではない。許容候補が見つからなければ補償式を改修する。PIDを上げれば解決すると扱わない。

出力はpid_candidates.jsonで、モデル/設定のSHA256、候補値、条件別指標、棄却理由を含む。自動でHubへ書き込まない。指令電圧はモデル入力であり、電流/過熱/Hub再起動の予測はしない。電流・電圧は実測ログの別の評価項目として扱う。SDK遅延や全パラメータの不確かさを保証する試験ではない。

## 動画の位置づけ

track_extend_video.pyは、センサー角度がCAD/実機と食い違う、軸より先のたわみ・遊びを外部観測したい場合だけ使う補助診断。今回のモデル同定/PID探索には不要。通常依存からOpenCVを外した。必要時だけ `pip install -r video_requirements.txt`。既存の動画追跡ファイルは残すが、現地の標準作業には含めない。

## 検証と根拠

リポジトリのonbotjava/robot-4で `python -m unittest discover -s tests -p "test_*.py"`。合成データで静止維持・正逆起動・停止・動摩擦分離・起動閾値・独立検証・CSVからPIDまで・不足時の拒否を検証する。**実機の同定・候補ゲインは未検証**。

モデル設計上の参考（当機体の数値をここから流用してはいない）：
- https://www.mathworks.com/help/sdl/ref/loadedcontacttranslationalfriction.html （静止/滑り状態）
- https://www.mathworks.com/help/simscape/ref/translationalfriction.html （Coulomb・速度比例・Stribeck）
- https://docs.wpilib.org/en/latest/docs/software/advanced-controls/controllers/feedforward.html （重力/摩擦/速度/加速の補償項）
