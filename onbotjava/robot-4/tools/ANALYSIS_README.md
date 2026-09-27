# 4号機 展開ログ・動画のPython解析

PC専用。Hub制御やゲインの書き込みは行わない。実機ログ未取得のため、この解析方法と候補ゲインの実機精度は未検証。

## 準備

Python 3.10以上。toolsフォルダで実行する。

```powershell
python -m venv .venv
.venv/Scripts/python -m pip install -r analysis_requirements.txt
.venv/Scripts/python extend_analysis.py analyze robot4_extend_tune_EXAMPLE.csv --out result
.venv/Scripts/python extend_analysis.py analyze robot4_extend_tune_EXAMPLE.csv --out result_fit --fit
```

CSVと同名の`.properties`（OpModeが生成した設定スナップショット）を隣に置く。手編集した設定例ではなく、その実験時のものを使う。`--properties PATH`で場所を指定可能。`report.json`に設定・入力CSVのSHA256・品質指標・試験別RMSE、`signals.png`に角度・指令出力・電流・電圧を出力する。既存`analyze_extend_tune.py`は標準ライブラリだけでTune/BlueTimingの要約が可能。

電流はseqごとに重複を除き、t_ms-age_msで実測時刻を復元。Hub電流は合算しない。ピークや電源断直前が記録されないことがあり、ログだけで再起動原因を断定できない。

## モデルと候補値

`--fit`は連続ACTIVE区間を分け、最初の十分長い区間で局所モデル `角加速度 = b*出力 - damping*角速度 + bias` を最小二乗推定する。角度は20msへ補間、出力はoutput_t_msに基づくゼロ次ホールド。220msの局所3次多項式で微分し、次の独立したACTIVE区間を自由応答再生で検証する。これは閉ループログからの探索的同定であり、ノイズによる推定の偏り・遅延・バックラッシュ・非線形摩擦は残る。信頼区間や安定性保証を出すものではない。

動き3度未満、入力変動不足、悪条件の回帰、負の減衰、検証RMSE>1度または最大誤差>3度では候補ランキングを出さない。±5度のシミュレーション範囲を学習データが覆う必要もある。閾値は品質スクリーニングの暫定値で安全規格ではない。静止ログや短い1回の試験だけでは同定できない。

候補はP=0..max(0.04,現在Pの2倍)、D=0..0.01の各21点。I=0、FFは記録された設定を維持。Java Tuneの目標スルーレート、誤差微分D、摩擦条件、出力クリップを再現し、±5度を8秒ずつ比較する。スコアはRMSE+5*飽和率+終端誤差。全候補の最適性は保証しない。現在Iが非ゼロなら現行制御との直接比較ではない。電流・電圧・保護停止は予測せず、モデルの適用範囲を超えた候補は採用しない。JSON上位10件は実験候補で、実機へそのまま採用する推奨値ではない。

検証に使わなかった別の角度・荷重・電圧で再検証が必要。荷重や設定の違うデータを混ぜない。応答はスルーレート付きなので理想ステップの公式による立上り時間やZiegler–Nicholsゲインは算出しない。

## 保持出力の角度依存

同じ荷重で、定常保持区間を人が選ぶ。JSON配列を作り、各項目を次の形式にする（時刻はログ先頭からの秒）。csvはこのJSONからの相対パス。

```json
[{"csv":"trial.csv","start_s":3.0,"end_s":4.0}]
```

```powershell
.venv/Scripts/python extend_analysis.py gravity intervals.json --out gravity.json
```

実際には4区間以上・30度以上の角度幅が必要。`u=A*cos(angle)+B*sin(angle)+C`を当てはめ、Javaの振幅・位相・定数形式へ変換する。静止摩擦と制御バイアスを含む保持出力なので、純粋な重力の同定とは言えない。上下両方向から到達した保持を比較して判断する。ここで出たFF値は候補ランキングへ自動適用しない。

## 動画

```powershell
.venv/Scripts/python track_extend_video.py trial.mp4 video_config.example.json --out video_result --log trial.csv
```

固定カメラで動作平面を真横から撮影。支点に緑、腕の先に青のマーカーを置く例。HSV範囲はOpenCVのH=0..179、S/V=0..255。例の値は環境に合わせて調整する。背景に同色があると曖昧なフレームは欠損になる。角度は画面右向きを0度・反時計回りが正。`zero_deg`と`direction`でセンサーの角度基準に合わせる。画面内2点の投影角度であり、3D角度やレンズ補正は行わない。

同期は `ログ秒 = 動画秒 + log_offset_s`。共通イベントを見て手動設定する。自動同期はしない。固定フレームレートの動画専用（可変フレームレートは事前に時間軸を保って変換）。動画の目印欠損を補間しない。`tracked.mp4`で追跡の正否を目視確認し、`video_angles.csv`、`video_overlay.png`、`video_report.json`でログと比較する。動画由来の速度・加速度やゲインは自動算出しない。

## 検証

リポジトリのonbotjava/robot-4から `python -m unittest discover -s tests -p "test_*analysis*.py"`。
合成データのモデル回収・ホールド時刻・励起不足拒否・色追跡・欠損を検証する。合成検証は実機精度を保証しない。
