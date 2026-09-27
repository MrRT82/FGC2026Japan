# 4 号機：OnBot Java

2026-09-27: 展開専用 `Extend_Tune_R4` とBlue計測版の全機構電源ログを追加。
手順・停止条件・未配布事項は [EXTEND_TUNING.md](EXTEND_TUNING.md) を参照。
通常版はユーザー更新1547d2aのまま。計測版の角度・Feed方向も4号機設定へ同期済み。

専用ブランチ: codex/robot-4
専用作業フォルダー: github_FGC2026Japan_robot4
ソース: src/org/firstinspires/ftc/teamcode/

2026-09-23に共通ブランチ codex/robots-3-4 のコミット 808316d から分離。
22個のJavaファイルは共通開始点と内容を変えずに移動した。
他号機とは作業ツリー・ブランチ・ソースパスを分離しており、変更は自動同期しない。
開発ルールはリポジトリ直下とこのディレクトリの AGENTS.md を参照。

4号機Hubは dddad6c23c4eeb3b（2026-09-23のユーザー指定と接続シリアル照合で確定）。3号機Hubは 1f4562e7e0deafa4。

引き継ぎ時の共通ソースはRobot Controller 11.1の新Hub上でビルド成功と記録されている。
分離後の実機動作は未検証。機器構成の最新状態は接続時に確認する。
既存Android Studio用TeamCodeとは独立しており、このソースはGradleビルドに含まれない。

通常Blue/Redを維持したまま、3号機と同じ独立した MainClass_Blue_Timing を追加。計測の使い方と制約は [TIMING.md](TIMING.md) を参照。

4号機Hubへの配布・ビルド・登録確認の記録: [DEPLOYMENT.md](DEPLOYMENT.md)。

展開の解析は「静止/動摩擦を含むモデル同定・独立検証 → PIDと摩擦補償の探索」の2段構成。
[解析手順](tools/ANALYSIS_README.md)を参照。旧線形モデルによる候補ランキングは廃止。動画は任意の補助診断。
