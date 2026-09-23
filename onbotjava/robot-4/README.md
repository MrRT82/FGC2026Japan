# 4 号機：OnBot Java

専用ブランチ: codex/robot-4
専用作業フォルダー: github_FGC2026Japan_robot4
ソース: src/org/firstinspires/ftc/teamcode/

2026-09-23に共通ブランチ codex/robots-3-4 のコミット 808316d から分離。
22個のJavaファイルは共通開始点と内容を変えずに移動した。
他号機とは作業ツリー・ブランチ・ソースパスを分離しており、変更は自動同期しない。
開発ルールはリポジトリ直下とこのディレクトリの AGENTS.md を参照。

対象Hubは `dddad6c23c4eeb3b`。書き込み前に接続先シリアルの一致を確認する。

引き継ぎ時の共通ソースはRobot Controller 11.1の新Hub上でビルド成功と記録されている。
分離後の実機動作は未検証。機器構成の最新状態は接続時に確認する。
既存Android Studio用TeamCodeとは独立しており、このソースはGradleビルドに含まれない。
