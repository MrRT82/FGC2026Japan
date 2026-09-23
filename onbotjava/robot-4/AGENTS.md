# 4 号機専用の開発ルール

- この作業ツリーは 4 号機専用。使用ブランチは codex/robot-4、ソースは onbotjava/robot-4/src/org/firstinspires/ftc/teamcode/。
- 作業前に作業ディレクトリとブランチを確認する。他号機の作業ツリーでは編集・ブランチ切り替えをしない。
- 共通開始点は 808316daaab3a69bd88c1874af569c2a013a07cb（codex/robots-3-4）。他号機・共通ブランチ・main・master・TeamCode へ自動反映しない。
- 他号機からの merge / cherry-pick や他号機への反映は、ユーザーの明示的な指示がある場合のみ行う。
- コード変更後は差分確認と可能なビルド・テストを行い、codex/robot-4 にコミットして origin の同名ブランチへpushする。失敗や未検証事項は明示する。強制pushはしない。
- 対象Hubは `dddad6c23c4eeb3b`。書き込み前に接続先シリアルの一致を確認する。
- 従来Hub 1f4562e7e0deafa4 には書き込まない。
- ロボットの走行やOpMode実行は明示された指示なしに開始しない。
- 認証情報、APK、機器ログ、バックアップ全体はGitHubへ追加しない。
- OnBot JavaソースはGradleビルド対象外。Gradle成功をこのソースのビルド成功として扱わない。
