# 3・4号機：OnBot Java

2026-09-23に従来Hubから新Hubへそのままコピーした22個のJavaファイルを保存しています。

- ソース: `src/org/firstinspires/ftc/teamcode/`
- 対象Hub: `dddad6c23c4eeb3b`
- Robot Controller: 11.1
- 新Hub上で22ファイルのビルド成功、更新前後のSHA-256完全一致を確認済み。
- MainClass_Blue、MainClass_Red、関連クラス、pot_angle_test、controllerCheckerを含みます。
- 実機動作は未検証。新Hubの機器構成は未設定です。

このフォルダーはOnBot Java用の独立ソースです。既存のAndroid Studio用TeamCodeとは自動連携しません。Gradleビルドにはこのフォルダーは含まれません。

今後はこのブランチで3・4号機を開発し、従来機側への変更適用は別途判断します。3号機と4号機をそれぞれ異なるコードにする場合は、さらに開発系統を分けてください。
