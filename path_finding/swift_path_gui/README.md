# swift_path_gui

添付された RRST-CONSOLE のデザイン言語を参考にした SwiftUI GUI です。

## 要件対応

- 左側に 3x4 マス (1..12)
- 右側に `logo` / `❍` / `✖` ボタンを縦配置
- タップで選択マークを配置
- 制約
  - logo は 3 個まで
  - ❍ は 4 個まで
  - ✖ は 1 個まで
  - ❍ は 5 番・8 番禁止
- 8 個配置後に経路探索実行
- 矢印をアニメーション表示

## C++ 連携

`cpp_path_finding/main.cpp` に GUI 用 CLI を追加済み。

```bash
cd /home/ubuntu/rrst-ros2-ws/path_finding/cpp_path_finding
./cpp_path_finding --solve-markers 1,2,4,7,9,10,11,12
```

出力例:

```text
PATH:1,2,5,4,7,10,11,12,9
```

Swift 側はこの `PATH:` 形式を読んで、実行可能なら C++ 結果を優先利用します。
失敗時は Swift 内蔵の同等ロジックへフォールバックします。
