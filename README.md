

# **rrst-ros2-workspace**

## 1. 🚀 概要
RRST, NHKプロジェクトのROS 2 ワークスペースです。現在使用中の各種パッケージ、回路データなどを格納しています。不要になったパッケージやデータは別リポジトリに移動してアーカイブしてください。


## 2. ⚙️ 動作環境
以下の環境での使用を想定しています。
| 項目 | 内容 |
|:---|:---|
| OS | Ubuntu 24.04 LTS |
| ROS | ROS 2 Jazzy |
| RAM | 16GB以上推奨 |

> 💡 **注意**: ビルド中にフリーズする場合は、RAMが足りていない可能性があります。スワップ領域を追加すると解決します。

---

## 3. 🛠️ Getting Started

### 3.1 📝 ワークスペースの作成

```bash
mkdir -p ~/ros2_ws/src
```

### 3.2 📥 リポジトリのクローン

```bash
cd ~/ros2_ws/src
git clone https://github.com/RRST-NHK-Project/ros2udp.git .
```

### 3.3 🔧 依存関係のインストール
パッケージの追加などで依存関係が増えた場合はスクリプトに追加してください。
```bash
cd ~/ros2_ws/src/setup
sudo chmod +x setup.sh
./setup.sh
```

### 3.4 🛠️ ビルド

```bash
cd ~/ros2_ws
colcon build
```

---

## 4. 📁 ディレクトリ構成

| パス | 説明 |
|:---|:---|
| `/example` | 講習関連 |
| `/ros2udp` | ros2udpパッケージ、詳細はパッケージ内のReadmeを参照 |
| `/ros2udp_core` | ros2udpパッケージの最小構成 |
| `/ldrobot-lidar-ros2` | LD19用パッケージ。[既存パッケージ](https://github.com/Myzhar/ldrobot-lidar-ros2.git)を改変 |
| `/resources/microcontroller` | マイコン用プログラム |
| `/resources/mainboard` | メイン基板のKiCadデータ |
| `/setup` | 初期設定や立ち上げスクリプト群 |

---

## 11. 🛠️ Build Status
各ブランチのビルド状況です。
### 11.1 main（安定版）

[![ROS 2 Jazzy Build](https://github.com/RRST-NHK-Project/ros2udp/actions/workflows/main_jazzy_build_and_test.yml/badge.svg?branch=main)](https://github.com/RRST-NHK-Project/ros2udp/actions/workflows/main_jazzy_build_and_test.yml)

### 11.2 develop（最新版）

[![ROS 2 Jazzy Build](https://github.com/RRST-NHK-Project/ros2udp/actions/workflows/main_jazzy_build_and_test.yml/badge.svg?branch=develop&event=push)](https://github.com/RRST-NHK-Project/ros2udp/actions/workflows/main_jazzy_build_and_test.yml)

---

## 12. 🌟 Powered by

2024年度立命館大学ロボット技術研究会 NHKプロジェクト  
2024 NHK Project, RRST, Ritsumeikan University

![Logo](https://www.rrst.jp/img/logo.png)

---
