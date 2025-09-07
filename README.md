

# **rrst-ros2-workspace**
ソフト班はdevelopブランチを使用すること。
### main（安定版）

[![ROS 2 Jazzy Build](https://github.com/RRST-NHK-Project/rrst-ros2-ws/actions/workflows/main_jazzy_build_and_test.yml/badge.svg?branch=main)](https://github.com/RRST-NHK-Project/rrst-ros2-ws/actions/workflows/main_jazzy_build_and_test.yml)

### develop（最新版）

[![ROS 2 Jazzy Build](https://github.com/RRST-NHK-Project/rrst-ros2-ws/actions/workflows/main_jazzy_build_and_test.yml/badge.svg?branch=develop&event=push)](https://github.com/RRST-NHK-Project/rrst-ros2-ws/actions/workflows/main_jazzy_build_and_test.yml)

### Docker
[![Build and Push Docker Image](https://github.com/RRST-NHK-Project/rrst-ros2-ws/actions/workflows/docker-publish.yml/badge.svg?branch=develop)](https://github.com/RRST-NHK-Project/rrst-ros2-ws/actions/workflows/docker-publish.yml)

## 1. 🚀 概要
RRST, NHKプロジェクトのROS 2 ワークスペースです。現在使用中の各種パッケージ、回路データ、マイコン側プログラムを格納しています。不要になったものは別リポジトリに移動しアーカイブしてください。


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
作成済みの場合は3.2へ
```bash
mkdir -p ~/ros2_ws/src
```

### 3.2 📥 リポジトリのクローン

main（安定版）
```bash
cd ~/ros2_ws/src
```
```bash
git clone https://github.com/RRST-NHK-Project/rrst-ros2-ws.git .
git submodule update --init --recursive
```

develop（最新版）
```bash
cd ~/ros2_ws/src
```
```bash
git clone https://github.com/RRST-NHK-Project/rrst-ros2-ws.git -b develop .
git submodule update --init --recursive
```

ソフト班向け
```bash
cd ~/ros2_ws/src
```
```bash
git clone git@github.com:RRST-NHK-Project/rrst-ros2-ws.git -b develop .
git submodule update --init --recursive
```


### 3.3 🛠️ ビルド

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
| `/ros2esp` | ros2espパッケージ、詳細はパッケージ内のReadmeを参照 |
| `/microcontrollerーws` | マイコン用プログラム |
| `/kicad-ws` | メイン基板のKiCadデータ |
---

## 5. 🌟 Powered by

2024年度立命館大学ロボット技術研究会 NHKプロジェクト  
2024, NHK Project, RRST, Ritsumeikan University

![Logo](https://www.rrst.jp/img/logo.png)

---
