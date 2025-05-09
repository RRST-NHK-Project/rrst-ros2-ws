

# **ros2udp パッケージ**

## 1. 🚀 概要
**NHK学生ロボコン2025**で使用中！
**ROS 2**とマイコン（NUCLEO-F767ZI、Raspberry Pi Pico）との通信を**UDP**を使用して行うパッケージです。  
このリポジトリのみで機体を構成できるようにマイコン側プログラム、基板データ、スクリプトファイルが含まれています。有線/無線でNUCLEO-F767ZI, Raspberry Pi Picoと通信可能（ルーター必須）です。汎用性を高めるために各種設定はソースから行うようにしています。
本パッケージは、**キャチロボ2024**に向けて開発した[f7_udpパッケージ](https://github.com/RRST-NHK-Project/f7_udp_catchrobo24.git)のC++移植版です。


## 2. ⚙️ 動作環境

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
| `/ros2udp` | ros2udpパッケージ |
| `/ldrobot-lidar-ros2` | LD19用パッケージ。[既存パッケージ](https://github.com/Myzhar/ldrobot-lidar-ros2.git)を改変 |
| `/resources/ros2udp_pio` | マイコン用PlatformIOプロジェクト群 |
| `/resources/NUCLEO_F767ZI_MB` | メイン基板のKiCadデータ |
| `/setup` | 初期設定や立ち上げスクリプト群 |

---

## 5. 🏎️ 機体立ち上げ手順

### 5.1 MR機体の立ち上げ

1. 機体のルーター（GL.iNet）を起動し、PCをアクセスポイントに接続。
2. 以下のスクリプトを実行して機体を立ち上げます。

```bash
cd ~/ros2_ws/src/setup
./boot_mr.sh
```

### 5.2 DR機体の立ち上げ

1. 機体のルーター（TP-Link）を起動し、PCを接続。
2. ラズパイにSSHで接続。

```bash
ssh dev@dev.local
```

3. ログイン後、次のコマンドを実行。

```bash
cd ~/ros2_ws/src/setup
./boot_ld19_fs.sh
```

4. PC側に戻り、以下を実行。

```bash
cd ~/ros2_ws/src/setup
./boot_dr.sh
```

---

## 6. 💻 機体への実装方法

### 6.1 マイコンプログラム書き込み

- NUCLEO-F767ZIにプログラムを書き込む。このときコード内のIPv4アドレスを任意のものに変更する。複数のマイコンを使うときはそれぞれに異なるIPを割り当てる。

### 6.2 ネットワーク接続

- マイコンとルーターをLANケーブルで接続、PCも有線or無線でルーターに接続する。（無線の場合は5GHz帯を推奨）

### 6.3 ROS2側IP設定

- `/src/include/IP.cpp`ファイル内で、宛先IPアドレスを一括設定する。

### 6.4 ROS2ノードの準備

- ROS2ノード内で、`data`配列を使いアクチュエータ制御の指令を記述する。

### 6.5 Run

- ビルドして各ノードを走らせる。  

### 6.6 動作確認

- マイコン側のLANポートのアクセスランプが点滅していれば通信は成功。安全な環境で動作確認を行う。

---

## 7. 📊 配列仕様（メイン基板 V1.3以降）

ROS 2ノードからマイコンに送信される配列`data`は19個の要素を持っています。各要素の詳細をここにまとめます。   
全ての機能を利用するにはメイン基板 V1.3以降が必要です。  
debug: マイコンのprintfを有効化, MD: モータードライバー, TR: トランジスタ

| data[n] | 詳細 | 範囲 |
|:---|:---|:---|
| data[0] | debug | 0 or 1 |
| data[1] | MD1 | -100 ~ 100 |
| data[2] | MD2 | -100 ~ 100 |
| data[3] | MD3 | -100 ~ 100 |
| data[4] | MD4 | -100 ~ 100 |
| data[5] | MD5 | -100 ~ 100 |
| data[6] | MD6 | -100 ~ 100 |
| data[7] | Servo1 | 0 ~ 270 |
| data[8] | Servo2 | 0 ~ 270 |
| data[9] | Servo3 | 0 ~ 270 |
| data[10] | Servo4 | 0 ~ 270 |
| data[11] | TR1 | 0 or 1 |
| data[12] | TR2 | 0 or 1 |
| data[13] | TR3 | 0 or 1 |
| data[14] | TR4 | 0 or 1 |
| data[15] | TR5 | 0 or 1 |
| data[16] | TR6 | 0 or 1 |
| data[17] | TR7 | 0 or 1 |
| data[18] | TR8 | 0 or 1 |

---

## 8. 📚 リファレンス

### 8.1 基本的な使い方

- `data`配列に数値を代入することで、各アクチュエータを制御します。以下はその例です。

| アクション | コード例 |
|:---|:---|
| モーター1を10%で正転 | `data[1] = 10;` |
| モーター1を20%で逆転 | `data[1] = -20;` |
| サーボ2を90度 | `data[8] = 90;` |
| ソレノイド1をON | `data[11] = 1;` |

---

## 9. 🌐 IPアドレス使用状況（NHK 2025）

動的IPに対応していますが、マイコンとのIP競合を防ぐためにIPアドレスの固定を推奨します。

| IP | 機器 | 詳細 |
|:---|:---|:---|
| 192.168.8.1 | ルーター | デフォルトゲートウェイ |
| 192.168.8.191 | PC | imori |
| 192.168.8.193 | PC | ubuntu |
| 192.168.8.195 | PC | dev |
| 192.168.8.197 | 空き | / |
| 192.168.8.199 | 空き | / |
| 192.168.8.205 | ラズパイ4 | 赤 |
| 192.168.8.215 | F767ZI | MR足回り |
| 192.168.8.216 | F767ZI | MR機構 |
| 192.168.8.217 | F767ZI | DR足回り |
| 192.168.8.218 | F767ZI | DR機構 |

---

## 10. 🏷️ 命名規則

- `NR25_` は**NHKロボコン2025**向けコードを意味します。
- 新しいノードを作成する際は、これらのファイルを参考にしてください。

---

## 11. 🛠️ Build Status
各ブランチのビルド状況です。
### 11.1 main（安定版）

[![ROS 2 Jazzy Build](https://github.com/RRST-NHK-Project/ros2udp/actions/workflows/main_jazzy_build_and_test.yml/badge.svg?branch=main)](https://github.com/RRST-NHK-Project/ros2udp/actions/workflows/main_jazzy_build_and_test.yml)

### 11.2 develop（最新版）

[![ROS 2 Jazzy Build](https://github.com/RRST-NHK-Project/ros2udp/actions/workflows/main_jazzy_build_and_test.yml/badge.svg?branch=develop&event=push)](https://github.com/RRST-NHK-Project/ros2udp/actions/workflows/main_jazzy_build_and_test.yml)

---

## 12. 🌟 Powered by

2024年度**立命館大学ロボット技術研究会 NHKプロジェクト**  
2024 NHK Project, RRST, Ritsumeikan University

![Logo](https://www.rrst.jp/img/logo.png)

---
