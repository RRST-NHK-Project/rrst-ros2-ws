# 概要
ROS 2とマイコンをUDP経由で通信するパッケージとその関連ファイル群です。このリポジトリのみで機体を構成できるようにマイコン側プログラム、基板データ、スクリプトファイルが含まれています。有線/無線でマイコンと通信可能（ルーター必須）です。NUCLEO-F767ZI, Raspberry Pi Picoで動作確認済みです。汎用性を高めるために各種設定はソースから行うようにしています。

# 動作環境
| 環境 | 開発環境 |
| :----: | :----: |
| OS | Ubuntu 24.04 LTS |
| ROS | ROS 2 Jazzy |
| RAM | 16GB以上推奨 |

ビルド中にフリーズしてしまうときはスワップ領域を追加してください。


# ディレクトリ構成
### /ros2udp
[f7_udpパッケージ](https://github.com/KouTashi/f7_udp)のC++移植版、NHKロボコン2025で使用予定。  
### /ldrobot-lidar-ros2
LD19用のパッケージです。既存の[パッケージ](https://github.com/Myzhar/ldrobot-lidar-ros2.git)を改変したものです。
### /resouces/ros2udp_pio
ros2udpパッケージのマイコン側プログラムのPlatformIOプロジェクト群です。
### /resouces/NUCLEO_F767ZI_MB
メイン基板のKiCadデータ
### /setup
初期設定や機体立ち上げのスクリプトをまとめています。

# Getting started
### １，ワークスペースの作成（初回のみ）
  
```
mkdir -p ~/ros2_ws/src
```
  
### ２，リポジトリのクローン
~/ros2_ws/srcに移動しパッケージをクローンします。
```
cd ~/ros2_ws/src
git clone https://github.com/RRST-NHK-Project/ros2udp.git .
```
  
### ３，依存関係のインストール
依存関係を一括インストールするスクリプトを実行します。
```
cd ~/ros2_ws/src/setup
sudo chmod +x setup.sh
./setup.sh
```

### ４,ビルド
ビルドします。コードを編集するたびにビルドが必要です。
```
cd ~/ros2_ws
colcon build
```

# パッケージの詳細


## 機体の立ち上げ（MR）
１，機体のルーターを起動しアクセスポイントに接続する。(GL.iNet)
```
cd ~/ros2_ws/src/setup
./boot_mr.sh
```
## 機体の立ち上げ（DR）
１，機体のルーターを起動しアクセスポイントに接続する。（TP-Link）  
２，SSHで機体のラズパイに接続する（User: dev, Pass: dev）
```
ssh dev@dev.local
```
ログインできたら
```
cd ~/ros2_ws/src/setup
./boot_ld19_fs.sh
```
PCに戻って

```
cd ~/ros2_ws/src/setup
./boot_dr.sh
```

## 機体への実装方法
### １，マイコン側プログラムの書き込み
NUCLEO-F767ZIにプログラムを書き込む。このときコード内のIPv4アドレスを任意のものに変更する。複数のマイコンを使うときはそれぞれに異なるIPを割り当てる。IPの割り当て状況については後述。
### ２，ネットワークに接続する
マイコンとルーターをLANケーブルで接続、PCも有線or無線でルーターに接続する。（無線の場合は5GHz帯を推奨）  
### ３，ROS2側IPの割り当て
/src/include/IP.cpp内で一括でIPアドレスを指定する。
### ４，ROS2ノードの準備
ROS2ノードの配列"data"の各要素は各アクチュエータの出力に対応している。たとえば"data[1] = 10"と代入すれば1番のモーターが10%で回る(1番に接続されたMDにduty比10%が出力される)。逆回転にしたい場合は-10を代入すれば良い。モーター以外にもPWMサーボ、ソレノイドバルブ（電磁弁）、パイロットランプなどの駆動ができる。また、マイコン側でエンコーダーの値を取得しROS 2で受信することもできる。
### ５，Run
ビルドしてノードを走らせる。  
### ６，確認
マイコン側のLANポートのアクセスランプが点滅していれば通信は成功。安全な環境で動作確認を行う。

## 諸注意
・接続しているネットワークをよく確認してから実行する。当然、マイコンと同じネットワークに接続しないと動かない。  
・MC_PRINTFはデバッグ時以外は0に設定する。マイコンのprintfが有効化されていると機構のシーケンスが正しく動きません。
・機体動作中にノードを切らない。機体が暴走する恐れあり。

## IP割り当て状況（NHK2025）
動的IPに対応していますが、マイコンとのIP競合を防ぐためにIPアドレスの固定を推奨します。
| IP | 機器 | 詳細 |
| ---- | ---- | ---- |
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

## 配列要素　（メイン基板V1.3以降）
ROS 2ノードからマイコンに送信される配列'data'は19個の要素を持っています。各要素の詳細をここにまとめます。  
debug: マイコンのprintfを有効化, MD: モータードライバー, TR: トランジスタ
| data[n] | 詳細 | 範囲 |
| ---- | ---- | ---- |
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
| data[11] | TR1 | 0 or 1|
| data[12] | TR2 | 0 or 1|
| data[13] | TR3 | 0 or 1|
| data[14] | TR4 | 0 or 1|
| data[15] | TR5 | 0 or 1|
| data[16] | TR6 | 0 or 1|
| data[17] | TR7 | 0 or 1|
| data[18] | TR8 | 0 or 1|

## 命名規則
NR25_はNHKロボコン2025を表しています。新しいノードを作成する際はこれらの大会向けコードを参考にしてください。

# Build Status
各ブランチのビルド状況です。
### main（安定版）
[![ROS 2 Jazzy Build](https://github.com/RRST-NHK-Project/ros2udp/actions/workflows/main_jazzy_build_and_test.yml/badge.svg?branch=main)](https://github.com/RRST-NHK-Project/ros2udp/actions/workflows/main_jazzy_build_and_test.yml)  
### develop（最新版）
[![ROS 2 Jazzy Build](https://github.com/RRST-NHK-Project/ros2udp/actions/workflows/main_jazzy_build_and_test.yml/badge.svg?branch=develop&event=push)](https://github.com/RRST-NHK-Project/ros2udp/actions/workflows/main_jazzy_build_and_test.yml)

# Powered by
2024年度立命館大学ロボット技術研究会NHKプロジェクト  
2024 NHK Project, RRST, Ritsumeikan University 

![Logo](https://www.rrst.jp/img/logo.png)
