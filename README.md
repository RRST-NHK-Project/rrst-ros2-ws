# 概要
ROS 2とマイコンをUDP経由で通信するパッケージ群です。有線/無線でマイコンと通信可能（ルーター必須）です。NUCLEO-F767ZI, Raspberry Pi Picoで動作確認済みです。

## ディレクトリ構成
### -ros2udp
f7_udpパッケージのC++移植版、NHKロボコン2025で使用予定。  
2024/04/07: f7_udpを削除しf7_udp_cpp, ros2udpを統合しました。
### -resouces/ros2udp_pio
ros2udpパッケージのマイコン側プログラムのPlatformIOプロジェクト群です。
### -resouces/NUCLEO_F767ZI_MB
メイン基板のKiCadデータ

## Getting started
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
  
### ３,ビルド
ビルドします。コードを編集するたびにビルドが必要です。
```
cd ~/ros2_ws
colcon build
```

### ４，依存関係のインストール
依存関係を一括インストールするスクリプトを実行します。
```
cd ~/ros2_ws/src/setup
sudo chmod +x setup.sh
./setup.sh

```

## Build & Test Status
各ブランチのビルド状況です。2024/04/09~ テストのみ無効化中
### main
[![ROS 2 Jazzy Build & Test](https://github.com/RRST-NHK-Project/ros2udp/actions/workflows/main_jazzy_build_and_test.yml/badge.svg?branch=main)](https://github.com/RRST-NHK-Project/ros2udp/actions/workflows/main_jazzy_build_and_test.yml)  
### develop
[![ROS 2 Jazzy Build & Test](https://github.com/RRST-NHK-Project/ros2udp/actions/workflows/main_jazzy_build_and_test.yml/badge.svg?branch=develop&event=push)](https://github.com/RRST-NHK-Project/ros2udp/actions/workflows/main_jazzy_build_and_test.yml)
