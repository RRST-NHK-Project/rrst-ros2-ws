# **rrst-ros2-workspace**

**日本語** | [English](README.en.md)

## 1. 概要
NHK プロジェクトのメインリポジトリです。現在使用している ROS 2 パッケージを管理しています。大会ごとにこのリポジトリをforkして使用することを想定しています。

|  | 説明 | ビルド状況 |
|---|---|---|
| **main** | 安定版ブランチ（動作確認済み） | [![ROS 2 Jazzy Build](https://github.com/RRST-NHK-Project/rrst-ros2-ws/actions/workflows/main_jazzy_build_and_test.yml/badge.svg?branch=main)](https://github.com/RRST-NHK-Project/rrst-ros2-ws/actions/workflows/main_jazzy_build_and_test.yml) |
| **develop** | 最新の開発ブランチ（開発中） | [![ROS 2 Jazzy Build](https://github.com/RRST-NHK-Project/rrst-ros2-ws/actions/workflows/main_jazzy_build_and_test.yml/badge.svg?branch=develop&event=push)](https://github.com/RRST-NHK-Project/rrst-ros2-ws/actions/workflows/main_jazzy_build_and_test.yml) |
| **Docker** | Docker ビルド（develop） | [![Docker Build](https://github.com/RRST-NHK-Project/rrst-ros2-ws/actions/workflows/docker-publish.yml/badge.svg?branch=develop)](https://github.com/RRST-NHK-Project/rrst-ros2-ws/actions/workflows/docker-publish.yml) |

---

## 2. 動作環境

| 項目 | 内容 |
|:---|:---|
| OS | Ubuntu 24.04 LTS |
| ROS | ROS 2 Jazzy |
| メモリ | 16 GB 以上推奨 |

> **注意**: ビルド中にフリーズする場合は、メモリ不足の可能性があります。  
> スワップ領域を追加するか、ビルドのスレッド数を制限してください。

---

## 3. はじめに

### 3.1 ワークスペースの作成
作成済みの場合はスキップしてください。

    mkdir -p ~/ros2_ws/src

### 3.2 SSH 鍵の登録
リポジトリ・サブモジュールとも **SSH** で運用しています。登録済みの場合はスキップしてください。

    ssh-keygen -t ed25519 -C "your_email@example.com"
    cat ~/.ssh/id_ed25519.pub

表示された公開鍵を GitHub の [Settings → SSH and GPG keys](https://github.com/settings/keys) で **New SSH key** から登録し、接続を確認します。

    ssh -T git@github.com

`Hi <ユーザー名>! You've successfully authenticated...` と表示されれば完了です。詳しくは [Wiki](https://github.com/RRST-NHK-Project/rrst-ros2-ws/wiki/SSH鍵の登録) を参照してください。

### 3.3 リポジトリのクローン

**main（安定版）**

    cd ~/ros2_ws/src
    git clone git@github.com:RRST-NHK-Project/rrst-ros2-ws.git .
    git submodule update --init --recursive

**develop（最新・開発中）**

    cd ~/ros2_ws/src
    git clone git@github.com:RRST-NHK-Project/rrst-ros2-ws.git -b develop .
    git submodule update --init --recursive

### 3.4 ビルド

    cd ~/ros2_ws
    colcon build

ビルドのスレッド数を指定する場合:

    cd ~/ros2_ws
    colcon build --parallel-workers 4

---

## 4. クレジット
Developed by NHK Project, RRST, Ritsumeikan University, Japan (2024)
- 公式サイト: https://www.rrst.jp
- X (Twitter): https://x.com/RRST_BKC

![Logo](https://www.rrst.jp/img/logo.png)

---
