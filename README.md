# f7_udp ROS2パッケージ
ROS2 ノードと NUCLEO-F767ZI をUDP経由で通信するパッケージです。
ROS2 Package for connecting ROS2 node and NUCLEO-F767ZI over UDP protocol.

## 概要 / Overview
4輪オムニ試作機向けに開発したパッケージです。キャチロボ2024でも採用、NHKロボコン2025でも使用予定です。
UDP経由でF767ZI（専用プログラム書き込み必須*1）と通信しモータードライバーを駆動することができます。
Mbedのサービス終了、既存のmicroROSに劣るということもあり近いうちに新環境に移行する予定です。

This package was developed for a 4-wheel omni-directional prototype. It has been adopted for Catchrobo 2024 and is planned to be used in NHK Robocon 2025.
It can communicate with F767ZI (requires specific program installation*1) via UDP to drive motor drivers.
Due to the end of Mbed service and its inferiority to existing microROS, we plan to transition to a new environment in the near future.

## 実行環境 / Execution Environment
Ubuntu 22.04 LTS
ROS2 Humble

## *1 Mbed側の専用プログラム / *1 Dedicated Program for Mbed
https://github.com/KouTashi/f7_udp_mbed_fb.git

エンコーダーの読み取りに必要なQEIライブラリ（Mbed OS 6に対応するように改変）を含みます。
フィードバック制御に対応できるようにエンコーダーから速度(m/s)を求め、UDPで返す機能もあります。（1番と２番のエンコーダーのみ動作確認済み）
UDPで返された速度をROS2ノードで受信＆Publishするノードについてはf7_udpパッケージ内のソースファイル、enc_obs.py を参照してください。

This includes the QEI library necessary for reading encoders (modified to be compatible with Mbed OS 6).
It also has the capability to calculate velocity (m/s) from encoders for feedback control and return it via UDP. (Only encoders 1 and 2 have been verified to work)
For the ROS2 node that receives and publishes the velocity returned via UDP, please refer to the source file enc_obs.py in the f7_udp package.

## Powered by
2024年度立命館大学ロボット技術研究会NHKプロジェクト
2024 NHK Project, RRST, Ritsumeikan University 

![Logo](https://www.rrst.jp/img/logo.png)