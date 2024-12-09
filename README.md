# f7_udp
ROS2 ノードと NUCLEO-F767ZI をUDP経由で通信するパッケージです。  

## 概要 
4輪オムニ試作機向けに開発したパッケージです。キャチロボ2024でも採用、NHKロボコン2025でも使用予定です。
UDP経由でF767ZI（専用プログラム書き込み必須*1）と通信しモータードライバーを駆動することができます。
Mbedのサービス終了、既存のmicroROSに劣るということもあり近いうちに新環境に移行する予定です。

## 実行環境 
Ubuntu 22.04 LTS
ROS2 Humble

## *1 Mbed側の専用プログラム 
https://github.com/KouTashi/f7_udp_mbed_fb.git

エンコーダーの読み取りに必要なQEIライブラリ（Mbed OS 6に対応するように改変）を含みます。
フィードバック制御に対応できるようにエンコーダーから速度(m/s)を求め、UDPで返す機能もあります。（1番と２番のエンコーダーのみ動作確認済み）
UDPで返された速度をROS2ノードで受信＆Publishするノードについてはf7_udpパッケージ内のソースファイル、enc_obs.py を参照してください。

## 実績
キャチロボ2024 ２回生機体
NHKロボコン2025　汎用機、ダンク機

## Powered by
2024年度立命館大学ロボット技術研究会NHKプロジェクト  
2024 NHK Project, RRST, Ritsumeikan University 

![Logo](https://www.rrst.jp/img/logo.png)