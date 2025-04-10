/*
RRST-NHK-Project 2025
宛先IPアドレスを一括で設定する
*/

#include "IP.hpp"

// 標準
#include <iostream>

// 送信先IPアドレス、宛先マイコンで設定したIPv4アドレスを指定
const std::string IP_MR_SD = "192.168.8.215";
const std::string IP_MR = "192.168.8.216";
const std::string IP_DR_SD = "192.168.0.217";
const std::string IP_DR = "192.168.0.218";

// 送信先ポート番号、宛先マイコンで設定したポート番号を指定
const int PORT_MR_SD = 5000;
const int PORT_MR = 5000;
const int PORT_DR_SD = 5000;
const int PORT_DR = 5000;