/*
RRST-NHK-Project 2025
宛先IPアドレスを一括で設定する
*/

#include "IP.hpp"

// 標準
#include <iostream>

// 送信先IPアドレス、宛先マイコンで設定したIPv4アドレスを指定
const std::string IP_NATSU_SD = "192.168.8.215";
const std::string IP_NATSU = "192.168.8.216";
const std::string IP_TEST = "192.168.128.205";

// 送信先ポート番号、宛先マイコンで設定したポート番号を指定
const int PORT_NATSU_SD = 5000;
const int PORT_NATSU = 5000;
const int PORT_TEST = 5000;