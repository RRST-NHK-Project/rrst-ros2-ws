# include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);  // 一応Serialを初期化
  SerialBT.begin("ESP32");
}

void loop() {
  SerialBT.println("Hello World");
  delay(1000);
}
