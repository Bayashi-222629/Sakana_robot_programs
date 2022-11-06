#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
int RX_PIN = 22;  // ESP32のRT,TXピン配置(変更不可!)
int TX_PIN = 23;
String rec_Data;  //受信した文字
String sen_Data;  //送信した文字

void setup() {
  Serial.begin(115200);                             // PCやAndroidスマホへ無線接続する際のbitレート
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);  // Arduinoへ有線接続する際のbitレート
  SerialBT.begin("Sakana");                         // 無線接続する際に表示する名前（なんでもOK）
}

void loop() {

  if (SerialBT.available())  //データ受信プログラム
  {
    rec_Data = SerialBT.readStringUntil(';');  //「;」までデータを受信する指示
    Serial.println(rec_Data);                  // Serial(PCやAndroidスマホ)へデータをオウム返しする。
    Serial1.println(rec_Data);                 // Serial1(Arduino)へデータを送信する。
  }

  if (Serial.available())  //データ送信プログラム
  {
    sen_Data = Serial.readStringUntil(';');
    SerialBT.print(sen_Data);
  }
}