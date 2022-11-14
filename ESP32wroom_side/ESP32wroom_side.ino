/*作成こばやし　2022/11/6更新　*/
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

int RX_PIN = 22; // ESP32のRT,TXピン配置(ピン位置変更不可)
int TX_PIN = 23;
String BT_rec_Data;  // Bluetoothから受信した文字
String Ard_rec_Data; // arduinoとのUART通信から受信した文字

void setup()
{
  Serial.begin(115200);                            // BTやAndroidスマホへ無線接続する際のbitレート
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Arduinoへ有線接続する際のbitレート
  SerialBT.begin("TDUNA_Computer");                // 無線接続する際に表示する名前（なんでもOK）
}

void loop()
{
  /*Bluetoothからの受信*/
  if (SerialBT.available()) // Bluetoothでデータが届いていたら以下を実行
  {
    BT_rec_Data = SerialBT.readStringUntil(';'); //「;」までデータを受信する指示

    Serial.println(BT_rec_Data); // Serial(ArduinoIDEなどのUSB接続先)へデータをオウム返しする。
    SerialBT.println("-ESP32-");
    SerialBT.println(BT_rec_Data);    // SerialBT(PCやAndroidスマホのBluetooth接続先)へデータをオウム返しする。
    Serial1.print(BT_rec_Data + ";"); // Serial1(ArduinoのTx1ピンRx0ピン)へデータを送信する。
  }

  /*Arduinoからの受信*/
  if (Serial1.available()) // ArduinoとのUART通信でデータが届いていたら以下を実行
  {
    Ard_rec_Data = Serial1.readStringUntil(';');

    Serial.println(Ard_rec_Data);
    SerialBT.println(Ard_rec_Data);
    // Serial1.println(Ard_rec_Data);
  }
}