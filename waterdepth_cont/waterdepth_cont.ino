#include "ping1d.h"
#include "VarSpeedServo.h"
VarSpeedServo vss_right;
VarSpeedServo vss_left;
VarSpeedServo vss_up;
VarSpeedServo vss_down;
#define SERVO_NUM_RIGHT 10 //サーボモータの接続ピン番号
#define SERVO_NUM_LEFT 11  //右：10　左：11　上：12　下：13
#define SERVO_NUM_UP 12
#define SERVO_NUM_DOWN 13
static const uint8_t arduinoTxPin = 16;
static const uint8_t arduinoRxPin = 17;

Ping1D ping{Serial1};

void setup()
{
  Serial1.begin(9600);
  Serial.begin(9600);
  Serial.println("Blue Robotics ping1d-simple.ino");

  while (!ping.initialize())
  {
    Serial.print("送信用の緑ピン（TX）の接続が確認できません!");
    Serial.println(arduinoTxPin);
    Serial.print("受信用の白ピン（RX）の接続が確認できません!");
    Serial.println(arduinoRxPin);
    delay(2000);
  }
}

void loop()
{
  if (ping.update())
  {
    Serial.print("Distance: ");
    Serial.print(ping.distance());
    Serial.print("\tConfidence: ");
    Serial.println(ping.confidence());
  }
  else
  {
    Serial.println("No update received!");
  }
}
