/*深度センサでは、UART通信を使用しています。（角度センサはI2C通信）
 */
#include "ping1d.h"
#include "VarSpeedServo.h"
VarSpeedServo vss_right;
VarSpeedServo vss_left;
VarSpeedServo vss_up;
VarSpeedServo vss_down;
#define SERVO_NUM_RIGHT 10 //サーボモータの接続ピン番号
#define SERVO_NUM_LEFT 11  //右：10　左：11
static const uint8_t arduinoTxPin = 18;
static const uint8_t arduinoRxPin = 19;

static float water_depth = 0.0;

Ping1D ping{Serial1};

void setup()
{
  Serial1.begin(9600);
  Serial.begin(9600);

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
  water_depth = ping.distance();

  Serial.print("Distance: ");
  Serial.print(ping.distance());
  Serial.print("\tConfidence: ");
  Serial.println(ping.confidence());
}
