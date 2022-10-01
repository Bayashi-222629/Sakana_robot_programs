/*作成こばやし　2022/10/1更新　*/

#include <Wire.h>
#include "SparkFun_MMA8452Q.h"
MMA8452Q ac;
#include "VarSpeedServo.h"
VarSpeedServo vss;

#define SERVO_NUM 9 //サーボモータの接続ピン番号

const float up = 180, down = -180; //シリアルプロッタの上限と下限の設定

int servo_speed = 80; //サーボモータの回転速度
int servo_first_pos = 90; //サーボモータの初期位置

const int sampling = 10; //角度データのサンプリング回数

const float target = 0.0; //目標値
const float kp = 20; //PID各種ゲイン
const float ki = 0.5;
const float kd = 0.5;


/*----------------------------------------------------------------------------------------*/
void setup() {
  Serial.begin(9600);
  Wire.begin();

  vss.attach(SERVO_NUM);

  /*センサの接続チェック*/
  if (ac.begin() == false) {
    Serial.println("Not Connected. Please check connections and read the hookup guide.");
    while (1);
  }
  vss.write(servo_first_pos, servo_speed, true);
  delay(2000);
}


/*----------------------------------------------------------------------------------------*/
void loop() {
  if (ac.available()) {

    float x_sum = 0, y_sum = 0, z_sum = 0, x_average, y_average, z_average ;

    for (int i = 0; i < sampling; i++) {   //ノイズ軽減のため、[sampling]回データを取って平均する。
      x_sum = x_sum + ac.getCalculatedX(); //取得データのばらつきが大分マシになる！
      y_sum = y_sum + ac.getCalculatedY();
      z_sum = z_sum + ac.getCalculatedZ();
    }
    x_average = x_sum / sampling;
    y_average = y_sum / sampling;
    z_average = z_sum / sampling;

    float x_ang = atan2(x_average , z_average) * 180.0 / PI;  //データを元に角度を算出する。
    float y_ang = atan2(y_average , z_average) * 180.0 / PI;  //atan2はarctanを計算する関数
    float z_ang = ac.getCalculatedZ(); //分度器で測ったところ、±3°くらいに収まった

    Serial.print(x_ang);
    Serial.print("° \t");
    Serial.print(y_ang);
    Serial.print("° \t");

    String str = "xの角度:" + String(x_ang) + "," + "yの角度:" + String(y_ang);
    String graph = ("," + String(up) + "," + String(down));
    Serial.println(str + graph);
    
    int x = round(x_ang)+90;
    
    vss.write(x);

    delay(10);

  }
}
