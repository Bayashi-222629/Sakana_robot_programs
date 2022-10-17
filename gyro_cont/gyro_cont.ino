/*作成こばやし　2022/10/17更新　*/

#include <Wire.h>
#include "SparkFun_MMA8452Q.h"
MMA8452Q ac;
#include "VarSpeedServo.h"
VarSpeedServo vss_right;
VarSpeedServo vss_left;
VarSpeedServo vss_up;
VarSpeedServo vss_down;
#define SERVO_NUM_RIGHT 10 //サーボモータの接続ピン番号
#define SERVO_NUM_LEFT 11  //右：10　左：11　上：12　下：13
#define SERVO_NUM_UP 12
#define SERVO_NUM_DOWN 13

const float up = 180, down = -180; //シリアルプロッタの上限と下限の設定

const float offset_deg = 0.0;  //モータの自然な角度
const float max_deg = 180.0;   //モータ角度上限
const float min_deg = 0.0;     //モータ標角度下限
const int servo_first_deg = 0; //サーボモータの初期角度
const int servo_speed = 80;    //サーボモータの回転速度
const int sampling = 20;       //角度データのサンプリング回数

float target_deg = 90.0;     //目標角度
float target_deg_max = 95.0; //許容角度上限
float target_deg_min = 85.0; //許容角度下限
float deg = 0.0, ctl_deg = 0.0, output_deg = 0.0;
float kp = 2, ki = 0.5, kd = 0.1; // PID各種ゲイン
float P, I, D;
float dt = 0.0, pre_dt = 0.0, pre_P = 0.0;

/*------------------------------------------------------------------------------------------------*/

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  motor_standby(servo_first_deg, servo_speed);
  check_sensor();

  delay(2000);
}

/*------------------------------------------------------------------------------------------------*/
void loop()
{

  float x_sum = 0, y_sum = 0, z_sum = 0, x_average, y_average, z_average;

  for (int i = 0; i < sampling; i++)
  {                                      //ノイズ軽減のため、[sampling]回データを取って平均する。
    x_sum = x_sum + ac.getCalculatedX(); //取得データのばらつきが大分マシになる！
    y_sum = y_sum + ac.getCalculatedY();
    z_sum = z_sum + ac.getCalculatedZ();
  }

  x_average = x_sum / sampling;
  y_average = y_sum / sampling;
  z_average = z_sum / sampling;
  float x_ang = change_deg(x_average, z_average); //角度をx,zの値から計算するよ！
  float y_ang = change_deg(y_average, z_average); //分度器で測ったところ、±1°くらいに収まった。
  float z_ang = ac.getCalculatedZ();

  // vss_right.write(ctl_deg, servo_speed, true);

  if (!((x_ang + target_deg) > target_deg_min && (x_ang + target_deg) < target_deg_max && (y_ang + target_deg) > target_deg_min && (y_ang + target_deg) < target_deg_max))
  {

    dt = (micros() - pre_dt) / 1000000; //疑似の微小時間
    pre_dt = micros();
    P = offset_deg - x_ang; //偏差
    I += P * dt;            //積分項（偏差*微小時間）
    D = (P - pre_P) / dt;   //微分項（傾き/微小時間）

    pre_P = P;

    ctl_deg += (P * kp) + (I * ki) + (D * kd); // 0を中心にどのくらい変化させるか？
    output_deg = ctl_deg + target_deg;         //ターゲット角度まで増加させただけ
    if (output_deg > max_deg)
    {
      ctl_deg = max_deg;
      output_deg = max_deg;
    }
    else if (output_deg < min_deg)
    {
      ctl_deg = min_deg;
      output_deg = min_deg;
    }

    String str = "xの角度:" + String(x_ang) + "," + "yの角度:" + String(y_ang); //シリアルモニタ表示用のメッセージ
    // String str = "P:" + String(P) + ",   " + "I:" + String(I) + ",   " + "D:" + String(D);
    //  String str = "output:" + String(output_deg) + "ctl:" + String(ctl_deg) + "target:" + String(target_deg);
    String graph = (String(up) + "," + String(down));
    Serial.println(str);
  }
  // delay(10);
}
