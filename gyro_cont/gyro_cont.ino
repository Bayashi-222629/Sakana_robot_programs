/*作成こばやし　2022/10/17更新
コメントの()内の文言は、その関数の内容がどのファイルに入っているか示しています。
X軸:ロール、Y軸:ピッチとします。
例「//～(gyro_set_func)」＝gyro_set_func.inoに記述*/

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

const float offset_deg = 0.0;   //モータの自然な角度[deg]
const float deg_max = 135.0;    //モータ角度上限[deg]
const float deg_min = 45.0;     //モータ角度下限[deg]
const int servo_first_deg = 180; //サーボモータの初期角度[deg]
const int servo_speed = 0;     //サーボモータの回転速度
const int sampling = 30;        //角度データのサンプリング回数

float target_deg_max = 95.0;            //許容角度上限
float target_deg_min = 85.0;            //許容角度下限
float target_deg_x = 60.0;              // X軸の目標角度[deg]
float target_deg_y = 60.0;              // Y軸の目標角度[deg]
float kp_x = 0.1, ki_x = 0.002, kd_x = 0.004; // PIDゲイン
float kp_y = 0.1, ki_y = 0.001, kd_y = 0.001;
float x_ctl, y_ctl;

/*------------------------------------------------------------------------------------------------*/

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  motor_standby(servo_first_deg, servo_speed); //モータを初期位置に移動させる(gyro_set_func)
  check_sensor();                              //センサーの接続確認を行う(gyro_set_func)

  delay(3000);
}

/*------------------------------------------------------------------------------------------------*/
void loop()
{

  float x_sum = 0, y_sum = 0, z_sum = 0, x_average, y_average, z_average;

  for (int i = 0; i < sampling; i++)
  {                                      //ノイズ軽減のため、[sampling]回データを取って平均する。
    x_sum = x_sum + ac.getCalculatedX(); //取得データのばらつきが大分マシになる
    y_sum = y_sum + ac.getCalculatedY();
    z_sum = z_sum + ac.getCalculatedZ();
  }

  x_average = x_sum / sampling;
  y_average = y_sum / sampling;
  z_average = z_sum / sampling;
  float x_ang = change_deg(x_average, z_average); //角度をx,zの値から計算する(gyro_set_func)
  float y_ang = change_deg(y_average, z_average); //分度器で測ったところ、±1°くらいに収まった。
  float z_ang = ac.getCalculatedZ();

  vss_right.write(x_ctl, servo_speed, true);

  /*指定した範囲内に角度が収まっていなければPID制御を開始する。*/
  /*if (!((x_ang + target_deg_x) > target_deg_min && (x_ang + target_deg_x) < target_deg_max && (y_ang + target_deg_x) > target_deg_min && (y_ang + target_deg_x) < target_deg_max))
  {
    x_ctl = PID_ctl_x(x_ang);
  }
  else
  {
    PID_reset_x();
  }
  if (!((y_ang + target_deg_y) > target_deg_min && (y_ang + target_deg_y) < target_deg_max && (y_ang + target_deg_y) > target_deg_min && (y_ang + target_deg_y) < target_deg_max))
  {
    y_ctl = PID_ctl_y(y_ang);
  }
  else
  {
    PID_reset_y();
  }*/

  x_ctl = PID_ctl_x(x_ang);
  y_ctl = PID_ctl_y(y_ang);

  // String str = "X:" + String(x_ang) + "," + "Y:" + String(y_ang);            //角度センサが検知した値
  // String str = "P:" + String(P) + ",   " + "I:" + String(I) + ",   " + "D:" + String(D); // pidの各パラメータ値
  // String str = "X:" + String(ctl_deg_x) + ",Y:" + String(ctl_deg_y);                     // pidからでた値
  // String str = "X:" + String(x_ctl) + ",Y:" + String(y_ctl); // pidからでた値を整形した値

  // String graph = "," + (String(up) + "," + String(down));
  // Serial.println(str + graph);

  // delay(10);
}
