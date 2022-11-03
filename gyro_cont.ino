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

static const float up = 180, down = 0; //シリアルプロッタの上限と下限の設定
static const float pi = 3.141592653589793;
float deg_g = 0.0;
float input, output;
float x_ctl, y_ctl;

const float offset_deg = 0.0;   //モータの自然な角度[deg]
const float deg_max = 135.0;    //モータ角度上限[deg]
const float deg_min = 45.0;     //モータ角度下限[deg]
const int servo_first_deg = 90; //サーボモータの初期角度[deg]
const int servo_speed = 0;      //サーボモータの回転速度(0で最高速度)
const int sampling = 30;        //角度データのサンプリング回数(センサの値飛びの防止)

float target_deg_max = 91.0; //許容角度上限
float target_deg_min = 89.0; //許容角度下限

/*------------------------------------------------------------------------------------------------*/

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  motor_standby(servo_first_deg, servo_speed); //モータを初期位置に移動させる(gyro_set_func)
  check_sensor();                              //センサーの接続確認を行う(gyro_set_func)

  delay(1000);
}

/*------------------------------------------------------------------------------------------------*/
void loop()
{
  // target_deg_x = deg_generater1(); sin波を再現する(gyro_set_func)
  // target_deg_x = deg_generater2(); ステップ入力を再現する(gyro_set_func)

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
  float x_ang = change_deg(x_average, z_average) + 90; //角度をx,zの値から計算する(gyro_set_func)
  float y_ang = change_deg(y_average, z_average) + 90; //分度器で測ったところ、±1°くらいに収まった。
  float z_ang = ac.getCalculatedZ();

  // x_ang = 0.0;
  // x_ang = deg_generater1(); sin波を再現する(gyro_set_func)
  // x_ang = deg_generater2(); ステップ入力を再現する(gyro_set_func)
  // x_ctl = PID_ctl_x(x_ang); x_angを元にPID制御を行う(gyro_pid)
  // y_ctl = PID_ctl_y(y_ang); y_angを元にPID制御を行う(gyro_pid)

  /*指定した範囲内に角度が収まっていなければPID制御を開始する。*/
  int flag = 0;
  if (!(x_ang > target_deg_min && x_ang < target_deg_max))
  {
    x_ctl = PID_ctl_x(x_ang);
    flag++;
  }
  else
    PID_reset_x();

  if (!(y_ang > target_deg_min && y_ang < target_deg_max))
  {
    y_ctl = PID_ctl_y(y_ang);
    flag++;
  }
  else
    PID_reset_y();

  if (flag > 0)
  {
    fillet_right(x_ctl, y_ctl, 0);
    fillet_left(x_ctl, y_ctl, 0);
    fillet_up(x_ctl, y_ctl, 0);
    fillet_down(x_ctl, y_ctl, 0);
  }

  // Serial.println(String(target_deg_x) + "," + String(x_ang) + "," + String(x_ctl));
  //  Serial.println("角度センサ:" + String(x_ang) + "," + "制御波形:" + String(x_ctl + 90) + "," + "目標値:" + String(target_deg_x));
  Serial.println("ロール(X):" + String(x_ang) + "," + "ピッチ(Y):" + String(y_ang) + "," + "ロールctl(Y):" + String(x_ctl) + "," + "ピッチctl(Y):" + String(y_ctl) + "," + "90°:" + String(90));

  // String graph = "," + (String(up) + "," + String(down));
  // Serial.println(str + graph);

  delay(5);
}