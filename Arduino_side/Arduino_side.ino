/*作成こばやし　2022/11/11更新　
コメントの()内の文言は、その関数の内容がどのファイルに入っているか示しています。
X軸:ロール、Y軸:ピッチとします。
例「//～(other_setting.ino)」＝other_setting.inoに記述*/

#include <Wire.h>              //シリアル通信用ライブラリ
#include "SparkFun_MMA8452Q.h" //加速度センサ読み取りライブラリ
MMA8452Q ac;
#include "VarSpeedServo.h" //サーボモータ駆動用ライブラリ
VarSpeedServo vss_right;
VarSpeedServo vss_left;
VarSpeedServo vss_up;
VarSpeedServo vss_down;
#include "ping1d.h"        //深度センサ読み取り用ライブラリ
#define SERVO_NUM_RIGHT 10 //サーボモータのArduino接続ピン*変更可
#define SERVO_NUM_LEFT 11  //右：10　左：11　上：12　下：13
#define SERVO_NUM_UP 12
#define SERVO_NUM_DOWN 13
static const uint8_t arduinoTxPin = 18; // Arduinoのソナー用txピン（緑）*ライブラリの都合上変更不可
static const uint8_t arduinoRxPin = 19; // Arduinoのソナー用rxピン（白）*ライブラリの都合上変更不可
static Ping1D ping{Serial1};

static const int up = 180, down = -180; //シリアルプロッタの上限と下限の設定
float deg_max = 135.0;                  //モータ角度上限[deg]
float deg_min = 45.0;                   //モータ角度下限[deg]
const int servo_first_deg = 90;         //サーボモータの初期角度[deg]
const int servo_speed = 0;              //サーボモータの回転速度(0で最高速度)
const int sampling = 30;                //角度データのサンプリング回数(センサのノイズによる値飛びの緩和)
const float target_deg_max = 91.0;      //機体の安定判定の角度上限（この角度内に機体が収まれば姿勢制御をしない）
const float target_deg_min = 89.0;      //機体の安定判定の角度下限

//目標値380mmだが45mmの誤差があるので425mm　最低345mm 最高505mm その間160mm 上下各80mm
float sensor_position_error = 45.0;                                 //深さセンサ配置位置から機体重心までの距離[mm]
float target_depth = 380;                                           //目標深度[mm]
float target_depth_max = target_depth + sensor_position_error + 80; //深度上限[mm]
float target_depth_min = target_depth - 80;                         //深度下限[mm]

float kp_x = 1.25, ki_x = 0.01, kd_x = 0.025; // xPIDゲイン
float kp_y = 1.25, ki_y = 0.01, kd_y = 0.025; // yPIDゲイン
float target_deg_x = 90.0;                    // X軸の目標角度[deg]
float target_deg_y = 90.0;                    // Y軸の目標角度[deg]
float depth_gain = 0.375;                     // depthゲイン

float deg_g = 0.0;
float input, output;
float x_ctl, y_ctl, depth_ctl;
int flag = 0;
int depth_flag = 0;

String rec_data;
String rec_data_mini[3];
float conv_data_mini[3];
/*------------------------------------------------------------------------------------------------*/

void setup()
{
  Serial.begin(9600);  //加速度センサとESP32とのシリアル通信
  Serial1.begin(9600); //深度センサとのシリアル通信
  Wire.begin();
  motor_standby(servo_first_deg, servo_speed); //モータを初期位置に移動させる(other_setting.ino)
  check_sensor();                              //センサーの接続確認を行う(other_setting.ino)
  Serial.println("power on");
  delay(1000);
}

/*------------------------------------------------------------------------------------------------*/
void loop()
{
  // Serial.println("power down");

  float x_sum = 0, y_sum = 0, z_sum = 0, x_average, y_average, z_average;

  for (int i = 0; i < sampling; i++)
  {                                      //ノイズ軽減のため、[sampling]回データを取って平均する。
    x_sum = x_sum + ac.getCalculatedX(); //取得データのばらつきが大分マシになる
    y_sum = y_sum + ac.getCalculatedY(); //角度センサから値を読み取る(ライブラリ)
    z_sum = z_sum + ac.getCalculatedZ();
  }

  x_average = x_sum / sampling;
  y_average = y_sum / sampling;
  z_average = z_sum / sampling;
  float x_ang = change_deg(x_average, z_average) + 90; //角度をx,zの値から計算する(other_setting.ino)
  float y_ang = change_deg(y_average, z_average) + 90; //分度器で測ったところ、±1°くらいに収まった。
  float z_ang = z_average;

  float depth = ping.distance(); //深さの数値取得(ライブラリ)
  depth = 0;

  /*テスト用の信号発生関数---------------------------------------------------------*/
  // target_deg_x = deg_generater1(); sin波を再現する(other_setting.ino)
  // target_deg_x = deg_generater2(); ステップ入力を再現する(other_setting.ino)
  // x_ang = deg_generater1(); sin波を再現する(other_setting.ino)
  // x_ang = deg_generater2(); ステップ入力を再現する(other_setting.ino)
  /*------------------------------------------------------------------------------*/

  /*指定した範囲内に角度が収まっていなければPID制御を開始する。*/
  flag = 0;       //このフラグが1以上になっていなければ制御を行わない。（安定していればflagは0のまま）
  depth_flag = 0; //このフラグが1以上なら姿勢制御よりも深度制御を優先する。

  /* if (!(depth > target_depth_min && depth < target_depth_max))  //目標の深度にいるかを判断
     {
       flag++;                          // flagが1以上なら最後にモータを動かす。
       depth_flag++;                    // depth_flagが1以上なら深度制御を優先する。
       depth_ctl = P_ctl_depth(depth);  //深さ制御(angle_control.ino)
     }*/
  if (!(x_ang > target_deg_min && x_ang < target_deg_max)) //目標のロール角度にいるかを判断
  {
    flag++;
    x_ctl = PID_ctl_x(x_ang); //角度制御(angle_control.ino)
  }
  else
  {
    PID_reset_x(); //ゲイン計算結果のリセット(angle_control.ino)
  }
  if (!(y_ang > target_deg_min && y_ang < target_deg_max)) //目標のピッチ角度にいるかを判断
  {
    flag++;
    y_ctl = PID_ctl_y(y_ang); //角度制御(angle_control.ino)
  }
  else
  {
    PID_reset_y(); //ゲイン計算結果のリセット(angle_control.ino)
  }
  if (flag > 0)
  {
    /*（ロール角,ピッチ角,深度,深度制御の割り込み）*/
    fillet_right(x_ctl, y_ctl, depth_ctl, depth_flag); //モータへ出力する値の計算及び出力(motor_movement.ino)
    fillet_left(x_ctl, y_ctl, depth_ctl, depth_flag);
    fillet_up(x_ctl, y_ctl, depth_ctl, depth_flag);
    fillet_down(x_ctl, y_ctl, depth_ctl, depth_flag);
  }

  // Serial.println(String(45) + "," + String(135) + "," + String(90) + "," + String(x_ang) + "," + String(x_ctl + 90) + "," + String(y_ang) + "," + String(y_ctl + 90));
  // Serial.println("角度センサX:" + String(x_ang) + "," + "制御波形X:" + String(x_ctl + 90) + "," + "目標値X:" + String(target_deg_x) + "," + "角度センサY:" + String(y_ang) + "," + "制御波形Y:" + String(y_ctl + 90) + "," + "目標値Y:" + String(target_deg_y));
  //  Serial.println("roll(X):" + String(x_ang) + "," + "pitch(Y):" + String(y_ang) + "," + "roll_ctl(X):" + String(x_ctl) + "," + "pitch_ctl(Y):" + String(y_ctl) + "," + "target:" + String(90));
  //  Serial.println("roll(X):" + String(x_ang) + "," + "pitch(Y):" + String(y_ang) + "," + "target:" + String(90) + "," + "up:" + String(180) + "," + " down:" + String(-180));
  //  String graph = ("," + "up:" + String(180) + "," + " down:" + String(-180));
  //  Serial.println(str + graph);

  /*Bluetoothによるコマンドの処理*/

  if (Serial.available() > 0)
  {

    rec_data = Serial.readStringUntil(';');
    rec_data = String(rec_data);

    Serial.println("-Arduino-");
    Serial.println("(active)");
    Serial.println(rec_data);

    if (rec_data == "START|")
    {
      Serial.println("EditMode_standby...");

      send_commands(); //コマンドの処理(connect_commands.ino)

      Serial.println("EditMode_Exit...");
      delay(2000);
      Serial.println("Control_restart.");
    }

    delay(10);
  }
}