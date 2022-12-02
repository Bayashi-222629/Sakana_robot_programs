/*作成こばやし　2022/11/15更新　[0222yuga@gmail.com]

  コメントの()内の文言は、その関数の内容がどのファイルに入っているか示しています。
  X軸:ロール、Y軸:ピッチとします。
  例))//～(other_setting.ino)＝　other_setting.inoに記述

  〇必要アイテム
  ・ArduinoMega 　・サーボモータ×4　　　　　・
  ・ESP32wroom32　・MMA8452Q（加速度センサ）・PingSonar（深度センサ）

*/

#include <Wire.h>              //シリアル通信用ライブラリ
#include "SparkFun_MMA8452Q.h" //加速度センサ読み取りライブラリ
MMA8452Q ac;
#include "VarSpeedServo.h" //サーボモータ駆動用ライブラリ
VarSpeedServo vss_right;
VarSpeedServo vss_left;
VarSpeedServo vss_up;
VarSpeedServo vss_down;
#include "ping1d.h" //深度センサ読み取り用ライブラリ
static Ping1D ping{Serial};

#define SERVO_NUM_RIGHT 10 //サーボモータのArduino接続ピン*変更可
#define SERVO_NUM_LEFT 11  //右：10　左：11　上：12　下：13
#define SERVO_NUM_UP 12
#define SERVO_NUM_DOWN 13
static const uint8_t arduinoRxPin = 18; // Arduinoのソナー用rxピン（白）*ライブラリの都合上変更不可
static const uint8_t arduinoTxPin = 19; // Arduinoのソナー用txピン（緑）*ライブラリの都合上変更不可
static const int up = 180, down = -180; //シリアルプロッタの上限と下限の設定

float deg_max = 135.0;             //モータ角度上限[deg]
float deg_min = 45.0;              //モータ角度下限[deg]
const int servo_first_deg = 90;    //サーボモータの初期角度[deg]
const int servo_speed = 0;         //サーボモータの回転速度(0で最高速度)
const float prob = 0.2;            //角度データの信頼度[%]（0～1の値 ローパスフィルタで使用）
const float target_deg_max = 91.0; //機体の安定判定の角度上限（この角度内に機体が収まれば姿勢制御をしない）
const float target_deg_min = 89.0; //機体の安定判定の角度下限
float pid_rate = 1.0;              // PIDとdepthの信号の優先割合（0～1）

//目標値380mmだが45mmの誤差があるので425mm　最低345mm 最高505mm その間160mm 上下各80mm
float sensor_position_error = 45.0;                                 //深さセンサ配置位置から機体重心までの距離[mm]
float target_depth = 380;                                           //目標深度[mm]
float target_depth_max = target_depth + sensor_position_error + 80; //深度上限[mm]
float target_depth_min = target_depth - 80;                         //深度下限[mm]

float kp_x = 1.5, ki_x = 0.0, kd_x = 0.5; // xPIDゲイン
float kp_y = 1.5, ki_y = 0.0, kd_y = 0.5; // yPIDゲイン
float target_deg_x = 90.0;                // X軸の目標角度[deg]
float target_deg_y = 90.0;                // Y軸の目標角度[deg]
float depth_gain = 0.375;                 // depthゲイン

float deg_g = 0.0;
float input, output;
float x_ctl, y_ctl, depth_ctl;
int flag = 0;

String rec_data;
String rec_data_mini[3];
float conv_data_mini[3];
/*------------------------------------------------------------------------------------------------*/

void setup()
{
  Serial.begin(9600); //加速度センサとESP32とのシリアル通信
  // Serial1.begin(9600); //深度センサとのシリアル通信
  Wire.begin();
  motor_standby(servo_first_deg, servo_speed); //モータを初期位置に移動させる(other_setting.ino)
  check_sensor();                              //センサーの接続確認を行う(other_setting.ino)

  // Serial.print("Distance: ");
  // Serial.print((double)ping.distance());
  // Serial.print("\tConfidence: ");
  // Serial.println(ping.confidence());

  Serial.println("power on");
  delay(1000);
}

/*------------------------------------------------------------------------------------------------*/
void loop()
{
  float z_ang = get_data_z(); //センサから角度を読み取り、フィルタに通してdegreeに変換(get_filtered_data.ino)
  float x_ang = get_data_x(); //角度変換でZを使用する関係でZを先に計算しておく。
  float y_ang = get_data_y();
  float depth = 0;      // ping.distance();        //深さの数値取得(ライブラリ)
  float depth_conf = 0; // ping.confidence(); //深さの信頼度取得(ライブラリ)

  //テスト用の信号発生関数---------------------------------------------------------
  // target_deg_x = deg_generater1(); //sin波を再現する(other_setting.ino)
  // target_deg_x = deg_generater2(); //ステップ入力を再現する(other_setting.ino)
  // x_ang = deg_generater1(); // sin波を再現する(other_setting.ino)
  // x_ang = deg_generater2(); //ステップ入力を再現する(other_setting.ino)
  // depth = deg_generater3(); // 深さ用sin波を再現する(other_setting.ino)
  //------------------------------------------------------------------------------
  // Serial.println(depth);

  //指定した範囲内に角度が収まっていなければPID制御を開始する。
  flag = 0; //このフラグが1以上になっていなければ制御を行わない。（安定していればflagは0のまま）

  // if (!(depth > target_depth_min && depth < target_depth_max)) //目標の深度にいるかを判断
  // {
  //   flag++;                         // flagが1以上なら最後にモータを動かす。
  //   depth_ctl = P_ctl_depth(depth); //深さ制御(angle_control.ino)
  // }

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
    //（ロール角,ピッチ角,深度,深度制御の割り込み
    fillet_right(x_ctl, y_ctl, depth_ctl); //モータへ出力する値の計算及び出力(motor_movement.ino)
    fillet_left(x_ctl, y_ctl, depth_ctl);
    fillet_up(x_ctl, y_ctl, depth_ctl);
    fillet_down(x_ctl, y_ctl, depth_ctl);
  }

  // Serial.println(String(45) + "," + String(135) + "," + String(90) + "," + String(x_ang) + "," + String(x_ctl + 90) + "," + String(y_ang) + "," + String(y_ctl));
   Serial.println("角度センサX:" + String(x_ang) + "," + "制御波形X:" + String(x_ctl) + "," + "角度センサY:" + String(y_ang) + "," + "制御波形Y:" + String(y_ctl) + "," + "上限:" + String(180) + "," + "下限:" + String(0));
  // Serial.println("roll(X):" + String(x_ang) + "," + "pitch(Y):" + String(y_ang) + "," + "target:" + String(90) + "," + "up:" + String(180) + "," + " down:" + String(-180));
  // String graph = ("," + "up:" + String(180) + "," + " down:" + String(-180));
  // Serial.println(str + graph);

  // Bluetoothによるコマンドの処理
  /*
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

      delay(5);
    }*/
}
