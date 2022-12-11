/*作成こばやし　2022/12/6更新　[0222yuga@gmail.com]

  コメントの()内の文言は、その関数の内容がどのファイルに入っているかを示しています。
  X軸:ロール、Y軸:ピッチとします。
  (例)//～(other_setting.ino)＝　other_setting.inoに記述

  〇必要アイテム
  ・ArduinoMega 　・サーボモータ×4　　　　　・FutabaR124H（プロポ受信機）
  ・ESP32wroom32　・MMA8452Q（加速度センサ）・PingSonar（深度センサ）
  ・Bluetoothでシリアル通信できるソフト（androidスマホなら「Serial Bluetooth terminal」というアプリなど）

  〇ファイル記述内容
  ・「Arduino_side.ino」…メイン処理、setup()loop()が入っています。
  ・「angle_control.ino」…姿勢のPID制御、深度のP制御をします。
  ・「get_filterd.ino」…各センサからの値を取得し、ローパスフィルタに通します。
  ・「motor_movement.ino」…動作可能な範囲内でモータを動かします。
  ・「connect_commands.ino」…Bluetoothで送られてきたコマンドを処理します。
  ・「other_setting.ino」…モータのピン割り当て、センサの接続確認など色々やります。
  ・「ESP32wroom_side」…Bluetoothのデータ受信やUART通信等のESP32で行う動作が入っています。
  ・「Serial_read_program.py」…PCへSerialprintした内容をリアルタイムで読み、CSVを生成して出力します。(python)

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

#define SERVO_NUM_RIGHT 10 // サーボモータのArduino接続ピン*変更可
#define SERVO_NUM_LEFT 11  // 右：10　左：11　上：12　下：13
#define SERVO_NUM_UP 12
#define SERVO_NUM_DOWN 13

#define RD_R_B A0 // ラジコンのレシーバのArduino接続ピン*変更可
#define RD_L_V A1 // 「レシーバ:Arduino」＝「1:A0,2:A1,3:A2,4:A3」
#define RD_R_V A2
#define RD_L_B A3

static const uint8_t arduinoTxPin = 18; // Arduinoのソナー用txピン（緑）*ライブラリの都合上変更不可
static const uint8_t arduinoRxPin = 19; // Arduinoのソナー用rxピン（白）*ライブラリの都合上変更不可
static Ping1D ping{Serial1};
static const int up = 180, down = -180; // シリアルプロッタの上限と下限の設定

float deg_max = 135.0;             // モータ角度上限[deg]
float deg_min = 45.0;              // モータ角度下限[deg]
const float target_deg_max = 91.0; // 機体の安定判定の角度上限（この角度内に機体が収まれば姿勢制御をしない）
const float target_deg_min = 89.0; // 機体の安定判定の角度下限
const int servo_first_deg = 90;    // サーボモータの初期角度[deg]
const int servo_speed = 0;         // サーボモータの回転速度(0で最高速度)
const float angle_prob = 0.3;      // 角度データの信頼度[%]（0～1の値 ローパスフィルタで使用）
const float radio_prob = 0.3;      // レシーバデータの信頼度[%]（0～1）
float pid_rate = 1.0;              // PIDとdepthの信号の優先割合（0～1）

float sensor_position_error = 180.0;                          // センサ読み取り位置から重心までの距離[mm]
float target_depth = 380;                                     // 目標深度[mm]
float target_depth_min = 340;                                 // 深度下限[mm]
float target_depth_max = target_depth * 2 - target_depth_min; // 深度上限[mm]
// const float ab_depth_min = 310.5;        //絶対深度下限[mm](固定値。ヒレが床に衝突する深度)

float kp_x = 1.5, ki_x = 0.0, kd_x = 0.5; // xPIDゲイン
float kp_y = 1.5, ki_y = 0.0, kd_y = 0.5; // yPIDゲイン
float depth_gain = 0.375;                 // depth比例ゲイン
float target_deg_x = 90.0;                // X軸の目標角度[deg]
float target_deg_y = 90.0;                // Y軸の目標角度[deg]

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
  Serial.begin(9600);  // 加速度センサとESP32とのシリアル通信
  Serial1.begin(9600); // 深度センサとのシリアル通信
  Wire.begin();
  motor_standby(servo_first_deg, servo_speed); // モータを初期位置に移動させる(other_setting.ino)
  check_sensor();                              // センサーの接続確認を行う(other_setting.ino)

  Serial.println("[Sakana program] power on");
  delay(1000);
}

/*------------------------------------------------------------------------------------------------*/
void loop()
{
  float rd_R_B = get_data_R_B(); // レシーバから数値を受信し、フィルタに通してdegreeに変換(get_filtered_data.ino)
  float rd_L_V = get_data_L_V(); // プロポのレバー配置は以下の通り。
  float rd_R_V = get_data_R_V(); // R:右, L:左, B:横, V:縦
  float rd_L_B = get_data_L_B(); // L_B＝ロール、L_V＝ピッチ、R_V＝メインモータ回転速度、R_B＝割り当てなし

  float z_ang = get_data_z(); // センサから角度を読み取り、フィルタに通してdegreeに変換(get_filtered_data.ino)
  float x_ang = get_data_x(); // 角度変換でZを使用する関係でZを先に計算しておく。
  float y_ang = get_data_y();

  if (ping.update()) // これを呼び出さないと深度数値を取得してくれない。公式サイトにはセンサ使用を助ける関数と書いてある(ライブラリ)
  {
    if (ping.confidence() > 0)
    {
      float depth = ping.distance(); // 深さの数値取得(ライブラリ)
    }
  }

  // テスト用信号の発生関数---------------------------------------------------------
  //  target_deg_x = deg_generater1(); //sin波を再現する(other_setting.ino)
  //  target_deg_x = deg_generater2(); //ステップ入力を再現する(other_setting.ino)
  //  x_ang = deg_generater1(); // sin波を再現する(other_setting.ino)
  //  x_ang = deg_generater2(); //ステップ入力を再現する(other_setting.ino)
  // depth = deg_generater3(); // 深さ用sin波を再現する(other_setting.ino)
  //------------------------------------------------------------------------------

  // 指定した範囲内に角度が収まっていなければPID制御を開始する。

  flag = 0;                                                    // このフラグが1以上になっていなければ制御を行わない。（安定していればflagは0のまま）
  if (!(depth > target_depth_min && depth < target_depth_max)) // 目標の深度にいるかを判断
  {
    flag++;                         // flagが1以上なら最後にモータを動かす。
    depth_ctl = P_ctl_depth(depth); // 深さ制御(angle_control.ino)
  }

  if (!(x_ang > target_deg_min && x_ang < target_deg_max)) // 目標のロール角度にいるかを判断
  {
    flag++;
    x_ctl = PID_ctl_x(x_ang); // 角度制御(angle_control.ino)
  }
  else
  {
    PID_reset_x(); // ゲイン計算結果のリセット(angle_control.ino)
  }

  if (!(y_ang > target_deg_min && y_ang < target_deg_max)) // 目標のピッチ角度にいるかを判断
  {
    flag++;
    y_ctl = PID_ctl_y(y_ang); // 角度制御(angle_control.ino)
  }
  else
  {
    PID_reset_y(); // ゲイン計算結果のリセット(angle_control.ino)
  }

  Serial.println(String(depth) + "," + String(depth_ctl));

  if (flag > 0)
  {
    // （ロール角,ピッチ角,深度,深度制御の割り込み
    fillet_right(x_ctl, y_ctl, depth_ctl, rd_L_B, rd_L_V); // モータへ出力する値の計算及び出力(motor_movement.ino)
    fillet_left(x_ctl, y_ctl, depth_ctl, rd_L_B, rd_L_V);
    fillet_up(x_ctl, y_ctl, depth_ctl, rd_L_B);
    fillet_down(x_ctl, y_ctl, depth_ctl, rd_L_B);
  }

  // Serial.println(String(45) + "," + String(135) + "," + String(90) + "," + String(x_ang) + "," + String(x_ctl + 90) + "," + String(y_ang) + "," + String(y_ctl));
  // Serial.println("角度センサX:" + String(x_ang) + "," + "制御波形X:" + String(x_ctl) + "," + "角度センサY:" + String(y_ang) + "," + "制御波形Y:" + String(y_ctl) + "," + "上限:" + String(180) + "," + "下限:" + String(0));
  // Serial.println("roll(X):" + String(x_ang) + "," + "pitch(Y):" + String(y_ang) + "," + "target:" + String(90) + "," + "up:" + String(180) + "," + " down:" + String(-180));
  // String graph = ("," + "up:" + String(180) + "," + " down:" + String(-180));
  // Serial.println(str + graph);

  // Bluetoothによるコマンドの処理

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

      send_commands(); // コマンドの処理(connect_commands.ino)

      Serial.println("EditMode_Exit...");
      delay(2000);
      Serial.println("Control_restart.");
    }

    delay(5);
  }
}
