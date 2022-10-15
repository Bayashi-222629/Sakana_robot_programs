/*作成こばやし　2022/10/15更新　*/

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

const int servo_speed = 80; //サーボモータの回転速度
const int servo_first_pos = 0; //サーボモータの初期位置

const int sampling = 10; //角度データのサンプリング回数
const float target = 90.0; //目標角度
const float target_max = 92.0; //目標角度上限
const float target_min = 88.0; //目標角度下限

const float kp = 0.1; //PID各種ゲイン
const float ki = 0.5;
const float kd = 0.5;

float buff;

/*関数いろいろ------------------------------------------------------------------------------------*/
int check_sensor(){  //センサの接続チェック
  while (!ac.begin()) {
    Serial.println("角度センサの応答がありません！配線位置を確認してみて！");
    delay(2000);
  }
}

float change_deg(float x_y_deg, float z_deg){ //ｘｙの値とｚの値を元に角度を計算する。
  float  deg = round(atan2(x_y_deg , z_deg) * 180.0 / PI);
  
  return deg;
}

/*------------------------------------------------------------------------------------------------*/

void setup() {
  Serial.begin(9600);
  Wire.begin();

  vss_right.attach(SERVO_NUM_RIGHT);
  vss_left.attach(SERVO_NUM_LEFT);
  vss_up.attach(SERVO_NUM_UP);
  vss_down.attach(SERVO_NUM_DOWN);
  check_sensor();
  //vss.write(servo_first_pos, servo_speed, true); //各モータの初期位置の設定

  delay(2000);
}


/*------------------------------------------------------------------------------------------------*/
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

    float x_ang = change_deg(x_average, z_average);  //角度をx,zの値から計算するよ！
    float y_ang = change_deg(y_average, z_average);  //分度器で測ったところ、±1°くらいに収まった。
    float z_ang = ac.getCalculatedZ();


    String str = "||||xの角度:" + String(x_ang) + "," + "yの角度:" + String(y_ang); //シリアルモニタ表示用のメッセージ
    String graph = ("||||設定用：" + String(up) + "," + String(down));
    Serial.println(str + graph);

    buff = target - x_ang;
    vss_right.write(buff,servo_speed,true);
    

    //delay(5);

  }
}
