int check_sensor() //センサの接続チェック
{
    while (!ac.begin())
    {
        Serial.println("角度センサの応答がありません！配線位置を確認してみて！");
        delay(2000);
    }
}

float change_deg(float x_y_deg, float z_deg) //ｘｙの値とｚの値を元に角度を計算する。
{
    float deg = round(atan2(x_y_deg, z_deg) * 180.0 / PI);

    return deg;
}

float motor_standby(float servo_first_pos, float servo_speed) //各モータの初期位置の設定
{
    vss_right.attach(SERVO_NUM_RIGHT);
    vss_left.attach(SERVO_NUM_LEFT);
    vss_up.attach(SERVO_NUM_UP);
    vss_down.attach(SERVO_NUM_DOWN);
    vss_right.write(servo_first_pos, servo_speed, true);
    vss_left.write(servo_first_pos, servo_speed, true);
    vss_up.write(servo_first_pos, servo_speed, true);
    vss_down.write(servo_first_pos, servo_speed, true);
}