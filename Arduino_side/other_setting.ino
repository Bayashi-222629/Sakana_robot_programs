/*作成こばやし　2022/11/11更新　*/
/*センサの接続チェック*/
int check_sensor()
{
    while (!ac.begin())
    {
        Serial.println("角度センサの応答がありません。Arduinoの配線位置を確認してみてください。");
        delay(2000);
    }
    /* while (!ping.initialize())
     {
         Serial.print("ソナーの応答がありません!Arduinoの配線位置を確認してみてください。");
         Serial.print(arduinoTxPin + "へ緑の線を接続してください。");
         Serial.print(arduinoRxPin + "へ白の線を接続してください。");
         delay(2000);
     }*/
}
/*ｘｙの値とｚの値を元に角度を計算する。*/
float change_deg(float x_y_deg, float z_deg)
{
    float deg = atan2(x_y_deg, z_deg) * 180.0 / PI;

    return deg;
}

/*各モータの初期位置の設定*/
float motor_standby(float servo_first_pos, float servo_speed)
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

/*目標角度を自動で動かすやつ*/
float deg_generater1()
{

    deg_g += 0.05;
    output = 90 + 45 * sin(0.5 * deg_g);
    // Serial.println(output);

    return output;
}
float deg_generater2()
{
    deg_g += 1.5;

    if (deg_g < 100)
    {
        output = 45;
    }
    else if (100 <= deg_g && deg_g < 200)
    {
        output = 90;
    }
    else if (200 <= deg_g && deg_g < 300)
    {
        output = 135;
    } /*
     else if (300 <= deg_g && deg_g < 400)
     {
         output = 90;
     }*/
    else
    {
        deg_g = 0;
    }
    return output;
}