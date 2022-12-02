/*作成こばやし　2022/11/15更新　*/
void fillet_right(float x_ctl, float y_ctl, float depth_ctl)
{

    input = 90+(x_ctl + y_ctl) * pid_rate + depth_ctl * (1 - pid_rate);
    // Serial.println(String(input));
    if (input > deg_max) //モータの上限以上を出力しないように設定
        input = deg_max;
    if (input < deg_min)
        input = deg_min;

    // input = 90;

    vss_right.write(input, servo_speed, true);
}

void fillet_left(float x_ctl, float y_ctl, float depth_ctl)
{

    input = 90 + (x_ctl - y_ctl) * pid_rate + depth_ctl * (1 - pid_rate);

    if (input > deg_max)
        input = deg_max;
    if (input < deg_min)
        input = deg_min;
    vss_left.write(input, servo_speed, true);
}
void fillet_up(float x_ctl, float y_ctl, float depth_ctl)
{
    input = 90 + x_ctl; //上下のヒレは深度制御に影響しないため制御は行わない。

    if (input > deg_max)
        input = deg_max;
    if (input < deg_min)
        input = deg_min;
    vss_up.write(input, servo_speed, true);
}
void fillet_down(float x_ctl, float y_ctl, float depth_ctl)
{
    input = 90 + x_ctl;

    if (input > deg_max)
        input = deg_max;
    if (input < deg_min)
        input = deg_min;
    vss_down.write(input, servo_speed, true);
}
