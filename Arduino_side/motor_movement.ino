/*作成こばやし　2022/11/11更新　*/

void fillet_right(float x_ctl, float y_ctl, float depth_ctl, int depth_flag)
{

    if (depth_flag != 0) // depth_flagが1以上なら深度制御を優先する。
        input = 90 + depth_ctl;
    else
        input = 90 + x_ctl + y_ctl; //大丈夫なら姿勢制御をする。

    if (input > deg_max) //モータの上限以上を出力しないように設定
        input = deg_max;
    if (input < deg_min)
        input = deg_min;
    vss_right.write(input, servo_speed, true);
}

void fillet_left(float x_ctl, float y_ctl, float depth_ctl, int depth_flag)
{
    if (depth_flag > 0)
        input = 90 + depth_ctl;
    else
        input = 90 + x_ctl - y_ctl;

    if (input > deg_max)
        input = deg_max;
    if (input < deg_min)
        input = deg_min;
    vss_left.write(input, servo_speed, true);
}
void fillet_up(float x_ctl, float y_ctl, float depth_ctl, int depth_flag)
{
    input = 90 + x_ctl; //上下のヒレは深度制御に影響しないため、優先は行わない。

    if (input > deg_max)
        input = deg_max;
    if (input < deg_min)
        input = deg_min;
    vss_up.write(input, servo_speed, true);
}
void fillet_down(float x_ctl, float y_ctl, float depth_ctl, int depth_flag)
{
    input = 90 + x_ctl;

    if (input > deg_max)
        input = deg_max;
    if (input < deg_min)
        input = deg_min;
    vss_down.write(input, servo_speed, true);
}
