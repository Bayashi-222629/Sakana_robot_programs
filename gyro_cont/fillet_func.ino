/*作成こばやし　2022/10/21更新　*/

void fillet_right(float x_ctl, float y_ctl, float water_depth)
{
    input = 90 + x_ctl + y_ctl; //処理
    // float input = 90;

    if (input > deg_max)
        input = deg_max;
    if (input < deg_min)
        input = deg_min;
    vss_right.write(input, servo_speed, true);
}

void fillet_left(float x_ctl, float y_ctl, float water_depth)
{
    input = 90 + x_ctl - y_ctl; //処理
    // float input = 90;

    if (input > deg_max)
        input = deg_max;
    if (input < deg_min)
        input = deg_min;
    vss_left.write(input, servo_speed, true);
}
void fillet_up(float x_ctl, float y_ctl, float water_depth)
{
    input = 90 + x_ctl; //処理
    // float input = 90;

    if (input > deg_max)
        input = deg_max;
    if (input < deg_min)
        input = deg_min;
    vss_up.write(input, servo_speed, true);
}
void fillet_down(float x_ctl, float y_ctl, float water_depth)
{
    input = 90 + x_ctl; //処理
    // float input = 90;

    if (input > deg_max)
        input = deg_max;
    if (input < deg_min)
        input = deg_min;
    vss_down.write(input, servo_speed, true);
}
