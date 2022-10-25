/*作成こばやし　2022/10/21更新　*/

void fillet_right(float x_ctl,float y_ctl,float water_depth)
{

    float input = x_ctl;
    vss_right.write(input, servo_speed, true);

}

void fillet_left(float x_ctl,float y_ctl,float water_depth)
{

    float input = x_ctl;
    vss_right.write(input, servo_speed, true);

}
