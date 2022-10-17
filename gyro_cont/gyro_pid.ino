/*作成こばやし　2022/10/15更新　*/
float PID_ctl_x(float x_ang)
{
    float output_deg=0.0;

    dt_x = (micros() - pre_dt_x) / 1000000; //疑似の微小時間
    pre_dt_x = micros();
    P_x = offset_deg - x_ang; //偏差
    I_x += P_x * dt_x;            //積分項（偏差*微小時間）
    D_x = (P_x - pre_P_x) / dt_x;   //微分項（傾き/微小時間）

    pre_P_x = P_x;

    ctl_deg_x += (P_x * kp_x) + (I_x * ki_x) + (D_x * kd_x); // 0を中心にどのくらい変化させるか？
    output_deg = ctl_deg_x + target_deg_x;         //ターゲット角度まで増加させただけ
    
    if (output_deg > max_deg)                  //モータの動く角度を制限する。
    {
        ctl_deg_x = max_deg;
        output_deg = max_deg;
    }
    else if (output_deg < min_deg)
    {
        ctl_deg_x = min_deg;
        output_deg = min_deg;
    }

    return output_deg;
}

float PID_ctl_y(float y_ang)
{
    float output_deg=0.0;

    dt_y = (micros() - pre_dt_y) / 1000000; //疑似の微小時間
    pre_dt_y = micros();
    P_y = offset_deg - y_ang; //偏差
    I_y += P_y * dt_y;            //積分項（偏差*微小時間）
    D_y = (P_y - pre_P_y) / dt_y;   //微分項（傾き/微小時間）

    pre_P_y = P_y;

    ctl_deg_y += (P_y * kp_y) + (I_y * ki_y) + (D_y * kd_y); // 0を中心にどのくらい変化させるか？
    output_deg = ctl_deg_y + target_deg_y;         //ターゲット角度まで増加させただけ
    
    if (output_deg > max_deg)                  //モータの動く角度を制限する。
    {
        ctl_deg_y = max_deg;
        output_deg = max_deg;
    }
    else if (output_deg < min_deg)
    {
        ctl_deg_y = min_deg;
        output_deg = min_deg;
    }

    return output_deg;
}