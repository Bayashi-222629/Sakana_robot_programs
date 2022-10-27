/*作成こばやし　2022/10/15更新　
ここでは、目標値を0として扱い、最後に実際の出力値まで矯正して出力します*/

static float output_deg = 0.0;
static float P_x, I_x, D_x; // PID値保存パラメータ
static float deg_x = 0.0, ctl_deg_x = 0.0;
static float dt_x, pre_dt_x, pre_P_x;

static float P_y, I_y, D_y;
static float deg_y = 0.0, ctl_deg_y = 0.0;
static float dt_y, pre_dt_y, pre_P_y;

/*X軸用のPID*/
float PID_ctl_x(float x_ang)
{

    dt_x = (micros() - pre_dt_x) / 1000000; //疑似の微小時間
    pre_dt_x = micros();
    P_x = target_deg_x - x_ang;       //偏差
    I_x += P_x * dt_x;                //積分項（偏差*微小時間）
    D_x = (P_x - pre_P_x) * 1 / dt_x; //微分項（傾き/微小時間）
    pre_P_x = P_x;

    ctl_deg_x = (P_x * kp_x) + (I_x * ki_x) + (D_x * kd_x); // 0を中心にどのくらい変化させるか？
    output_deg = ctl_deg_x + target_deg_x;

    if (output_deg > deg_max) //モータの動く角度を制限する。
    {
        P_x = 0.0, I_x = 0.0, D_x = 0.0;
        ctl_deg_x = deg_max;
        output_deg = deg_max;
    }
    else if (output_deg < deg_min)
    {
        P_x = 0.0, I_x = 0.0, D_x = 0.0;
        ctl_deg_x = deg_min;
        output_deg = deg_min;
    }
    //Serial.println("P:" + String(round(P_x * kp_x)) + ",  " + "I:" + String(round(I_x * ki_x)) + ",  " + "D:" + String(round(D_x * kd_x)) + ",  " + "ang:" + String(round(x_ang)) + ",  " + "ctl:" + String(round(ctl_deg_x)));

    return output_deg;
}

/*Y軸用のPID*/
float PID_ctl_y(float y_ang)
{

    dt_y = (micros() - pre_dt_y) / 1000000;
    pre_dt_y = micros();
    P_y = target_deg_y - y_ang;
    I_y += P_y * dt_y;
    D_y = (P_y - pre_P_y) * 1 / dt_y;
    pre_P_y = P_y;

    ctl_deg_y = (P_y * kp_y) + (I_y * ki_y) + (D_y * kd_y); // 0を中心にどのくらい変化させるか？
    output_deg = ctl_deg_y + target_deg_y;

    if (output_deg > deg_max)
    {
        P_y = 0.0, I_y = 0.0, D_y = 0.0;
        ctl_deg_y = deg_max;
        output_deg = deg_max;
    }
    else if (output_deg < deg_min)
    {
        P_y = 0.0, I_y = 0.0, D_y = 0.0;
        ctl_deg_y = deg_min;
        output_deg = deg_min;
    }

    return output_deg;
}

void PID_reset_x()
{
    output_deg = 0.0;
    P_x, I_x, D_x;
    deg_x = 0.0, ctl_deg_x = 0.0;
    dt_x, pre_dt_x, pre_P_x;
}
void PID_reset_y()
{
    P_y, I_y, D_y;
    deg_y = 0.0, ctl_deg_y = 0.0;
    dt_y, pre_dt_y, pre_P_y;
}