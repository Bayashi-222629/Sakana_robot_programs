/*作成こばやし　2022/10/15更新　
ここでは、目標値を0として扱い、最後に実際の出力値まで矯正して出力します*/

static float output_deg = 0.0;
static float P_x, I_x, D_x; // PID値保存パラメータ
static float deg_x = 0.0, ctl_deg_x = 0.0;
static float dt_x, pre_dt_x, pre_P_x, pre_x_ang;

static float P_y, I_y, D_y;
static float deg_y = 0.0, ctl_deg_y = 0.0;
static float dt_y, pre_dt_y, pre_P_y;

/*X軸用のPID*/
float PID_ctl_x(float x_ang)
{

    dt_x = (millis() - pre_dt_x) / 1000; //疑似の微小時間0.001
    pre_P_x = P_x;
    pre_x_ang = x_ang;
    pre_dt_x = millis();

    P_x = target_deg_x - x_ang;        //偏差
    I_x += (P_x + pre_P_x) * dt_x / 2; //積分項（(上底-下底)*高さ/2）
    D_x = -(x_ang - pre_x_ang) / dt_x; //微分項（傾き/微小時間dx/dt）

    ctl_deg_x = (P_x * kp_x) + (I_x * ki_x) + (D_x * kd_x); // 0を中心にどのくらい変化させるか？
    output = ctl_deg_x + x_ang;

    if (output_deg > deg_max) //モータの動く角度を制限する。
    {
        // P_x = 0.0, I_x = 0.0, D_x = 0.0;
        ctl_deg_x = deg_max;
        output = deg_max;
    }
    else if (output_deg < deg_min)
    {
        // P_x = 0.0, I_x = 0.0, D_x = 0.0;
        ctl_deg_x = deg_min;
        output = deg_min;
    }
    // Serial.println("P:" + String(round(P_x)) + ",  " + "I:" + String(round(I_x)) + ",  " + "D:" + String(round(D_x)));
    Serial.println("偏差:" + String(P_x) + ",  " + "変化量:" + String(P_x - pre_P_x) + ",  " + "I:" + String(I_x) + ",  " + "D:" + String(D_x) + ",  " + "目標値:" + String(round(target_deg_x)));
    // Serial.println("ang:" + String(round(x_ang)) + ",  " + "ctl:" + String(round(output_deg)) + ",  " + "target:" + String(round(target_deg_x)));

    // return output_deg;

    return output;
}

/*Y軸用のPID*/
float PID_ctl_y(float y_ang)
{

    dt_y = (millis() - pre_dt_y) / 1000;
    pre_dt_y = millis();
    P_y = target_deg_y - y_ang;
    I_y = (P_y - pre_P_y) * dt_y / 2;
    D_y = (P_y - pre_P_y) / dt_y;
    pre_P_y = P_y;

    ctl_deg_y = (P_y * kp_y) + (I_y * ki_y) + (D_y * kd_y);
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