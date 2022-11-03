static float kp_x = 1.25, ki_x = 0.0, kd_x = 0.0; // PIDゲイン
float target_deg_x = 90.0;                           // 目標深度
static float dt = 0.1;                               //微小時間

static float P_x, I_x, D_x; // PID値保存パラメータ
static float deg_x = 0.0, ctl_deg_x = 0.0;
static float dt_x, pre_dt_x, pre_P_x, pre_x_ang;
static float P_y, I_y, D_y;

/*X軸用のPID*/
/* x_ang：センサで取得したロール角度、target_deg_x：目標値、ctl_deg_x：PID出力*/
float PID_ctl_x(float x_ang)
{

    P_x = target_deg_x - x_ang;      //偏差  U(s)=(Kp+Kis+Kds)E(s)
    I_x += (P_x + pre_P_x) * dt / 2; //積分項（(上底-下底)*高さ/2）
    D_x = (P_x - pre_P_x) / dt;      //微分項（傾き/微小時間dx/dt）
    pre_P_x = P_x;

    ctl_deg_x = (P_x * kp_x) + (I_x * ki_x) + (D_x * kd_x);

    // Serial.println("P:" + String(round(P_x)) + ",  " + "I:" + String(round(I_x)) + ",  " + "D:" + String(round(D_x)));
    // Serial.println("センサ:" + String(x_ang) + ",  " + "偏差:" + String(P_x) + ",  " + "I:" + String(I_x * ki_x) + ",  " + "D:" + String(D_x * kd_x) + ",  " + "目標値:" + String(round(target_deg_x)) + ", " + "出力:" + String(ctl_deg_x));
    // Serial.println("ang:" + String(round(x_ang)) + ",  " + "ctl:" + String(round(ctl_deg_x)) + ",  " + "target:" + String(round(target_deg_x)));

    return ctl_deg_x;
}

void PID_reset_x()
{
    P_x, I_x, D_x;
    deg_x = 0.0, ctl_deg_x = 0.0;
    dt_x, pre_dt_x, pre_P_x;
}
