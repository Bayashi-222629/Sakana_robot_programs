/*作成こばやし　2022/11/11更新　*/

static float dt = 0.1; //微小時間

static float P_x, I_x, D_x; // PID値保存パラメータ
static float deg_x = 0.0, ctl_deg_x = 0.0;
static float dt_x, pre_dt_x, pre_P_x, pre_x_ang;

static float P_y, I_y, D_y;
static float deg_y = 0.0, ctl_deg_y = 0.0;
static float dt_y, pre_dt_y, pre_P_y;

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

/*Y軸用のPID*/
float PID_ctl_y(float y_ang)
{
    P_y = target_deg_y - y_ang;
    I_y += (P_y + pre_P_y) * dt / 2;
    D_y = (P_y - pre_P_y) / dt;
    pre_P_y = P_y;

    ctl_deg_y = (P_y * kp_y) + (I_y * ki_y) + (D_y * kd_y);

    // Serial.println("P:" + String(round(P_y)) + ",  " + "I:" + String(round(I_y)) + ",  " + "D:" + String(round(D_y)));
    // Serial.println("偏差:" + String(P_y) + ",  " + "センサ:" + String(y_ang) + ",  " + "I:" + String(I_y * ki_y) + ",  " + "D:" + String(D_y * kd_y) + ",  " + "目標値:" + String(round(target_deg_y)) + ", " + "出力:" + String(ctl_deg_y));

    return ctl_deg_y;
}

/*深さ用の比例制御*/
float P_ctl_depth(float depth)
{
    //目標値380mmだが45mmの誤差があるので425mm　最低345mm 最高505mm その間16㎝ 上下各8cmで最大30°動かす.
    // 30° ÷ 80mm = 0.375(R3のプログラムを流用)
    // target_depth ＝機体の重心の高さ位置
    depth_ctl = (target_depth - depth) * depth_gain;

    return depth_ctl;
}

void PID_reset_x()
{
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