/*作成こばやし　2022/12/2更新　
センサで取得した情報を元にヒレの角度を算出する関数が置いてあります。
姿勢制御はPIDを用いて、深さ制御は比例制御です。
*/
static float dt = 0.1; //微小時間

float P_x, I_x, D_x; // PID値保存パラメータ
float deg_x = 0.0, ctl_deg_x = 0.0;
float dt_x, pre_dt_x, pre_P_x, pre_x_ang;

float P_y, I_y, D_y;
float deg_y = 0.0, ctl_deg_y = 0.0;
float dt_y, pre_dt_y, pre_P_y;

/*X軸用のPID*/
/* x_ang：センサで取得したロール角度、target_deg_x：目標値、ctl_deg_x：PID出力*/
float PID_ctl_x(float x_ang)
{
  P_x = target_deg_x - x_ang; //偏差  U(s)=(Kp+Kis+Kds)E(s)
  I_x += P_x * dt;            //積分項
  D_x = (P_x - pre_P_x) / dt; //微分項（傾き/微小時間 dx/dt）
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
  I_y += P_y * dt;
  D_y = (P_y - pre_P_y) / dt;
  pre_P_y = P_y;
  ctl_deg_y = (P_y * kp_y) + (I_y * ki_y) + (D_y * kd_y);

  return ctl_deg_y;
}

/*深さ用の比例制御*/
float P_ctl_depth(float depth)
{
  //目標値380mmだが45mmの誤差があるので425mm　最低345mm 最高505mm その間16㎝ 上下各8cmで最大30°動かす.
  // 30° ÷ 80mm = 0.375(R3のプログラムを流用)

  /*プールの高さを1000[mm]と仮定すると…
  センサ位置～重心の距離：180[mm]
  重心～ヒレ先端の距離：310.5[mm]
  床面とヒレが衝突する重心高さ：310.5[mm]
  水面からヒレが飛び出る重心高さ：689.5[mm]
  ヒレの角度をmax45°動かす時の深さ：340[mm]～420[mm]（目標値±40mm）
  target_depth ＝機体の重心の目標高さ,depth = 現在の高さ
  */

  if (depth > target_depth)
    depth_ctl = (deg_max - 0) / (target_depth_max - target_depth) * (depth - target_depth) + 0;

  if (depth <= target_depth)
    depth_ctl = (0 - deg_min) / (target_depth - target_depth_min) * (depth - target_depth_min) + deg_min;
    
  
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