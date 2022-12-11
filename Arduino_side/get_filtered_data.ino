/*作成こばやし　2022/12/5更新*/
float x_data = 0, y_data = 0, z_data = 0;
float filtered_x_data = 0, pre_filtered_x_data = 0;
float filtered_y_data = 0, pre_filtered_y_data = 0;
float filtered_z_data = 0, pre_filtered_z_data = 0;

float rd_R_B_data, rd_R_B = 2.5, pre_rd_R_B = 2.5;
float rd_L_V_data, rd_L_V = 2.5, pre_rd_L_V = 2.5;
float rd_R_V_data, rd_R_V = 2.5, pre_rd_R_V = 2.5;
float rd_L_B_data, rd_L_B = 2.5, pre_rd_L_B = 2.5;

float R_B_volt_error = 4.75; //レシーバのピンが場所によって若干電圧が違うため、無操作時に2.5V統一されるように微調整する。
float L_V_volt_error = 4.76; //全て目視による実験値です。
float R_V_volt_error = 5.47;
float L_B_volt_error = 4.81;
float rd_range_max = 4.6;
float rd_range_min = 0.25;
float voltage = 5;

float deg;

float get_data_z()
{
    z_data = ac.getCalculatedZ();                                       //角度センサから値を読み取る(ライブラリ)
    filtered_z_data = z_data * angle_prob + pre_filtered_z_data * (1 - angle_prob); //ローパスフィルタの再現
    pre_filtered_z_data = filtered_z_data;

    return (filtered_z_data);
}

float get_data_x()
{
    x_data = ac.getCalculatedX();
    filtered_x_data = x_data * angle_prob + pre_filtered_x_data * (1 - angle_prob);
    pre_filtered_x_data = filtered_x_data;

    deg = atan2(filtered_x_data, filtered_z_data) * 180.0 / PI + 90; //データをdegree表記に変換

    // Serial.println(String(x_data) + "," + String(filtered_x_data) + "," + String(deg));

    return (deg);
}

float get_data_y()
{
    y_data = ac.getCalculatedY();
    filtered_y_data = y_data * angle_prob + pre_filtered_y_data * (1 - angle_prob);
    pre_filtered_y_data = filtered_y_data;

    deg = atan2(filtered_y_data, filtered_z_data) * 180.0 / PI + 90;

    return (deg);
}

float get_data_R_B()
{
    rd_R_B_data = pulseIn(RD_R_B, HIGH, 0.1) * voltage / 1024 - R_B_volt_error; // pulseIN関数でレシーバのPWM信号を受信する。*volt/1024で電圧換算する。 errorは固体差による誤差修正。
    rd_R_B = rd_R_B_data * radio_prob + pre_rd_R_B * (1 - radio_prob);
    pre_rd_R_B = rd_R_B;

    deg = ((deg_max - deg_min) / (rd_range_max - rd_range_min) * (rd_R_B - rd_range_max)) + deg_max; //(180[deg],4.65[v])~(0[deg],0.35[deg])の二点を通る比例式

    return (deg);
}

float get_data_L_V()
{
    rd_L_V_data = pulseIn(RD_L_V, HIGH, 0.1) * voltage / 1024 - L_V_volt_error;
    rd_L_V = rd_L_V_data * radio_prob + pre_rd_L_V * (1 - radio_prob);
    pre_rd_L_V = rd_L_V;

    deg = ((deg_max - deg_min) / (rd_range_max - rd_range_min) * (rd_L_V - rd_range_max)) + deg_max;

    return (deg);
}

float get_data_R_V()
{
    rd_R_V_data = pulseIn(RD_R_V, HIGH, 0.1) * voltage / 1024 - R_V_volt_error;
    rd_R_V = rd_R_V_data * radio_prob + pre_rd_R_V * (1 - radio_prob);
    pre_rd_R_V = rd_R_V;

    deg = ((deg_max - deg_min) / (rd_range_max - rd_range_min) * (rd_R_V - rd_range_max)) + deg_max;

    return (deg);
}

float get_data_L_B()
{
    rd_L_B_data = pulseIn(RD_L_B, HIGH, 0.1) * voltage / 1024 - L_B_volt_error;
    rd_L_B = rd_L_B_data * radio_prob + pre_rd_L_B * (1 - radio_prob);
    pre_rd_L_B = rd_L_B;

    deg = ((deg_max - deg_min) / (rd_range_max - rd_range_min) * (rd_L_B - rd_range_max)) + deg_max;

    return (deg);
}