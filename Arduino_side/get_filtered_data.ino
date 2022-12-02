/*作成こばやし　2022/11/15更新*/
float x_data = 0, y_data = 0, z_data = 0;
float filtered_x_data = 0, pre_filtered_x_data = 0;
float filtered_y_data = 0, pre_filtered_y_data = 0;
float filtered_z_data = 0, pre_filtered_z_data = 0;

float get_data_z()
{
    z_data = ac.getCalculatedZ();                                       //角度センサから値を読み取る(ライブラリ)
    filtered_z_data = z_data * prob + pre_filtered_z_data * (1 - prob); //ローパスフィルタの再現
    pre_filtered_z_data = filtered_z_data;

    return (filtered_z_data);
}

float get_data_x()
{
    x_data = ac.getCalculatedX();
    filtered_x_data = x_data * prob + pre_filtered_x_data * (1 - prob);
    pre_filtered_x_data = filtered_x_data;

    float deg = atan2(filtered_x_data, filtered_z_data) * 180.0 / PI + 90; //データをdegree表記に変換

    //Serial.println(String(x_data) + "," + String(filtered_x_data) + "," + String(deg));

    return (deg);
}

float get_data_y()
{
    y_data = ac.getCalculatedY();
    filtered_y_data = y_data * prob + pre_filtered_y_data * (1 - prob);
    pre_filtered_y_data = filtered_y_data;

    float deg = atan2(filtered_y_data, filtered_z_data) * 180.0 / PI + 90;

    return (deg);
}
