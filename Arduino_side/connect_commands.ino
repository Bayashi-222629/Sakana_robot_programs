/*作成こばやし　2022/11/11更新　*/
void send_commands()
{
    int roop_flag = 0;
    int cmd_flag = 0;

    while (roop_flag < 1) // roop_flagが1以上でコマンドモードを終了
    {

        if (Serial.available() > 0)
        {
            rec_data = Serial.readStringUntil('|'); //「｜」が来るまで受信
            Serial.println("-Arduino-");
            Serial.println("[edit mode]");
            // Serial.println(rec_data);

            if (rec_data == "END") // END感知でroop_flagを1にする。
            {
                roop_flag++;
                Serial.println("recieved END");
            }

            if (rec_data == "ROLL")
            {
                cmd_flag++;
                Serial.println("Roll gain edit");
                for (int i = 0; i < 3; i++)
                {
                    rec_data_mini[i] = Serial.readStringUntil('-');
                    Serial.println(rec_data_mini[i]);
                    conv_data_mini[i] = rec_data_mini[i].toFloat();
                }
                Serial.println("kp_x" + String(kp_x) + "|ki_x," + String(ki_x) + "|kd_x," + String(kd_x));
                kp_x = conv_data_mini[0];
                ki_x = conv_data_mini[1];
                kd_x = conv_data_mini[2];
                Serial.println("↓↓↓");
                Serial.println("kp_x" + String(kp_x) + "|ki_x," + String(ki_x) + "|kd_x," + String(kd_x));
            }

            if (rec_data == "PITCH")
            {
                cmd_flag++;
                Serial.println("Pitch gain edit");
                for (int i = 0; i < 3; i++)
                {
                    rec_data_mini[i] = Serial.readStringUntil('-');
                    Serial.println(rec_data_mini[i]);
                    conv_data_mini[i] = rec_data_mini[i].toFloat();
                }
                Serial.println("kp_y" + String(kp_y) + "|ki_y," + String(ki_y) + "|kd_y," + String(kd_y));
                kp_y = conv_data_mini[0];
                ki_y = conv_data_mini[1];
                kd_y = conv_data_mini[2];
                Serial.println("↓↓↓");
                Serial.println("kp_y" + String(kp_y) + "|ki_y," + String(ki_y) + "|kd_y," + String(kd_y));
            }

            if (rec_data == "DEPTH")
            {
                cmd_flag++;
                Serial.println("Depth gain edit");
                rec_data_mini[0] = Serial.readStringUntil('-');
                Serial.println(rec_data_mini[0]);
                conv_data_mini[0] = rec_data_mini[0].toFloat();

                Serial.println("depth_gain" + String(depth_gain));
                depth_gain = conv_data_mini[0];
                Serial.println("↓↓↓");
                Serial.println("depth_gain" + String(depth_gain));
            }
            if (rec_data == "TG_ANGLE")
            {
                cmd_flag++;
                Serial.println("Target angle edit");
                for (int i = 0; i < 2; i++)
                {
                    rec_data_mini[i] = Serial.readStringUntil('-');
                    Serial.println(rec_data_mini[i]);
                    conv_data_mini[i] = rec_data_mini[i].toFloat();
                }
                Serial.println("target_deg_roll" + String(target_deg_x) + "|target_deg_pitch," + String(target_deg_y));
                target_deg_x = conv_data_mini[0];
                target_deg_y = conv_data_mini[1];
                Serial.println("↓↓↓");
                Serial.println("target_deg_roll" + String(target_deg_x) + "|target_deg_pitch," + String(target_deg_y));
            }
            if (rec_data == "TG_DEPTH")
            {
                cmd_flag++;
                Serial.println("Target depth edit");
                rec_data_mini[0] = Serial.readStringUntil('-');
                Serial.println(rec_data_mini[0]);
                conv_data_mini[0] = rec_data_mini[0].toFloat();

                Serial.println("depth_gain" + String(depth_gain));
                target_depth = conv_data_mini[0];
                Serial.println("↓↓↓");
                Serial.println("depth_gain" + String(depth_gain));
            }

            if (rec_data == "RG_ANGLE")
            {
                cmd_flag++;
                Serial.println("Range angle edit");
                for (int i = 0; i < 2; i++)
                {
                    rec_data_mini[i] = Serial.readStringUntil('-');
                    Serial.println(rec_data_mini[i]);
                    conv_data_mini[i] = rec_data_mini[i].toFloat();
                }
                Serial.println("deg_max" + String(deg_max) + "|deg_min," + String(deg_min));
                deg_max = conv_data_mini[0];
                deg_min = conv_data_mini[1];
                Serial.println("↓↓↓");
                Serial.println("deg_max" + String(deg_max) + "|deg_min," + String(deg_min));
            }
            if (rec_data == "STATUS")
            {
                Serial.println("・Roll_gain");
                Serial.println("kp_x" + String(kp_x) + "|ki_x," + String(ki_x) + "|kd_x," + String(kd_x));
                Serial.println("・Pitch_gain");
                Serial.println("kp_y" + String(kp_y) + "|ki_y," + String(ki_y) + "|kd_y," + String(kd_y));
                Serial.println("・Depth_gain");
                Serial.println("depth_gain" + String(depth_gain));
                Serial.println("・Target_angle");
                Serial.println("target_deg_roll" + String(target_deg_x));
                Serial.println("target_deg_pitch," + String(target_deg_y));
                Serial.println("・Target_depth");
                Serial.println("depth_gain" + String(depth_gain));
                Serial.println("・Range_angle");
                Serial.println("deg_max" + String(deg_max) + "|deg_min," + String(deg_min));
            }
            if (cmd_flag = 0)
            {
                Serial.println("[" + rec_data + "]と一致するコマンドがありません。");
            }
        }
        delay(1000);
    }
}
