void refreshLCD() {
  //---------------------------------------------------被动解析模式
  StopRefresh();
  if (CurPage == 1) {
    uint16_t color;
    //-------------------------------功率%
    //    if (PowerPercent >= 99)
    //      PowerPercent = 99;
    //    UpdateInt(13, PowerPercent);

    // if (PowerPercent >= 99)
    //   PowerPercent = 99;
    // UpdateInt(13, PowerPercent);

    //-------------------------------电流
    if (Current_mA < 0)
      color = GREEN;
    else if (Current_mA < 300000)
      color = WHITE;
    else
      color = RED;
    if (abs(Current_mA) < 10000)
      UpdateFloat(0, Current_mA / 10.0, 2, color);
    else if (abs(Current_mA) < 100000)
      UpdateFloat(0, Current_mA / 100.0, 1, color);
    else
      UpdateFloat(0, Current_mA / 1000.0, 0, color);
    //-------------------------------功率
    if (abs(power) < 1000)
      UpdateFloat(1, power, 2, color);
    else if (abs(power) < 10000)
      UpdateFloat(1, power / 10.0, 1, color);
    else
      UpdateFloat(1, power / 100.0, 0, color);
    Serial1.print("j0.val=");  //功率条
    Serial1.print(int(abs(PowerPercent)/1.25));
    End();
    Serial1.print("j0.pco=");
    if (PowerPercent < 0) {
      Serial1.print(GREEN);
    } else if (PowerPercent < 75) {
      Serial1.print(BLUE);
    } else {
      Serial1.print(RED);
    }
    End();
    Serial1.print("j1.val=");  //扭矩条
    Serial1.print(TorquePercent);
    End();
    Serial1.print("j1.pco=");
    if (Torque < 0) {
      Serial1.print(GREEN);
    } else if (TorquePercent < 75) {
      Serial1.print(BLUE);
    } else {
      Serial1.print(RED);
    }
    End();
    Serial1.print("j2.val=");  //制动力条
    Serial1.print(BrakePercent);
    End();
    //-------------------------------12V电压电流功率
    UpdateFloat(13, current12V / 10);
    UpdateFloat(12, voltage12V);
    UpdateFloat(11, (current12V / 100) * (voltage12V / 10) / 10);
    //-------------------------------电压 速度 内阻 能耗
    UpdateFloat(4, Voltage);
    UpdateFloat(3, spd, 1);
    if ((flag & 0x08) == 0x08) {
      UpdateInt(7, Ri);
      flag -= 0x08;
    }
    UpdateFloat(6, consumeRate);
    //------------------------------制热功率
    UpdateFloat(2, CompressorPower / 100);
    UpdateFloat(10, PTCPower / 100);
    //-------------------------------加速度/坡度
    //    if (spd == 0 || (abs(spd - spd_prev) < 2)) //显示坡度
    //    {
    //      UpdateFloat(9, asin(accel / 38.196 / 9.8) * 1800 / 3.1415, 1);
    //    }
    //    else  //显示加速度
    //    {
    //      UpdateFloat(9, ((spd - spd_prev) / 3.6 * 10.0) / 0.2, 2);
    //    }
    //    spd_prev = spd;
    //-------------------------------慢速刷新
    if (SlowRefresh == 5)  //
    {
      //-----------------------------------单体电压
      UpdateInt(2, MaxVolt / 10);
      UpdateInt(3, MinVolt / 10);
      UpdateInt(4, (MaxVolt - MinVolt) / 10);
      //-----------------------------------非牵引功率 SOC ODO

      if (SOC < 150)
        UpdateFloat(7, SOC, 1, RED);
      else if (SOC < 250)
        UpdateFloat(7, SOC, 1, YELLOW);
      else
        UpdateFloat(7, SOC, 1, GREEN);
      UpdateFloat(8, used_SOC, 1);
      Serial1.print("x5.val=");
      Serial1.print(ODO);
      End();
      //-----------------------------------剩余续航 电池温度
      UpdateInt(0, MaxRange);
      UpdateInt(1, RangeRemaining);
      UpdateInt(5, MaxBatProbTemp);
      UpdateInt(6, MinBatProbTemp);
      //-----------------------------------其它温度
      UpdateInt(8, interior_temp / 2 - 49);
      UpdateInt(10, Temp[3]);                   //IGBT
      UpdateInt(9, Temp[4]);                    //变流器冷却液
                                                //      UpdateInt(12, uint16_t(Temp[0])); //AC进风口
                                                //      UpdateInt(10, uint16_t(Temp[1]) / 2 - 50); //？冷却液
      UpdateInt(14, MotorCoolentTemp[0] - 50);  //PDU、MCU、电机水温
      UpdateInt(15, MotorCoolentTemp[1] - 50);  //PDU、MCU、电机水温
      UpdateInt(16, BatCoolentTemp[0] - 50);    //电池水温
      UpdateInt(17, BatCoolentTemp[1] - 50);    //电池水温
      UpdateInt(12, HeatCoreTemp[0] - 50);      //暖水箱出口温度
      UpdateInt(11, HeatCoreTemp[1] - 50);      //暖水箱进口温度
                                                //-----------------------------------------------------------主动格栅开度
      if (grillOpen >= 100) {
        Serial1.print("p0.pic=5");
        End();
      } else if (grillOpen >= 67) {
        Serial1.print("p0.pic=4");
        End();
      } else if (grillOpen >= 50) {
        Serial1.print("p0.pic=3");
        End();
      } else if (grillOpen >= 33) {
        Serial1.print("p0.pic=2");
        End();
      } else {
        Serial1.print("p0.pic=1");
        End();
      }
      //-------------------------------------冷却风扇转速
      UpdateInt(13, fanRPM * 10);

      //      Serial.print(msg644[0]);

      //    Serial.print(NonTractionPower / 5);
      //      if (NonTractionPower / 5 < 100)
      //        UpdateFloat(2, 0, 2, NO_CHANGE);
      //      else if (NonTractionPower / 5 < 1000)
      //        UpdateFloat(2, NonTractionPower / 5, 2, NO_CHANGE);
      //      else
      //        UpdateFloat(2, 999, 2, NO_CHANGE);
      //      //-----------------------------------更新0x511
      //      String str = "t34.txt=\"", str1[4];
      //      for (i = 0; i <= 3; i++) {
      //        if (msg511[i] <= 15) {
      //          str.concat("0");
      //        }
      //        str1[i] = String(msg511[i], HEX);
      //        str.concat(str1[i]);
      //        str.concat(" ");
      //      }
      //      str.concat("\"");
      //      Serial1.print(str); End();
      //      //      Serial.print("String511=");
      //      //      Serial.println(str);
      //      //-----------------------------------更新0x511
      //      str = "t37.txt=\"";
      //      if (msg507[1] <= 15) {
      //        str.concat("0");
      //      }
      //      str.concat(String(msg507, HEX));
      //      str.concat("\"");
      //      //      Serial.print("String507=");
      //      //      Serial.println(str);
      //      Serial1.print(str); End();//更新0x507
      //      //      Serial1.print(String(msg507, HEX)); End();
      SlowRefresh = 0;
    }

    Serial1.print("dim=");
    Serial1.print(bright);
    End();

  } else if (CurPage == 2 && SlowRefresh == 5) {
    if ((ChargerHandShake == 1 && BMSHandShake == 1) || ChargerCurrent > 100) {
      Serial1.print("t21.txt=\"√\"");
      End();
      Serial1.print("t21.pco=GREEN");
      End();
    } else {
      Serial1.print("t21.txt=\"×\"");
      End();
      Serial1.print("t21.pco=RED");
      End();
    }
    if (BMSReady == 0xAA || ChargerCurrent > 100) {
      Serial1.print("t17.txt=\"√\"");
      End();
      Serial1.print("t17.pco=GREEN");
      End();
    } else {
      Serial1.print("t17.txt=\"×\"");
      End();
      Serial1.print("t17.pco=RED");
      End();
    }
    if (ChargerReady == 0xAA || ChargerCurrent > 100) {
      Serial1.print("t19.txt=\"√\"");
      End();
      Serial1.print("t19.pco=GREEN");
      End();
    } else {
      Serial1.print("t19.txt=\"×\"");
      End();
      Serial1.print("t19.pco=RED");
      End();
    }
    UpdateInt(4, ChargerMaxVoltage / 10);
    UpdateInt(5, ChargerMaxCurrent / 10);
    UpdateInt(6, ChargerMaxCurrent * ChargerMaxVoltage / 100000);
    UpdateFloat(3, ChargerVoltage);
    UpdateFloat(0, ChargerCurrent);
    UpdateFloat(5, ChargerVoltage * ChargerCurrent / 10000);
    UpdateFloat(2, RequireVoltage);
    UpdateFloat(1, RequireCurrent);
    UpdateFloat(4, RequireVoltage * RequireCurrent / 10000);
    UpdateFloat(6, Voltage);
    UpdateFloat(7, abs(Current_mA) / 100);
    UpdateFloat(8, Voltage * abs(Current_mA) / 1000000);
    UpdateFloat(9, BMSVoltage);
    UpdateFloat(10, BMSCurrent);
    UpdateFloat(11, BMSVoltage * BMSCurrent / 10000);
    UpdateInt(2, MaxBatProbTemp);
    UpdateInt(3, MinBatProbTemp);
    UpdateInt(0, MaxVolt / 10);
    UpdateInt(1, MinVolt / 10);
    if (CCCV == 1) {
      Serial1.print("t22.txt=\"恒压充电\"");
      End();
    } else if (CCCV == 2) {
      Serial1.print("t22.txt=\"恒流充电\"");
      End();
    } else {
      Serial1.print("t22.txt=\"充电模式\"");
      End();
    }
    //-----------------------------------更新0x511
    String str = "t34.txt=\"", str1[4];
    for (i = 0; i <= 3; i++) {
      if (msg511[i] <= 15) {
        str.concat("0");
      }
      str1[i] = String(msg511[i], HEX);
      str.concat(str1[i]);
      str.concat(" ");
    }
    str.concat("\"");
    Serial1.print(str);
    End();
    //      Serial.print("String511=");
    //      Serial.println(str);
    //-----------------------------------更新0x511
    str = "t37.txt=\"";
    if (msg507[1] <= 15) {
      str.concat("0");
    }
    str.concat(String(msg507, HEX));
    str.concat("\"");
    //      Serial.print("String507=");
    //      Serial.println(str);
    Serial1.print(str);
    End();                                    //更新0x507
                                              //      Serial1.print(String(msg507, HEX)); End();
    UpdateInt(14, MotorCoolentTemp[0] - 50);  //PDU、MCU、电机水温
    UpdateInt(15, MotorCoolentTemp[1] - 50);  //PDU、MCU、电机水温
    UpdateInt(16, BatCoolentTemp[0] - 50);    //电池水温
    UpdateInt(17, BatCoolentTemp[1] - 50);    //电池水温
    SlowRefresh = 0;
  } else if (CurPage == 3 and SlowRefresh == 2) {
    String str;
    UpdateTxt(27, "1B0");
    UpdateTxt(36, "29A");
    UpdateTxt(45, "321");
    UpdateTxt(54, "32C");
    for (i = 0; i <= 7; i++) {  //507
      str = "";
      if (msg507[i] <= 15) {
        str.concat("0");
      }
      str.concat(String(msg507[i], HEX));
      UpdateTxt(i + 1, str);
    }
    for (i = 0; i <= 7; i++) {  //511
      str = "";
      if (msg511_2[i] <= 15) {
        str.concat("0");
      }
      str.concat(String(msg511_2[i], HEX));
      UpdateTxt(i + 10, str);
    }
    for (i = 0; i <= 7; i++) {  //505
      str = "";
      if (msg505[i] <= 15) {
        str.concat("0");
      }
      str.concat(String(msg505[i], HEX));
      UpdateTxt(i + 19, str);
    }
    for (i = 0; i <= 7; i++) {  //1B0
      str = "";
      if (msg1B0[i] <= 15) {
        str.concat("0");
      }
      str.concat(String(msg1B0[i], HEX));
      UpdateTxt(i + 28, str);
    }
    for (i = 0; i <= 7; i++) {  //29A
      str = "";
      if (msg29A[i] <= 15) {
        str.concat("0");
      }
      str.concat(String(msg29A[i], HEX));
      UpdateTxt(i + 37, str);
    }
    for (i = 0; i <= 7; i++) {  //321
      str = "";
      if (msg321[i] <= 15) {
        str.concat("0");
      }
      str.concat(String(msg321[i], HEX));
      UpdateTxt(i + 46, str);
    }
    for (i = 0; i <= 7; i++) {  //32C
      str = "";
      if (msg32C[i] <= 15) {
        str.concat("0");
      }
      str.concat(String(msg32C[i], HEX));
      UpdateTxt(i + 55, str);
    }
    SlowRefresh = 0;
  }
  StartRefresh();
}
