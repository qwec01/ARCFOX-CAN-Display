void calc() {
  uint8_t i;
  SOC = Range / 6.530;  //SOC与剩余续航一一对应
  Current_mA = (current - 25000) * 40;
  ODO = (((uint32_t)odo[1] << 24) + ((uint32_t)odo[2] << 16) + ((uint32_t)odo[3] << 8) + (uint32_t)odo[4]) / 10.0;
  Voltage = voltage * 2 / 10;
  power = (Voltage / 10.0) * (Current_mA / 1000.0) / 10.0;  //100倍,kW
  Torque = torque - 32768;
  TorquePercent = abs(Torque)/37;
  spd = abs((MotorRPM - 32768)) / 6.793;
  CompressorPower = compressor_current / 4000.0 * Voltage / 10.0;  //单位W
  PTCPower = PTC_current / 1000.0 * Voltage / 10.0;                //单位W
  if (NonTractionPowerCount == 5) {
    NonTractionPower = 0;
    NonTractionPowerCount = 0;
  }
  BrakePercent=cylinder_pressure/2;
  NonTractionPower += power - PowerPercent / 125.0 * 17000;
  NonTractionPowerCount++;
  //-------------------------------------------------内阻
  if (abs(Current_mA) < 2000)
    Voltagei = Voltage;
  if (abs(Current_mA) >= 100000 && Voltagei != 0) {
    Ri = 1000 * ((float)(Voltagei - Voltage) / Current_mA)*100;
    Voltagei = 0;
    if ((flag & 0x08) == 0)
      flag += 0x08;
    // Serial.print("dV=");
    // Serial.print(Voltagei-Voltage);

  }
  //  if (((start_SOC - SOC) < -10) && spd == 0) //检查是否充电
  //    start_SOC = SOC + trip_SOC;
  //  trip_SOC = start_SOC - SOC;
  //  used_SOC = trip_SOC + used_SOC_at_start;
  //--------------------------------------------------已用SOC累加
  if (SOCt != 0 && (Current > -10 || spd != 0)) {
    used_SOC += SOCt - SOC;
    SOCt = SOC;
  }
  //-------------------------------------------------------------计算平均续航
  if (used_SOC != 0) {
    meter_per_SOC = ((ODO - ODObeginForMaxRangeCalc) / (used_SOC / 10.0)) * 100.0;
    MaxRange = meter_per_SOC / 10.0;
    if (MaxRange > 999) {
      MaxRange = 999;
    }
  } else {
    MaxRange = KM;
  }
  RangeRemaining = MaxRange * (SOC / 1000.0);
  if (RangeRemaining > 999) {
    RangeRemaining = 999;
  }
  //电压条算法：(BOX的y2-1)-((电压-最小电压)/(最大电压-最小电压))*(条高度-2)
  //  Voltagebox = 312 - ((Voltage / 10.0 - VMin) / (VMax - VMin)) * 201;
  //  powerbox = 312 - (abs(power) / 18000.0) * 201;
  if (SOC == 0)
    RangeRemaining = 0;
  //---------------------------------------------电池温度
  MaxBatProbTemp = -50;
  MinBatProbTemp = 120;
  for (i = 0; i <= 63; i++) {
    if (MaxBatProbTemp < bat_probe_temp[i])
      MaxBatProbTemp = bat_probe_temp[i];
    if (MinBatProbTemp > bat_probe_temp[i])
      MinBatProbTemp = bat_probe_temp[i];
  }
  //---------------------------------------------单体电压最值
  MaxVolt = 10000;
  MinVolt = 50000;

  for (i = 0; i <= 95; i++) {
    if (MaxVolt < bat_cell_volt[i])
      MaxVolt = bat_cell_volt[i];
    if (MinVolt > bat_cell_volt[i])
      MinVolt = bat_cell_volt[i];
  }
  if (spd > 10 && Current_mA > 0) {
    consumeRate = power * 100.0 / spd;  //10倍
    if (consumeRate >= 999) {
      consumeRate = 999;
    }
  } else {
    consumeRate = 0;
  }
  if (MultiPacketReady == 1) {
    if (PGN == 0x06) {                                                     //PGN=0x0600,动力蓄电池参数
      BMSMaxCurrent = abs((MessageData[3] << 8) + MessageData[2] - 4000);  //10倍
      BMSMaxVoltage = abs((MessageData[7] << 8) + MessageData[6]);         //10倍
      BMSMaxTemp = MessageData[8] - 50;
      Serial.print("BMSMaxCurrent: ");
      Serial.println(BMSMaxCurrent / 10);
      Serial.print("BMSMaxVoltage: ");
      Serial.println(BMSMaxVoltage / 10.0);
      Serial.print("BMSMaxTemp: ");
      Serial.println(BMSMaxTemp);
    }
    if (PGN == 0x11) {                                                  //PGN=0x1100,电池充电信息
      BMSVoltage = abs((MessageData[1] << 8) + MessageData[0]);         //10倍
      BMSCurrent = abs((MessageData[3] << 8) + MessageData[2] - 4000);  //10倍
    }
    PGN = 0;
    uint8_t a;
    for (a = 0; a < sizeof(MessageData); a++) {
      //        Serial.println(MessageData[a],HEX);
      MessageData[a] = 0;
    }
    MultiPacketReady = 0;
  }
}
