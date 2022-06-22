void FastChargeReset()
{
  ChargerReady = 0; BMSReady = 0; ChargerHandShake = 0, BMSHandShake = 0, PGN = 0;
  TotalPacketNum = 0; PacketNum = 0; MultiPacketReady = 0; BMSMaxTemp = 0; CCCV = 0;
  MessageBytes = 0; ChargerVoltage = 0; ChargerCurrent = 0; ChargerMaxVoltage = 0; ChargerMaxCurrent = 0;
  BMSVoltage = 0; BMSCurrent = 0; BMSMaxVoltage = 0; BMSMaxCurrent = 0; RequireVoltage = 0; RequireCurrent = 0;
}
void ClrInterrupts(byte CS_PIN){
  digitalWrite(CS_PIN, LOW);
  SPI.transfer (BIT_MODIFY_COMMAND) ;
  SPI.transfer (CANINTF_REGISTER) ;
  SPI.transfer (0x03) ;
  SPI.transfer (0) ;
  digitalWrite(CS_PIN, HIGH);
}
void MCP2515RST()
{
  pinMode(RST_PIN, OUTPUT);
  digitalWrite (RST_PIN, HIGH);
  delay(2);
  digitalWrite(RST_PIN, LOW);
  delay(2);
  digitalWrite(RST_PIN, HIGH);
  delay(2);
}
void RespondButton() {
  char t, SerialBuffer[5];
  if (Serial1.available() > 0) {
    t = Serial1.read();
    //      Serial.println(t);
    if (t == '$') {
      Serial1.readBytes(SerialBuffer, 3);
      //        Serial.println(SerialBuffer);
    }
    if (SerialBuffer[0] == 'r' && SerialBuffer[1] == 's' && SerialBuffer[2] == 't') {//清零
#if debug
      //        Serial.println("rst data");
#endif
      if (!file.open("EEPROM.txt", O_RDWR)) {
        //          error("open failed");
#if debug
        Serial.print("File Open Failed");
#endif
      }
      for (i = 0; i < 5; i++) {
        SerialBuffer[i] = '0';
      }
      file.print(ODO);
      file.print(',');
      file.print(0);  //used_SOC
      file.print(',');
      file.print(0);
      file.print("........");
      file.close();
      ODObeginForMaxRangeCalc = ODO;
      used_SOC = 0;
      SOCt = SOC;
    }
    else if (SerialBuffer[0] == 'a' && SerialBuffer[1] == 'n' && SerialBuffer[2] == 'a') { //去分析页面
      Serial1.print("page page3"); End();
      CurPage = 3;
      SlowRefresh = 0;
      NonTractionPowerCount = 0;
    }
    else if (SerialBuffer[0] == 'r' && SerialBuffer[1] == 't' && SerialBuffer[2] == 'n') { //返回
      Serial1.print("page page1"); End();
      CurPage = 1;
      SlowRefresh = 0;
      NonTractionPowerCount = 0;
    }
  }
  dispatch();
  //----------------------------------------停车保存数据
}
void UpdateData() {
  if (spd == 0 && CheckInterval >= 5) {
    if (!file.open("EEPROM.txt", O_RDWR)) {
      //        error("open failed");
#if debug
      Serial.print("File Open Failed");
#endif
    }
    char *ptr;
    uint32_t used_SOC_temp, SOC_temp;
    ReadNextNum();
    ODObeginForMaxRangeCalc = strtoul(NumFromSD, &ptr, 10);
    ReadNextNum();
    used_SOC_temp = strtoul(NumFromSD, &ptr, 10);
    ReadNextNum();
    SOC_temp = strtoul(NumFromSD, &ptr, 10);
    //    Serial.print(used_SOC_temp); Serial.print(','); Serial.println(SOC_temp);
    file.rewind();
    if ( (used_SOC_temp != used_SOC) || (SOC_temp != SOC)) {
      file.print(ODObeginForMaxRangeCalc);
      file.print(',');
      file.print(used_SOC);  //used_SOC
      file.print(',');
      file.print(SOC);
      file.print("........");
      Serial.println("updated");
    }
    file.close();
    CheckInterval = 0;
  }
}
void clearSerialInput() {
  uint32_t m = micros();
  do {
    if (Serial.read() >= 0) {
      m = micros();
    }
  } while (micros() - m < 10000);
}
void ReadNextNum() {
  uint8_t a = 0, i = 0;
  for (i = 0; i <= 9; i++)
  {
    NumFromSD[i] = 0;
  }
  i = 0;
  while (1) {
    a = file.read();  //读下一个字节
    //    Serial.print(a);
    if (a == ',') {
      break;
    }
    NumFromSD[i] = a;
    i++;
    if (i > 9) {
      break;
    }
  }
}
void End()  //HMI屏报尾
{
  for (uint8_t i = 0; i <= 2; i++)
    Serial1.write(0xff);
  //  Serial1.println();
}


void UpdateFloat(uint8_t id, int16_t value, uint8_t dot = 0xff, uint16_t color = NO_CHANGE) //更新虚拟浮点数(id, 数值, 小数位数(0xff=不改小数位数))
{
  Serial1.print('x'); Serial1.print(id); Serial1.print(".val="); Serial1.print(value); End();
  if (dot < 0xff) {
    Serial1.print('x'); Serial1.print(id); Serial1.print(".vvs1="); Serial1.print(dot); End();
  }
  if (color != NO_CHANGE) {
    Serial1.print('x'); Serial1.print(id); Serial1.print(".pco="); Serial1.print(color); End();
  }
}

void UpdateInt(uint8_t id, int16_t value, uint8_t len = 0xff, uint16_t color = NO_CHANGE) //更新整数(id,数值，长度，oxff=不改长度)
{
  Serial1.print('n'); Serial1.print(id); Serial1.print(".val="); Serial1.print(value); End();
  if (len < 0xff) {
    Serial1.print('n'); Serial1.print(id); Serial1.print(".length="); Serial1.print(len); End();
  }
  if (color != NO_CHANGE) {
    Serial1.print('n'); Serial1.print(id); Serial1.print(".pco="); Serial1.print(color); End();
  }
}
void UpdateTxt(uint8_t id, String value, uint8_t len = 0xff, uint16_t color = NO_CHANGE) //更新文本(id,数值，长度，oxff=不改长度)
{
  value.toUpperCase();
  while (Serial1.available()) {
    Serial1.read();
  }
  Serial1.print("prints t"); Serial1.print(id); Serial1.print(".txt,3"); End();
  String in;
  in = Serial1.readString();
  Serial.println(in);
  if (in == value) {
    color = WHITE;
  }
  else {
    color = RED;
  }
  Serial1.print('t'); Serial1.print(id); Serial1.print(".txt=\""); Serial1.print(value); Serial1.print("\""); End();
  if (len < 0xff) {
    Serial1.print('t'); Serial1.print(id); Serial1.print(".txt_maxl="); Serial1.print(len); End();
  }
  if (color != NO_CHANGE) {
    Serial1.print('t'); Serial1.print(id); Serial1.print(".pco="); Serial1.print(color); End();
  }
  //  Serial1.println();
}
void StopRefresh() {
  Serial1.print("ref_stop");
  End();
}
void StartRefresh() {
  Serial1.print("ref_star");
  End();
}
//----------------------------------------------------------------------------------------//
//                                    ↓CAN报文处理子程序↓                                  //
//---------------------------------------------------------------------------------------//
static void RCV0 (const CANMessage & inMessage) {   //ID=215 电压
  if (inMessage.id == 0x50C)
  {
    PowerPercent = inMessage.data[0] - 125;

  }
  if (inMessage.id == 0x580)
  {
    AhBurnt = (inMessage.data[2] << 8) + inMessage.data[3];
    AhRegen = (inMessage.data[0] << 8) + inMessage.data[1];
  }
  if (inMessage.id == 0x582)  //车内温度
  {
    interior_temp = inMessage.data[0] - 96;
  }
  if (inMessage.id == 0x504) //剩余续航
  {
    Range = (inMessage.data[3] << 8) + (inMessage.data[4]); //100倍
  }
  if (inMessage.id == 0x503) //加速度
  {
    accel = (inMessage.data[4] << 8) + (inMessage.data[5]) - 370; //100倍
  }

  if (inMessage.id == 0x18E00009) //1 2组温度最值
  {
    MaxBatProbTemp[0] = inMessage.data[2] - 50;
    MinBatProbTemp[0] = inMessage.data[4] - 50;
  }
  if (inMessage.id == 0x18E10009) //3 4组温度最值
  {
    MaxBatProbTemp[1] = inMessage.data[2] - 50;
    MinBatProbTemp[1] = inMessage.data[4] - 50;
  }
  if (inMessage.id == 0x18E20009) //5 6组温度最值
  {
    MaxBatProbTemp[2] = inMessage.data[2] - 50;
    MinBatProbTemp[2] = inMessage.data[4] - 50;
  }
  if (inMessage.id == 0x18E30009) //7 8组温度最值
  {
    MaxBatProbTemp[3] = inMessage.data[2] - 50;
    MinBatProbTemp[3] = inMessage.data[4] - 50;
  }
  if (inMessage.id == 0x100956F4) //BMS准备就绪
  {
    FastChargeTimer = millis();
    BMSReady = inMessage.data[0];
  }
  if (inMessage.id == 0x100AF456) //充电机准备就绪
  {
    FastChargeTimer = millis();
    ChargerReady = inMessage.data[0];
  }
  if (inMessage.id == 0x511) //电池相关
  {
    msg511[0] = inMessage.data[0];
    msg511[1] = inMessage.data[1];
    msg511[2] = inMessage.data[4];
    msg511[3] = inMessage.data[7];
    for (i = 0; i <= 7; i++) {
      msg511_2[i] = inMessage.data[i];
    }
  }
  if (inMessage.id == 0x507) //电池相关
  {
    for (i = 0; i <= 7; i++) {
      msg507[i] = inMessage.data[i];
    }
  }
  if (inMessage.id == 0x505) //电池相关
  {
    for (i = 0; i <= 7; i++) {
      msg505[i] = inMessage.data[i];
    }
  }

}

static void RCV1 (const CANMessage & inMessage)   //ID=215 电压
{
  //8us
  //  Serial.println(inMessage.id);
  if ((inMessage.id & 0xFFF0000F) == 0x18E00007) //各电池模组电压最值
  {
    byte i = (inMessage.id & 0x000F0000) >> 16;
    MaxVolt[i] = (inMessage.data[2] << 8) + inMessage.data[3];
    MinVolt[i] = (inMessage.data[5] << 8) + inMessage.data[6];
  }
  if (inMessage.id == 0x641)  //ODO
  {
    uint8_t i;
    for (i = 0; i <= 2; i++)
      odo[i] = inMessage.data[i + 3];
  }
  if (inMessage.id == 0x1826F456) //充电机握手
  {
    FastChargeTimer = millis();
    ChargerHandShake = 1;
  }
  if (inMessage.id == 0x182756F4) //BMS握手
  {
    FastChargeTimer = millis();
    BMSHandShake = 1;
  }
  if (inMessage.id == 0x644)  //温度
  {
    //    Serial.println("644");
    msg644[0] = inMessage.data[0];
    msg644[1] = inMessage.data[1];
    msg644[2] = inMessage.data[4];
    msg644[3] = inMessage.data[5];
  }
}

static void RCV2 (const CANMessage & inMessage)   //ID=504 剩余续航
{
  if (inMessage.id == 0x215)  //电池组总电压
  {
    BatModuleVolt[0] = (inMessage.data[4] << 8) + inMessage.data[5];
  }

  if (inMessage.id == 0x130003C2 && inMessage.data[1] == 0x0A) //arcfox电池电流，带滤波，6-7次/s
  {
    Current = ((uint32_t)(inMessage.data[2]) << 16) + ((uint16_t)(inMessage.data[3]) << 8) + inMessage.data[4] - 0x800000;
  }
  if (inMessage.id == 0x1CEC56F4 || inMessage.id == 0x1CECF456)
  {
    FastChargeTimer = millis();
    if (inMessage.data[0] == 0x10) { //请求多包传送
      MessageBytes = (inMessage.data[2] << 8) + inMessage.data[1];
      TotalPacketNum = inMessage.data[3];
      PGN = inMessage.data[6];
      MultiPacketReady = 0;
    }
    if (inMessage.data[0] == 0x13) { //结束多包传送
      MessageBytes = 0;
      TotalPacketNum = 0;
      //      PGN=0;
      //      uint8_t i;
      //      for (i=0;i<sizeof(MessageData);i++){
      //        MessageData[i]=0;
      //      }
      MultiPacketReady = 1;
    }
  }
  if (inMessage.id == 0x1CEB56F4 || inMessage.id == 0x1CEBF456)
  {
    FastChargeTimer = millis();
    PacketNum = inMessage.data[0];
    uint8_t i;
    for (i = 0; i <= 6; i++) {
      //      if(inMessage.data[i+1]<0x10){
      //        Serial.print("0");
      //      }
      //      Serial.print(inMessage.data[i+1],HEX);
      MessageData[7 * (PacketNum - 1) + i] = inMessage.data[i + 1];
    }
  }
  if (inMessage.id == 0x29A && inMessage.data[1] == 1) {
    for (i = 0; i <= 7; i++) {
      msg29A[i] = inMessage.data[i];
    }

  }
}
static void RCV3 (const CANMessage & inMessage)   //ID=318 速度 小计里程
{
  if (inMessage.id == 0x318)  //速度
  {
    spd = (inMessage.data[5] << 8) + (inMessage.data[6]); //10倍
  }
  if (inMessage.id == 0x1808F456) {
    FastChargeTimer = millis();
    ChargerMaxVoltage = (inMessage.data[1] << 8) + inMessage.data[0];
    ChargerMaxCurrent = abs((inMessage.data[5] << 8) + inMessage.data[4] - 4000); //10倍
  }
  if (inMessage.id == 0x321) {
    for (i = 0; i <= 7; i++) {
      msg321[i] = inMessage.data[i];
    }
  }
  if (inMessage.id == 0x32C) {
    for (i = 0; i <= 7; i++) {
      msg32C[i] = inMessage.data[i];
    }
  }
    if (inMessage.id == 0x345)
  {
    Temp[0] = inMessage.data[4];//AC进风口温度
    Temp[1] = inMessage.data[5];//冷却水
    Temp[2] = inMessage.data[7];
  }
  if (inMessage.id == 0x375)
  {
    Temp[3] = inMessage.data[5] - 50; //IGBT
    Temp[4] = inMessage.data[6] - 50; //coolent
  }
}
static void RCV4 (const CANMessage & inMessage)   //ID=345 375
{
  if (inMessage.id == 0x345)
  {
    Temp[0] = inMessage.data[4];//AC进风口温度
    Temp[1] = inMessage.data[5];//冷却水
    Temp[2] = inMessage.data[7];
  }
  if (inMessage.id == 0x375)
  {
    Temp[3] = inMessage.data[5] - 50; //IGBT
    Temp[4] = inMessage.data[6] - 50; //coolent
  }
  if (inMessage.id == 0x1812F456) { //桩输出
    FastChargeTimer = millis();
    ChargerVoltage = (inMessage.data[1] << 8) + inMessage.data[0];
    ChargerCurrent = abs((inMessage.data[3] << 8) + inMessage.data[2] - 4000); //10倍
  }
  if (inMessage.id == 0x181056F4) { //车辆需求
    FastChargeTimer = millis();
    RequireVoltage = (inMessage.data[1] << 8) + inMessage.data[0];
    RequireCurrent = abs((inMessage.data[3] << 8) + inMessage.data[2] - 4000); //10倍
    CCCV = inMessage.data[4];
  }

}

static void RCV5 (const CANMessage & inMessage)   //
{
  if (inMessage.id == 0x1B0 && inMessage.data[1] == 0) {
    for (i = 0; i <= 7; i++) {
      msg1B0[i] = inMessage.data[i];
    }
  }
}
//----------------------------------------------------------------------------------------//
//                                    ↑CAN报文处理子程序↑                                  //
//----------------------------------------------------------------------------------------//

//--------------------------------------亮度滤波程序
byte brightFilter()
{
  byte x;
  BrightFilterBuf[15] = analogRead(A0);
  for (x = 0; x < 15; x++)
  {
    BrightFilterBuf[x] = BrightFilterBuf[x + 1]; // 所有数据左移，低位仍掉
    brightorg += BrightFilterBuf[x];
  }
  brightorg = brightorg / 15;
  bright = brightorg / 8.83;
  if (bright < 3) bright = 3;
  if (bright > 100) bright = 100;
  return bright;
}

//--------------------------取出CAN数据
void dispatch()
{
  //  if (Vehicle.receiveBufferCount() > CANbufferCount[0])
  //    CANbufferCount[0] = Vehicle.receiveBufferCount();
  //  if (BMS.receiveBufferCount() > CANbufferCount[1])
  //    CANbufferCount[1] = BMS.receiveBufferCount();
  while (BMS.dispatchReceivedMessage());
  while (Vehicle.dispatchReceivedMessage());
  while (FC.dispatchReceivedMessage());
}

uint8_t read2515Register (const uint8_t inRegister, uint8_t mCS) {
  delayMicroseconds(1);
  SPI.beginTransaction(SPISettings (10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(mCS, LOW);
  delayMicroseconds(1);
  SPI.transfer (READ_COMMAND) ;
  SPI.transfer (CANSTAT_REGISTER) ;
  const uint8_t readValue = SPI.transfer (0) ;
  delayMicroseconds(1);
  digitalWrite(mCS, HIGH);
  delayMicroseconds(1);
  SPI.endTransaction();
  return readValue ;
}
