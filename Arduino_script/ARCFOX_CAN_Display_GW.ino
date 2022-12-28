#include <Arduino.h>
#include <SPI.h>
#include <ACAN2515.h>
#include <SdFat.h>
#include <sdios.h>
#define debug 0   //不需要debug时设0
#define recmod 0  //1为主动解析，0为被动解析（默认）
//-----------------------------------------------------//
//                     根据车型修改数据                  //
//-----------------------------------------------------//
#define KM 653       //满电续航，km， 根据车型修改
#define ENERGY 93.6  //标称电量，kWh，根据车型修改
#define VMax 400     //充满电时的最高电压，根据车型修改
#define VMin 280     //快没电时的最低电压，根据车型修改

//---------------------------------------SD卡相关设置-----------
// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 1
// SDCARD_SS_PIN is defined for the built-in SD on some boards.
const uint8_t SD_CS_PIN = 13;
// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
#define SPI_CLOCK SD_SCK_MHZ(10)
// SD card configuration.
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK, &SPI1)
SdFat32 sd;
File32 file;
// Serial output stream
ArduinoOutStream cout(Serial);
char NumFromSD[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//------------------------------------------------------------------------------
// Store error strings in flash to save RAM.
//#define error(s) sd.errorHalt(&Serial, F(s))
//---------------------------------------------------------------
//----------------------------------颜色值-----------------
static const uint16_t WHITE = 65535;      //
static const uint16_t RED = 63488;        //
static const uint16_t BLUE = 31;          //
static const uint16_t GREEN = 2016;       //
static const uint16_t YELLOW = 65504;     //
static const uint16_t NO_CHANGE = 65534;  //不更改颜色   //
//------------------------------------------------------//
static const uint8_t WRITE_COMMAND = 0x02;
static const uint8_t READ_COMMAND = 0x03;
static const uint8_t CANSTAT_REGISTER = 0x0E;
static const uint8_t CANINTF_REGISTER = 0x2C;
static const uint8_t BIT_MODIFY_COMMAND = 0x05;
static const uint8_t RST_PIN = 14;
static const uint16_t RatedVoltage = 3398;  //电池额定电压339.8V
static const uint16_t RatedAh = 2756;       //电池额定容量275.6Ah
static const uint16_t Mass = 2125;          //整备质量kg
//CAN报文计算用变量
uint16_t current = 25000, voltage, torque=32768, MotorRPM = 32768, odo[5], current12V, voltage12V, compressor_current, PTC_current, bat_cell_volt[96];
uint8_t fanRPM, grillOpen,interior_temp;
int8_t bat_probe_temp[64];
uint16_t cylinder_pressure;
//屏显示用变量
uint16_t Current12V, Voltage12V, CompressorPower, PTCPower;
uint16_t x1 = 0, AhBurnt, AhRegen, Range, MaxVolt, MinVolt, spd, spd_prev;
uint16_t BrightFilterBuf[16] = { 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024 }, brightorg;
uint16_t Voltage, Voltagei, Ri, start_SOC, SOC, SOCt, trip_SOC, used_SOC, /*used_SOC_at_start,*/ last_stop_SOC, meter_per_SOC, MaxRange, RangeRemaining;
uint16_t Voltagebox, powerbox;
int16_t power, accel, consumeRate,Torque;
int8_t PowerPercent,  MaxBatProbTemp, MinBatProbTemp, Temp[8],MotorCoolentTemp[2],BatCoolentTemp[2],HeatCoreTemp[2];
uint8_t dot = 2, SlowRefresh, bright, CANbufferCount[2] = { 0, 0 };

uint8_t NonTractionPowerCount, flag,TorquePercent,BrakePercent;
uint8_t i, CheckInterval;
uint32_t runtime, ODO, brightruntime, ODObeginForMaxRangeCalc;
int32_t NonTractionPower;
static const uint32_t XTAL = 8000000;  //晶振8M
long Current, Current_mA;
//以下快充CAN专用变量
uint8_t FastCharge, CurPage, ChargerReady, BMSReady, ChargerHandShake = 0, BMSHandShake = 0, PGN = 0;
uint8_t TotalPacketNum, PacketNum, MessageData[64], MultiPacketReady, BMSMaxTemp, CCCV = 0;
uint16_t MessageBytes, ChargerVoltage, ChargerCurrent, ChargerMaxVoltage, ChargerMaxCurrent;
uint16_t BMSVoltage, BMSCurrent, BMSMaxVoltage, BMSMaxCurrent, RequireVoltage, RequireCurrent;
uint32_t FastChargeTimer = 0;
static const uint32_t FastChargeTimeout = 10000;  //ms
uint8_t msg511[4], msg511_2[8], msg507[8], msg505[8], msg1B0[8], msg29A[8], msg321[8], msg32C[8];


static const byte MCP2515_SCK = 2;   // SCK input of MCP2515
static const byte MCP2515_MOSI = 3;  // SDI input of MCP2515
static const byte MCP2515_MISO = 4;  // SDO output of MCP2517
static const byte EVBUS_CS = 5;    // EVBUS CAN线CS
static const byte EVBUS_INT = 21;  // EVBUS CAN线INT
static const byte FC_CS = 6;         // 快充CAN线
static const byte FC_INT = 20;       // 快充CAN线
static const byte IBUS1_CS = 7;        // IBUS1 CAN线
static const byte IBUS1_INT = 19;      // IBUS1 CAN线
ACAN2515 EVBUS(EVBUS_CS, SPI, EVBUS_INT);
ACAN2515 FC(FC_CS, SPI, FC_INT);
ACAN2515 IBUS1(IBUS1_CS, SPI, IBUS1_INT);
ACAN2515Mask rxm1 = standard2515Mask(0x7FF, 0, 0);
ACAN2515Settings settings(XTAL, 500UL * 1000UL);  //8Mhz 500kbps
void setup() {
  delay(2000);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  //while (!Serial);//可能是bug, 直接上电卡在这一步
  digitalWrite(LED_BUILTIN, LOW);
  Serial1.begin(115200);
  while (!Serial1)
    ;
  pinMode(EVBUS_CS, OUTPUT);
  pinMode(FC_CS, OUTPUT);
  pinMode(IBUS1_CS, OUTPUT);
  digitalWrite(EVBUS_CS, HIGH);
  digitalWrite(FC_CS, HIGH);
  digitalWrite(IBUS1_CS, HIGH);
  MCP2515RST();
  delay(50);
  Serial1.print("page page0");
  End();
  delay(200);
  //  Serial.println();
  //--- There are no default SPI pins so they must be explicitly assigned
  SPI.setSCK(MCP2515_SCK);
  SPI.setTX(MCP2515_MOSI);
  SPI.setRX(MCP2515_MISO);
  SPI.begin();

  Serial1.print("xstr 0,200,200,16,9,WHITE,BLACK,0,1,1,\"Starting MCP22515.\"");
  End();

  //  while(1);
  //  while (Vehicle.dispatchReceivedMessage());
  //------------------------------------------------↓初始化快充 CAN线
  {
    settings.mRequestedMode = ACAN2515Settings::NormalMode;
    settings.mReceiveBufferSize = 16;
    settings.mTransmitBuffer0Size = 0;
    ACAN2515Settings settings(XTAL, 250000);           //8Mhz 250kbps
    ACAN2515Mask rxm0 = extended2515Mask(0x1FF80000);  //前2
    ACAN2515Mask rxm1 = extended2515Mask(0x1FF80000);  //后4
    const ACAN2515AcceptanceFilter filters[] = {
      { extended2515Filter(0x100956F4), RCV0 },  //1009 100A BMS、充电机准备就绪
      { extended2515Filter(0x182756F4), RCV1 },  //握手
      { extended2515Filter(0x1CEC56F4), RCV2 },  //多包传送
      { extended2515Filter(0x1808F456), RCV3 },  //
      { extended2515Filter(0x1812F456), RCV4 },  //桩输出
      { extended2515Filter(0x18FFFFFF), RCV5 }   //
    };

    uint16_t errorCode = FC.begin(
      settings, [] {
        FC.isr();
      },
      rxm0, rxm1, filters, 6);
    if (errorCode == 0) {
      Serial1.print("xstr 0,0,200,16,9,WHITE,BLACK,0,1,1,\"FC CAN Succ\"");
      End();
    } else {
      Serial.print("FC CAN Fail 0x");
      Serial.println(errorCode, HEX);
      Serial1.print("xstr 0,0,200,16,9,WHITE,BLACK,0,1,1,\"FC CAN Fail 0x");
      Serial1.print(errorCode, HEX);
      Serial1.print('\"');
      End();
    }
  }
  delay(50);
  //------------------------------------------------↓初始化EVCAN-----------------------
  {
    settings.mRequestedMode = ACAN2515Settings::NormalMode;
    settings.mReceiveBufferSize = 32;
    settings.mTransmitBuffer0Size = 0;
    ACAN2515Settings settings(8000000, 500000);            //8Mhz 500kbps
    ACAN2515Mask rxm0 = standard2515Mask(0x792, 0, 0x04);  //前2 只管第2字节低3位的生命信号是x4或xC时通过filter，
    ACAN2515Mask rxm1 = standard2515Mask(0x700, 0, 0);     //后4
    const ACAN2515AcceptanceFilter filters[] = {
      { standard2515Filter(0x7FF, 0, 0), RCV0 },  //
      { standard2515Filter(0x7FF, 0, 0), RCV1 },  //641ODO 644温度
      { standard2515Filter(0x7FF, 0, 0), RCV2 },  //215 29A
      { standard2515Filter(0x7FF, 0, 0), RCV3 },  //318速度 里程 50/s
      { standard2515Filter(0x7FF, 0, 0), RCV4 },  //345 375温度 345
      { standard2515Filter(0x7FF, 0, 0), RCV5 }   //
    };
    //    while (BMS.dispatchReceivedMessage() && FC.dispatchReceivedMessage());
    uint16_t errorCode = 255;
    i = 1;
    while (errorCode != 0) {
      errorCode = EVBUS.begin(
        settings, [] {
          EVBUS.isr();
        },
        rxm0, rxm1, filters, 6);
      //      errorCode = Vehicle.begin (settings, [] { Vehicle.isr (); });
      if (errorCode == 0) {
        Serial1.print("xstr 0,32,300,16,9,WHITE,BLACK,0,1,1,\"EVBUS CAN Succ\"");
        End();
#if debug
        Serial.println("DBG: EVBUS CAN Success.");
#endif
      } else {
#if debug
        Serial.print("DBG: EVBUS CAN Fail 0x");
        Serial.println(errorCode, HEX);
#endif
        Serial1.print("xstr 0,32,300,16,9,WHITE,BLACK,0,1,1,\"EVBUS CAN Fail 0x");
        Serial1.print(errorCode, HEX);
        Serial1.print(" retried ");
        Serial1.print(i);
        Serial1.print('\"');
        End();
        i++;
      }
      delay(100);
    }
  }
  //------------------------------------------------↓初始化IBUS1 CAN线
  i = 0;
  {
    settings.mRequestedMode = ACAN2515Settings::NormalMode;
    settings.mReceiveBufferSize = 32;
    settings.mTransmitBuffer0Size = 0;
    //    ACAN2515Settings settings (XTAL, 500000); //8Mhz 500kbps
    ACAN2515Mask rxm0 = standard2515Mask(0x7FF, 0, 0);  //前2
    ACAN2515Mask rxm1 = standard2515Mask(0x7FF, 0, 0);  //后4
    const ACAN2515AcceptanceFilter filters[] = {
      { standard2515Filter(0x7FF, 0, 0), RCV0 },  //
      { standard2515Filter(0x7FF, 0, 0), RCV1 },  //
      { standard2515Filter(0x7FF, 0, 0), RCV2 },  //
      { standard2515Filter(0x7FF, 0, 0), RCV3 },  //
      { standard2515Filter(0x7FF, 0, 0), RCV4 },  //
      { standard2515Filter(0x7FF, 0, 0), RCV5 }   //
    };
    //    while (FC.dispatchReceivedMessage());
    uint16_t errorCode = 255;
    //    uint16_t errorCode = BMS.begin (settings, [] { BMS.isr (); }, rxm0, rxm1, filters, 6);

    while (errorCode != 0) {
      errorCode = IBUS1.begin(
        settings, [] {
          IBUS1.isr();
        },
        rxm0, rxm1, filters, 6);
      if (errorCode == 0) {

        Serial1.print("xstr 0,16,300,16,9,WHITE,BLACK,0,1,1,\"IBUS1 CAN Succ\"");
        End();
#if debug
        Serial.println("DBG: IBUS1 CAN Success.");
#endif
      } else {
#if debug
        Serial.print("DBG: IBUS1 CAN Fail 0x");
        Serial.println(errorCode, HEX);
#endif
        Serial1.print("xstr 0,16,300,16,9,WHITE,BLACK,0,1,1,\"IBUS1 CAN Fail 0x");
        Serial1.print(errorCode, HEX);
        Serial1.print(" retried ");
        Serial1.print(i);
        Serial1.print('\"');
        End();
        i++;
      }
      delay(100);
    }
  }


  //--set filters--------------------------------------------
  {//-----------------------------------------------IBUS1---------------------------------
    ACAN2515Mask rxm0 = standard2515Mask(0x7FF, 0, 0x04);  //前2 只管第2字节低3位的生命信号是x4或xC时通过filter
    ACAN2515Mask rxm1 = standard2515Mask(0x7FF, 0, 0);  //后4
    const ACAN2515AcceptanceFilter filters[] = {
      { standard2515Filter(0x120, 0, 0x04), RCV0 },  //制动系统数据
      { standard2515Filter(0x700, 0, 0), RCV1 },  //
      { standard2515Filter(0x700, 0, 0), RCV2 },  //
      { standard2515Filter(0x700, 0, 0), RCV3 },  //
      { standard2515Filter(0x700, 0, 0), RCV4 },  //
      { standard2515Filter(0x700, 0, 0), RCV5 }   //
    };
    const uint16_t errorCode = IBUS1.setFiltersOnTheFly(rxm0, rxm1, filters, 6);
    if (errorCode != 0) {
#if debug
      Serial.print("IBUS1 Filter Fail 0x");
      Serial.println(errorCode, HEX);
#endif
    } else {
      Serial1.print("xstr 0,48,200,16,9,WHITE,BLACK,0,1,1,\"IBUS1 Filter Set\"");
      End();
    }
  }  //----------------------------------------------------------------------------------

  {//---------------------------------------------EVBUS--------------------------------
    ACAN2515Mask rxm0 = standard2515Mask(0x792, 0, 0x04);  //前2 只管第2字节低3位的生命信号是x4或xC时通过filter，
    ACAN2515Mask rxm1 = standard2515Mask(0x700, 0, 0);     //后4
    const ACAN2515AcceptanceFilter filters[] = {
      { standard2515Filter(0x150, 0, 0x04), RCV0 },  //150电机速度 扭矩
      { standard2515Filter(0x220, 0, 0x04), RCV1 },  //220牵引电流 电压 225扭矩限制 248电流
      { standard2515Filter(0x300, 0, 0), RCV2 },     //3xx，218条/s
      { standard2515Filter(0x400, 0, 0), RCV3 },     //4xx，60条/s
      { standard2515Filter(0x500, 0, 0), RCV4 },     //5xx，323条/s
      { standard2515Filter(0x641, 0, 0), RCV5 }      //6xx
    };
    const uint16_t errorCode = EVBUS.setFiltersOnTheFly(rxm0, rxm1, filters, 6);
    if (errorCode != 0) {
#if debug
      Serial.print("EVBUS Filter Fail 0x");
      Serial.println(errorCode, HEX);
#endif
    } else {
      Serial1.print("xstr 0,64,200,16,9,WHITE,BLACK,0,1,1,\"EVBUS Filter Set\"");
      End();
    }
  }
  //-------------------------------------------------------------------------------------


  //------------------------------------------------↑初始化车辆 CAN线
  delay(50);
  //-------------------------------------------------初始化SD卡
  SPI1.setSCK(10);
  SPI1.setTX(11);
  SPI1.setRX(12);
  if (!sd.begin(SD_CONFIG)) {
    //    sd.initErrorHalt(&Serial);
#if debug
    Serial.println("SD Begin Failed");
    Serial1.print("xstr 0,80,300,16,9,WHITE,BLACK,0,1,1,\"SD Card Fail\"");
    End();
#endif
  } else {
#if debug
    Serial.println("SD Begin Done.");
    Serial1.print("xstr 0,80,300,16,9,WHITE,BLACK,0,1,1,\"SD Card Init Success\"");
    End();
#endif
  }

  if (!file.open("EEPROM.txt", O_READ)) {
    //    error("open failed");
#if debug
    Serial.print("File Open Failed");
    Serial1.print("xstr 0,96,300,16,9,WHITE,BLACK,0,1,1,\"File Open Failed\"");
    End();
#endif
  }
  //读取SD卡数据
  char *ptr;
  ReadNextNum();
  ODObeginForMaxRangeCalc = strtoul(NumFromSD, &ptr, 10);
  ReadNextNum();
  used_SOC = strtoul(NumFromSD, &ptr, 10);
  ReadNextNum();
  last_stop_SOC = strtoul(NumFromSD, &ptr, 10);
  file.close();
#if debug
  Serial.print("ODObeginForMaxRangeCalc:");
  Serial.println(ODObeginForMaxRangeCalc / 10.0);
  Serial.print("used_SOC:");
  Serial.println(used_SOC / 10.0);
  Serial.print("last_stop_SOC:");
  Serial.println(last_stop_SOC / 10.0);
#endif
  dispatch();
  delay(500);  //清空启动时的垃圾数据
  dispatch();
  delay(500);  //清空启动时的垃圾数据
  //  ODO = (odo[0] * 65536 + (unsigned long)odo[1] * 256 + odo[2])/10.0;
  //  ODObeginForMaxRangeCalc=ODO; //取当前ODO作为初始值
  runtime = millis();
  while ((millis() < runtime + 1200) && SOC == 0) {
    dispatch();
    SOC = Range / 6.530;
    SOCt = SOC;
  }
  runtime = millis();
  while ((millis() < runtime + 1200) && ODO == 0) {
    dispatch();
    ODO = ((uint32_t)odo[0] * 65536 + (uint32_t)odo[1] * 256 + odo[2]) / 10.0;
    //    ODObeginForMaxRangeCalc = ODO; //取当前ODO作为初始值
  }

  if (SOC > 0) {
    start_SOC = SOC;
  } else {
    //start_SOC=last_stop_SOC;
  }
  //↓计算停车掉电
  if (start_SOC < last_stop_SOC && SOC > 0) {
    Serial.print(last_stop_SOC);
    used_SOC = used_SOC + (last_stop_SOC - start_SOC);
  }
  if (SOC > 0) {
    Serial1.print("xstr 0,96,200,16,9,WHITE,BLACK,0,1,1,\"Lost SOC when stop: ");
    if(used_SOC<0)
      Serial1.print("0.00");
    else
      Serial1.print((last_stop_SOC - start_SOC) / 10.0);
    Serial1.print("%\"");
    End();
  }
  //  used_SOC_at_start = used_SOC;
  dispatch();
  delay(3000);
  dispatch();
  Serial1.print("page page1");
  End();
  CurPage = 1;
  delay(100);
  SlowRefresh = 4;  //启动时立即刷新慢速数据
  NonTractionPowerCount = 4;
  //  CurPage=3;
  //  Serial1.print("page page3");End();
  Serial1.setTimeout(10);
  delay(1000);
}

void loop() {
  dispatch();
  if (millis() - FastChargeTimer > FastChargeTimeout || millis() < FastChargeTimeout) {
    FastCharge = 0;  //非快充中
    FastChargeReset();
  } else {
    FastCharge = 1;  //快充中
  }
  if (millis() > runtime + 200) {

    if (FastCharge == 1 && CurPage != 2 && CurPage != 3) {
      Serial1.print("page page2");
      End();
      CurPage = 2;
      BMSHandShake = 0;
      ChargerHandShake = 0;
      BMSReady = 0;
      ChargerReady = 0;
    }
    if (FastCharge == 0 && CurPage != 1 && CurPage != 3) {
      Serial1.print("page page1");
      End();
      CurPage = 1;
    }

    bright = brightFilter();
    calc();
    SlowRefresh++;
    CheckInterval++;
    refreshLCD();
    //    dispatch();
    UpdateData();
    RespondButton();
    runtime = millis();
  }
}
