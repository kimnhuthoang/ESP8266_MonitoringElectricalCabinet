#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <BlynkSimpleEsp8266.h>
#include <ModbusMaster.h>
#include <PCF8574.h>

PCF8574 pcf8574(0x20);

WiFiEventHandler disconnectedEventHandler;

// instantiate ModbusMaster object
ModbusMaster MFM383A;
ModbusMaster SHT20;

//------------------------------------------------------------------------//
#define ON  1
#define OFF 0

#define X0(Port) ((Port >> 2)&0x01)
#define X1(Port) ((Port >> 3)&0x01)
#define X2(Port) ((Port >> 4)&0x01)
#define X3(Port) ((Port >> 5)&0x01)
#define X4(Port) ((Port >> 6)&0x01)
#define X5(Port) ((Port >> 7)&0x01)

#define Y0 P0
#define Y1 P1
//------------------------------------------------------------------------//
#define LED_STATUS 2
#define timeReadMFM383A 2000 //miliseconds
#define timeReadSHT20   4000 //miliseconds
#define EEP_Temp 100 //EEPROM Address Setting Temperature
#define EEP_Mode 105 //EEPROM Address Setting Mode Auto/Manual Fan
#define AUTO_MODE   0
#define MANUAL_MODE 1
//------------------------------------------------------------------------//
//#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL6KG1JWIUQ"
#define BLYNK_TEMPLATE_NAME "MonitoringCapacitorESP8266MainBoard"
#define BLYNK_AUTH_TOKEN "J2rZbDFnnSVebfQWIgweE6nEBlliTV5v"

const char ssid[] = "Ethernet2";
const char pass[] = "4422336688";

WidgetTerminal terminal(V7);
//------------------------------------------------------------------------//
time_t now;
//------------------------------------------------------------------------//
uint16_t MFM383A_Buffer[60];
float temperatureSet = 0.00f;
uint8_t fanMode;
uint8_t WifiConnectTimeOut = 0;
//------------------------------------------------------------------------//
uint32_t waitReadSHT20 = millis();
uint32_t waitReadMFM383A = millis();
uint32_t waitScanInput = millis();
uint32_t waitcheckConnectWifi = millis();
//------------------------------------------------------------------------//
bool readData_MFM383A (uint16_t *Buffer);
float dataModbustoFloat(uint16_t DecWord, uint16_t IntWord);
void NTPConnect(void);
void EEPROM_writeFloat(uint16_t address, float f);
float EEPROM_readFloat(uint16_t address);
//------------------------------------------------------------------------//

void setup() {

  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, HIGH);

  disconnectedEventHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& event)
  {
    WiFi.disconnect();
  });

  ESP.wdtDisable();
  ESP.wdtEnable(10000);

  Serial.begin(19200);
  //ModbusSerial.begin(19200);
  EEPROM.begin(512);

  pcf8574.pinMode(Y0, OUTPUT, HIGH);
  pcf8574.pinMode(Y1, OUTPUT, HIGH);
  for (uint8_t X = 2; X < 8; X++)
  {
    pcf8574.pinMode(X, INPUT_PULLUP);
  }
  pcf8574.begin();

  MFM383A.begin(1, Serial);
  MFM383A.idle(yield);
  SHT20.begin(2, Serial);
  SHT20.idle(yield);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  terminal.clear();
  terminal.flush();
  temperatureSet = EEPROM_readFloat(EEP_Temp);
  fanMode = EEPROM.read(EEP_Mode);
  //Serial.println(temperatureSet);
  NTPConnect();
}

void loop() {

  ESP.wdtFeed();

  Blynk.run();

  if (millis() - waitcheckConnectWifi >= 1000) {
    if (WiFi.status() == WL_CONNECTED) {
      WifiConnectTimeOut = 0;
      digitalWrite(LED_STATUS, !digitalRead(LED_STATUS));
    } else {
      WifiConnectTimeOut++;
      if (WifiConnectTimeOut == 60) {
        ESP.restart();
        while(1);
      }
      digitalWrite(LED_STATUS, HIGH);
    }
  }

  if (millis() - waitScanInput >= 500) {
    byte Input = pcf8574.digitalReadAll(); 
    if (X0(Input) == ON) {
      Blynk.virtualWrite(V4, "ON");
    } else {
      Blynk.virtualWrite(V4, "OFF");
    }

    if (X1(Input) == ON) {
      Blynk.virtualWrite(V5, "ON");
    } else {
      Blynk.virtualWrite(V5, "OFF");
    }

    if (X2(Input) == ON) {
      Blynk.virtualWrite(V6, "ON");
    } else {
      Blynk.virtualWrite(V6, "OFF");
    }

    waitScanInput = millis();
  }

  if (millis() - waitReadMFM383A >= (timeReadMFM383A/10)) {
    if (readData_MFM383A(MFM383A_Buffer)) { 
      float mfmV1N = dataModbustoFloat(MFM383A_Buffer[0], MFM383A_Buffer[1]);
      float mfmV2N = dataModbustoFloat(MFM383A_Buffer[2], MFM383A_Buffer[3]);
      float mfmV3N = dataModbustoFloat(MFM383A_Buffer[4], MFM383A_Buffer[5]);
      float mfmAvgVLN = dataModbustoFloat(MFM383A_Buffer[6], MFM383A_Buffer[7]);

      float mfmV12 = dataModbustoFloat(MFM383A_Buffer[8], MFM383A_Buffer[9]);
      float mfmV23 = dataModbustoFloat(MFM383A_Buffer[10], MFM383A_Buffer[11]);
      float mfmV31 = dataModbustoFloat(MFM383A_Buffer[12], MFM383A_Buffer[13]);
      float mfmAvgVLL = dataModbustoFloat(MFM383A_Buffer[14], MFM383A_Buffer[15]);

      float mfmI1 = dataModbustoFloat(MFM383A_Buffer[16], MFM383A_Buffer[17]);
      float mfmI2 = dataModbustoFloat(MFM383A_Buffer[18], MFM383A_Buffer[19]);
      float mfmI3 = dataModbustoFloat(MFM383A_Buffer[20], MFM383A_Buffer[21]);
      float mfmAvgI= dataModbustoFloat(MFM383A_Buffer[22], MFM383A_Buffer[23]);

      float mfmkW1 = dataModbustoFloat(MFM383A_Buffer[24], MFM383A_Buffer[25]);
      float mfmkW2 = dataModbustoFloat(MFM383A_Buffer[26], MFM383A_Buffer[27]);
      float mfmkW3 = dataModbustoFloat(MFM383A_Buffer[28], MFM383A_Buffer[29]);

      float mfmkVA1 = dataModbustoFloat(MFM383A_Buffer[30], MFM383A_Buffer[31]);
      float mfmkVA2 = dataModbustoFloat(MFM383A_Buffer[32], MFM383A_Buffer[33]);
      float mfmkVA3 = dataModbustoFloat(MFM383A_Buffer[34], MFM383A_Buffer[35]);

      float mfmkVAr1 = dataModbustoFloat(MFM383A_Buffer[36], MFM383A_Buffer[37]);
      float mfmkVAr2 = dataModbustoFloat(MFM383A_Buffer[38], MFM383A_Buffer[39]);
      float mfmkVAr3 = dataModbustoFloat(MFM383A_Buffer[40], MFM383A_Buffer[41]);

      float mfmTotalkW = dataModbustoFloat(MFM383A_Buffer[42], MFM383A_Buffer[43]);
      float mfmTotalkVA = dataModbustoFloat(MFM383A_Buffer[44], MFM383A_Buffer[45]);
      float mfmTotalkVAr = dataModbustoFloat(MFM383A_Buffer[46], MFM383A_Buffer[47]);

      float mfmPF1 = dataModbustoFloat(MFM383A_Buffer[48], MFM383A_Buffer[49]);
      float mfmPF2 = dataModbustoFloat(MFM383A_Buffer[50], MFM383A_Buffer[51]);
      float mfmPF3 = dataModbustoFloat(MFM383A_Buffer[52], MFM383A_Buffer[53]);
      float mfmAvgPF = dataModbustoFloat(MFM383A_Buffer[54], MFM383A_Buffer[55]);

      float mfmFrequency = dataModbustoFloat(MFM383A_Buffer[56], MFM383A_Buffer[57]);
      float mfmEnergy = dataModbustoFloat(MFM383A_Buffer[58], MFM383A_Buffer[59]);

      for (uint8_t i = 0; i < 60; i++) {
        MFM383A_Buffer[i] = 0;
      }
      
      Blynk.virtualWrite(V0, mfmPF1);
      Blynk.virtualWrite(V1, mfmPF2);
      Blynk.virtualWrite(V2, mfmPF3);
      Blynk.virtualWrite(V3, mfmAvgPF);

      char buffer_time[40];
      if(WiFi.status() == WL_CONNECTED) {
        struct tm *time_info;
        time(&now);
        time_info = localtime(&now);
        strftime(buffer_time,80,"%A, %d/%m/%Y, %I:%M:%S %p", time_info);
      }

      String MFM383A_TerminalBuff = "> " + String(buffer_time) +
                                    "\n> V1N: " + String(mfmV1N) + " V  |  " + "V2N: " + String(mfmV2N) + " V  |  " 
                                    + "V3N: " + String(mfmV3N) + " V  |  " + "AVG VLN: " + String(mfmAvgVLN) + " V" +

                                    "\n> V12: " + String(mfmV12) + " V  |  " + "V23: " + String(mfmV23) + " V  |  " 
                                    + "V31: " + String(mfmV31) + " V  |  " + "AVG VLL: " + String(mfmAvgVLL) + " V" +

                                    "\n> I1: " + String(mfmI1) + " A  |  " + "I2: " + String(mfmI2) + " A  |  " 
                                    + "I3: " + String(mfmI3) + " A  |  " + "AVG I: " + String(mfmAvgI) + " A" +

                                    "\n> kW1: " + String(mfmkW1) + "  |  " + "kW2: " + String(mfmkW2) + "  |  " 
                                    + "kW3: " + String(mfmkW3) + "  |  " + "Total kW: " + String(mfmTotalkW) +

                                    "\n> kVA1: " + String(mfmkVA1) + "  |  " + "kVA2: " + String(mfmkVA2) + "  |  " 
                                    + "kVA3: " + String(mfmkVA3) + "  |  " + "Total kVA: " + String(mfmTotalkVA) +

                                    "\n> kVAr1: " + String(mfmkVAr1) + "  |  " + "kVAr2: " + String(mfmkVAr2) + "  |  " 
                                    + "kVAr3: " + String(mfmkVAr3) + "  |  " + "Total kVAr: " + String(mfmTotalkVAr) +

                                    "\n> Frequency (Hz): " + String(mfmFrequency) +
                                    "\n> Energy (kWh): " + String(mfmEnergy) +
                                    "\n-----------------------------------------------------------------------";

      terminal.println(MFM383A_TerminalBuff);
      terminal.flush();
    }
    waitReadMFM383A = millis();
  }
  

  if (millis() - waitReadSHT20 >= timeReadSHT20) {
    static uint8_t readSHT20_fail = 0;
    uint8_t result_2 = SHT20.readInputRegisters(0x0001, 2);
    if (result_2 == SHT20.ku8MBSuccess) {
      readSHT20_fail = 0;
      float temp = (SHT20.getResponseBuffer(0)/10)/10.0f;
      float humi = (SHT20.getResponseBuffer(1)/10)/10.0f;
      Blynk.virtualWrite(V8, temp);
      Blynk.virtualWrite(V9, humi);
      if (fanMode == AUTO_MODE) {
        if (temp >= temperatureSet) {
          pcf8574.digitalWrite(Y0, LOW);
        } else {
          pcf8574.digitalWrite(Y0, HIGH);
        }
      }
      //Serial.print("Temperature: "); Serial.println((slave_2.getResponseBuffer(0)/10)/10.0f);
      //Serial.print("Humidity: "); Serial.println((slave_2.getResponseBuffer(1)/10)/10.0f);
      //Serial.println("-----------------------------------------------------------------");
      SHT20.clearResponseBuffer();
    } 
    else {
      readSHT20_fail++;
      if (readSHT20_fail == 3) {
        readSHT20_fail = 0;
        Blynk.virtualWrite(V8, 0);
        Blynk.virtualWrite(V9, 0);
      }
    }
    waitReadSHT20 = millis();
  }
}

//-------------------------------------------------------------------------------------------------//
//Blynk VirtualPin
BLYNK_WRITE(V7) {
  String strVal= param.asStr();
  String subStrVal = strVal.substring(0, 7);
  //Serial.println(strVal); SETTEMP:30.0, GETTEMP, SETMODE:AT or MN, GETMODE
  if (subStrVal == String("SETTEMP")) {
    if (strVal.length() == 12) {
      temperatureSet = (strVal.substring(8, 12)).toFloat();
      EEPROM_writeFloat(EEP_Temp, temperatureSet);
    } 
    else {
      terminal.println("Syntax Error !!!");
      terminal.flush();
    }
  }
  else if (subStrVal == String("GETTEMP")) {
    terminal.println("Temperature set is: " + String(temperatureSet));
    terminal.flush();
  }
  else if (subStrVal == String("SETMODE")) {
    if (strVal.substring(8, 10) == String("AT")) {
      fanMode = 0;
      EEPROM.write(EEP_Mode, fanMode);
      EEPROM.commit();
    }
    else if (strVal.substring(8, 10) == String("MN")) {
      fanMode = 1;
      EEPROM.write(EEP_Mode, fanMode);
      EEPROM.commit();
    }
    else {
      terminal.println("Syntax Error !!!");
      terminal.flush();
    }
  }
  else if (subStrVal == String("GETMODE")) {
    if (fanMode == 0) {
      terminal.println("FAN operating Mode is: AUTO");
    } 
    else {
      terminal.println("FAN operating Mode is: MANUAL");
    }
    terminal.flush();
  } 
  else {
    terminal.println("Syntax Error !!!");
    terminal.flush();
  }
}
//-------------------------------------------------------------------------------------------------//
bool readData_MFM383A (uint16_t *Buffer) {
  static uint8_t readTime = 0;
  if (MFM383A.readInputRegisters(readTime*6, 6) == MFM383A.ku8MBSuccess)
  {
    for (uint8_t i = 0; i < 6; i++)
    {
      Buffer[i + readTime*6] = MFM383A.getResponseBuffer(i);
    }
    MFM383A.clearResponseBuffer();
  }
  readTime++;
  if (readTime == 10)
  {
    readTime = 0;
    return true;
  }
  return false;
}

float dataModbustoFloat(uint16_t DecWord, uint16_t IntWord) {
  union
  { 
    uint32_t dataWord;
    float dataFloat;
  } Data;
  Data.dataWord = (uint32_t)IntWord << 16 | DecWord;
  return Data.dataFloat;
}

//-------------------------------------------------------------------------------------------------//
void NTPConnect(void)
{
  uint16_t i = 2000;
  //Serial.print("Setting time using SNTP");
  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  now = time(nullptr);
  while(i--) {
    now = time(nullptr);
    delay(1);
  }
  delay(1000);
}

//-------------------------------------------------------------------------------------------------//
void EEPROM_writeFloat(uint16_t address, float f) {
  union
  {
    float x;
    uint8_t b[4];
  }f_data;
  
  f_data.x = f;
  EEPROM.write(address, f_data.b[0]);
  EEPROM.write(address + 1, f_data.b[1]);
  EEPROM.write(address + 2, f_data.b[2]);
  EEPROM.write(address + 3, f_data.b[3]);
  EEPROM.commit();
}

float EEPROM_readFloat(uint16_t address) {
  union
  {
    float x;
    uint8_t b[4];
  }f_data;
  f_data.b[0] = EEPROM.read(address);
  f_data.b[1] = EEPROM.read(address + 1);
  f_data.b[2] = EEPROM.read(address + 2);
  f_data.b[3] = EEPROM.read(address + 3);
  return f_data.x;
}
