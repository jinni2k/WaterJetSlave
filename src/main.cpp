
/*
   //##############################################################################
   //Pump 측 DI/O mapping   Slave
   01, DI Hyd' Motor Overload       sDI_HydMotorOVR         sdi01      MbReg[10.0]
   02, DI Hyd' Motor Run Status     sDI_HydMotorStatus      sdi02      MbReg[10.1]
   03, DI Control Power Ready       sDI_CtrlPowerReady      sdi03      MbReg[10.2]
   04  DI     Supply Pump Fault     sDI_SupplyPumpFault          sdi04      MbReg[10.3]
   05, DI Oil Level Low             sDI_OilLevelLow         sdi05      MbReg[10.4]
   06, DI Oil Temp LOW              sDI_OilTempLow          sdi06      MbReg[10.5]
   07, DI Supply Press' Low         sDI_SupplyPressLow      sdi07      MbReg[10.6]
   --삭제-- 8,DI Emergency Stop     sDI_EmergencyStop       sdi08      MbReg[10.7]

   09, DI Int' Left Sensor          sDI_IntLeftSensor       sdi09      MbReg[10.8]
   10, DI Int  Right Sensor         sDI_IntRightSensor      sdi10      MbReg[10.9]
   11, DI Oil Temp High             sDI_OilTempHigh         sdi11      MbReg[10.A]
   12, DI Supply Pump Overload      sDI_SupplyMotorOL       sdi12      MbReg[10.B]
   13, DI Local mode                sDI_LocalMode           sdi13      MbReg[10.C]
   14, Supply Pump Run Command      sDI_SupplyPumpRunC      sdi14      MbReg[10.D]
   15, Hyd' Motor Run Command       sDI_HydMotorRunC        sdi15      MbReg[10.E]
   16, Pressure High Command        sDI_PressHighC          sdi16      MbReg[10.F]

   Extension DI
   Not Connected

   Slave Relay Output
   01, DO Hyd' Motor Start com.     sDO_HydMotorStart       sdo01      MbReg[14.0]
   02, DO Emg' Stop                 sDO_EmgStop             sdo02      MbReg[14.1]
   03, DO Int' left Sol.V           sDO_IntleftSol          sdo03      MbReg[14.2]
   04, DO Int' Right Sol.V          sDO_IntRightSol         sdo04      MbReg[14.3]
   05, DO Supply Sol.V              sDO_SupplySol           sdo05      MbReg[14.4]
   06, DO Low Press Sol.V           sDO_LowPresSol          sdo06      MbReg[14.5]
   07, DO High Press Sol.V          sDO_HighPresSol         sdo07      MbReg[14.6]
   08, DO Supply Pump Start Com.    sDO_SupplyPumpStart     sdo08      MbReg[14.7]

   09, DO Hyd' YD Timer             sDO_HydYDTimer          sdo09      MbReg[14.8]
   10, DO Bleed down V              sDO_BleedDownV          sdo10      MbReg[14.9]
   11, DO Servo Fault Lamp          sDO_ServoFaultL         sdo11      MbReg[14.A]
   12, DO Pump Fault  Lamp          sDO_PumpFaultL          sdo12      MbReg[14.B]
   13, DO HydMotor Fault Lamp       sDO_HydMotorFaultL      sdo13      MbReg[14.C]
   14, DO Pump Run Lamp             sDO_PumpRunL            sdo14      MbReg[14.D]
   15, DO Hyd' MotorRun Lamp        sDO_HydMotorRunL        sdo15      MbReg[14.E]
   16, DO Slave Com' Fault          sDO_ComFault            sdo16      MbReg[14.F]
 */
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <TaskScheduler.h>
#include <pcf8574_esp.h>
#include <advancedSerial.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <Ticker.h>
#include <SoftwareSerial.h>
#include <RemoteDebug.h>
#include "modbus.h"

#define  TXEN D7        //D7: DE     422 Comm' TX enable
#define  _RXEN D8       //D8: _RE    422 Comm' RX enable low Active
#define HOST_NAME "JWCS-Slave;'"

void controlLoop();
void communications();
void display();
void comCheck();
void chk_key();
Task t0(100,   TASK_FOREVER, &controlLoop);

Scheduler runner;

RemoteDebug Debug;
SoftwareSerial swSerial(D5,D6, false, 1024);  //RX, TX
const char* host = "esp8266";
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;
const char* OTAssid = "jujunet";
const char* OTApassword = "12345678";

int initial = 0,pumpStartT1=0,pumpStartT2=0;
void otaSetup();
unsigned int controlScan = 0,ocontrolScan,octlS[2],hydMotorTime;
bool mDI_EmergencyStop,mDI_PumpStart,mDI_HydMotorStart,mDI_NozzleOn,mDI_AbrasiveIn,mDI_AbrasiveLow,mDI_ServoFault;
bool mDO_CtrlPowerReady,mDO_HydMotorOVR,mDO_HydMotorStatus,mDO_SupplyPumpFault,mDO_ComFault;
bool sDI_HydMotorOVR,sDI_HydMotorStatus,sDI_CtrlPowerReady,sDI_SupplyPumpFault,sDI_OilLevelLow,sDI_OilTempLow,sDI_SupplyPressLow,sDI_EmergencyStop;
bool sDI_IntLeftSensor,sDI_IntRightSensor,sDI_OilTempHigh,sDI_SupplyMotorOL,sDI_LocalMode,sDI_SupplyPumpRunC,sDI_HydMotorRunC,sDI_PressHighC;
bool sDO_HydMotorStart,sDO_EmgStop,sDO_IntleftSol,sDO_IntRightSol,sDO_SupplySol,sDO_LowPresSol,sDO_HighPresSol,sDO_SupplyPumpStart;
bool sDO_HydYDTimer,sDO_BleedDownV,sDO_ServoFaultL,sDO_PumpFaultL,sDO_HydMotorFaultL,sDO_PumpRunL,sDO_HydMotorRunL,sDO_ComFault;

volatile bool PCFInterruptFlag = false;
PCF857x DI8_0(0x20, &Wire);  //DI00 ~ DI07      DI
PCF857x DI8_1(0x21, &Wire);  //DI08 ~ DI15      DI
PCF857x RO8_0(0x22, &Wire);  //DO00 ~ DO07      DO
PCF857x FaceBt(0x23, &Wire); //Face Plate Button Input
PCF857x DI8_2(0x24, &Wire);  //DI16 ~ DI23      DI
PCF857x DI8_3(0x25, &Wire);  //DI24 ~ DI31      DI
PCF857x RO8_1(0x26, &Wire);  //DO08 ~ DO15      DO

uint8_t DI[4],cntScan;

void communications() {
        cntScan++;
        if(cntScan> 2) cntScan =0;
        DI[0] = ~DI8_0.read8();
        DI[1] = ~DI8_1.read8();
        DI[2] = ~DI8_2.read8();
        DI[3] = ~DI8_3.read8();

        MbReg[0] = word(DI[1],DI[0]);         //DI00~15  DI
        MbReg[1] = word(DI[3],DI[2]);         //DI16~31  Extension DI

        sDI_HydMotorOVR       = bitRead(MbReg[0],0);
        sDI_HydMotorStatus    = bitRead(MbReg[0],1);
        sDI_CtrlPowerReady    = bitRead(MbReg[0],2);
        sDI_SupplyPumpFault   = bitRead(MbReg[0],3);
        sDI_OilLevelLow       = bitRead(MbReg[0],4);
        sDI_OilTempLow        = bitRead(MbReg[0],5);
        sDI_SupplyPressLow    = bitRead(MbReg[0],6);
        sDI_EmergencyStop     = bitRead(MbReg[0],7);
        sDI_IntLeftSensor     = bitRead(MbReg[0],8);
        sDI_IntRightSensor    = bitRead(MbReg[0],9);
        sDI_OilTempHigh       = bitRead(MbReg[0],10);
        sDI_SupplyMotorOL     = bitRead(MbReg[0],11);
        sDI_LocalMode         = bitRead(MbReg[0],12);
        sDI_SupplyPumpRunC    = bitRead(MbReg[0],13);
        sDI_HydMotorRunC      = bitRead(MbReg[0],14);
        sDI_PressHighC        = bitRead(MbReg[0],15);

        //slave Relay Output
        bitWrite(MbReg[4],0,sDO_HydMotorStart);
        bitWrite(MbReg[4],1,sDO_EmgStop);
        bitWrite(MbReg[4],2,sDO_IntleftSol);
        bitWrite(MbReg[4],3,sDO_IntRightSol);
        bitWrite(MbReg[4],4,sDO_SupplySol);
        bitWrite(MbReg[4],5,sDO_LowPresSol);
        bitWrite(MbReg[4],6,sDO_HighPresSol);
        bitWrite(MbReg[4],7,sDO_SupplyPumpStart);

        bitWrite(MbReg[4],8,sDO_HydYDTimer);
        bitWrite(MbReg[4],9,sDO_BleedDownV);
        bitWrite(MbReg[4],10,sDO_ServoFaultL);
        bitWrite(MbReg[4],11,sDO_PumpFaultL);
        bitWrite(MbReg[4],12,sDO_HydMotorFaultL);
        bitWrite(MbReg[4],13,sDO_PumpRunL);
        bitWrite(MbReg[4],14,sDO_HydMotorRunL);
        bitWrite(MbReg[4],15,sDO_ComFault);

        //Master Relay Output
        bitWrite(MbReg[5],0,mDO_HydMotorOVR);
        bitWrite(MbReg[5],1,mDO_HydMotorStatus);
        bitWrite(MbReg[5],2,mDO_CtrlPowerReady);
        bitWrite(MbReg[5],3,mDO_SupplyPumpFault);

        //MbReg[24] =10;
        //if(cntScan==0)MBMaster(0,234,23,10,5,10,0,5,0);
        //if(cntScan==0)MBMaster(0,234,6,24,1,24);
        MBMaster(0,234,23,10,5,10,0,6,0);
        //else if(cntScan==1)MBMaster(0,234,6,24,1,24);

        //else if(cntScan==1)MBMaster(0,234,23,10,5,10,10,5,10);
        //if(cntScan==0) MBMaster(0,234,23,10,5,10,0,5,0);
        //else if(cntScan==1) MBMaster(0,234,6,14,1,14);
        //if(cntScan==2) MBMaster(0,234,6,4,1,4);
        mDI_EmergencyStop     =  !bitRead(MbReg[10],0);
        mDI_PumpStart         =  bitRead(MbReg[10],1);
        mDI_HydMotorStart     =  bitRead(MbReg[10],2);

        mDI_NozzleOn          =  bitRead(MbReg[10],4);
        mDI_AbrasiveIn        =  bitRead(MbReg[10],5);
        mDI_AbrasiveLow       =  bitRead(MbReg[10],6);
        mDI_ServoFault        =  bitRead(MbReg[10],7);

        for(int i =0; i<8; i++) RO8_0.write(i,bitRead(~MbReg[4],i));
        for(int i =0; i<8; i++) RO8_1.write(i,bitRead(~MbReg[4],i+8));
        //RO8_1.write8(~highByte(MbReg[4]));
}

void comCheck(){
        if(octlS[0] == MbReg[13]) sDO_ComFault = 1;
        else sDO_ComFault = 0;
        rdebugVln("comScan : %u, %u",MbReg[13],octlS[0]);
        octlS[0] = MbReg[13];
}

void controlLoop() {
        wdt_reset();
        controlScan++;
        if(controlScan>10000) controlScan = 0;
        MbReg[3] =controlScan;

        communications();
        if(controlScan%10==0) comCheck();
        //Prepare Start
        if(sDI_CtrlPowerReady) sDO_BleedDownV =1;
        else sDO_BleedDownV = 0;

        ///////////// Master Remote Supply Pump Start
        if(sDI_CtrlPowerReady && !sDI_LocalMode && !mDI_EmergencyStop && mDI_PumpStart && !sDI_SupplyMotorOL) {
                if(pumpStartT1 <100) pumpStartT1++;
                sDO_SupplySol = 1;
                if(pumpStartT1>20) sDO_SupplyPumpStart = 1;
        }
        else if(sDI_CtrlPowerReady && sDI_LocalMode && !sDI_SupplyMotorOL && sDI_SupplyPumpRunC) {
                if(pumpStartT2 <100) pumpStartT2++;
                sDO_SupplySol = 1;
                if(pumpStartT2>20) sDO_SupplyPumpStart = 1;
        }
        else{
                sDO_SupplySol = 0;
                sDO_SupplyPumpStart = 0;
                pumpStartT1 =0;
                pumpStartT2 =0;
        }

        ////////// local low prssure sol high sol
        if(sDI_CtrlPowerReady && sDI_LocalMode && sDI_PressHighC) {
                sDO_LowPresSol = 0;
                sDO_HighPresSol = 1;
        }
        else if(sDI_CtrlPowerReady && sDI_LocalMode && !sDI_PressHighC) {
                sDO_LowPresSol = 1;
                sDO_HighPresSol = 0;
        }

        ///////////// Master Hyd' Motor Start
        if(sDI_CtrlPowerReady && !sDI_LocalMode && !sDI_EmergencyStop && !sDI_HydMotorOVR && mDI_HydMotorStart) sDO_HydMotorStart = 1;
        //if(!sDI_LocalMode && !mDI_EmergencyStop && mDI_HydMotorStart) sDO_HydMotorStart = 1;
        else if(sDI_CtrlPowerReady && sDI_LocalMode && !sDI_HydMotorOVR && sDI_HydMotorRunC) sDO_HydMotorStart = 1;
        else sDO_HydMotorStart = 0;

        ///////// Hyd' Motor  Seq'
        if(sDO_HydMotorStart) {
                sDO_HydYDTimer = 1;
                if(!sDI_LocalMode) {
                        if(hydMotorTime <100 && sDO_SupplyPumpStart) hydMotorTime++;
                        if(hydMotorTime < 30 && sDO_SupplyPumpStart) {
                                sDO_LowPresSol =1;
                                sDO_HighPresSol = 0;
                        }
                        else if(hydMotorTime >=30 && sDO_SupplyPumpStart) {
                                sDO_LowPresSol = 0;
                                sDO_HighPresSol = 1;
                        }
                        if(!sDO_SupplyPumpStart) {
                                hydMotorTime = 0;
                                sDO_LowPresSol = 0;
                                sDO_HighPresSol = 0;
                        }
                }
                if (sDO_LowPresSol || sDO_HighPresSol ) {
                        if(sDI_IntLeftSensor && !sDO_IntleftSol) {
                                sDO_IntRightSol =0;
                                sDO_IntleftSol =1;
                        }
                        else if(sDI_IntRightSensor && !sDO_IntRightSol) {
                                sDO_IntleftSol =0;
                                sDO_IntRightSol =1;
                        }
                }
                else {
                        sDO_IntleftSol =0;
                        sDO_IntRightSol =0;
                }
                //if(sDO_HighPresSol && hydMotorTime >30 && !sDO_SupplySol)sDO_SupplySol =1;
                //if(sDO_SupplySol && !sDI_SupplyMotorOL && !sDO_SupplyPumpStart)sDO_SupplyPumpStart =1;
        }
        else if(!sDO_HydMotorStart  ){
                hydMotorTime    = 0;
                sDO_HydYDTimer  = 0;
                sDO_IntRightSol = 0;
                sDO_IntleftSol  = 0;
                sDO_LowPresSol  = 0;
                sDO_HighPresSol = 0;
        }
        else if(!sDO_SupplyPumpStart){
            pumpStartT1 =0;
            pumpStartT2 =0;
            sDO_IntRightSol = 0;
            sDO_IntleftSol  = 0;
            sDO_LowPresSol  = 0;
            sDO_HighPresSol = 0;
        }

        if(mDI_ServoFault) sDO_ServoFaultL = 1;
        else sDO_ServoFaultL = 0;

        if(sDI_SupplyMotorOL) sDO_PumpFaultL =1;
        else sDO_PumpFaultL = 0;

        if(sDI_HydMotorOVR) {
                sDO_HydMotorFaultL =1;
                mDO_HydMotorOVR =1;
        }
        else if(!sDI_CtrlPowerReady || sDI_SupplyPumpFault) mDO_HydMotorOVR = 1;
        else{
                sDO_HydMotorFaultL = 0;
                mDO_HydMotorOVR =0;
        }

        if(sDI_HydMotorStatus) mDO_HydMotorStatus =1;
        else mDO_HydMotorStatus = 0;

        if(sDI_CtrlPowerReady) mDO_CtrlPowerReady =1;
        else mDO_CtrlPowerReady = 0;
       if(sDO_SupplyPumpStart)sDO_PumpRunL =1;
       else sDO_PumpRunL =0;
       if(sDI_HydMotorStatus)sDO_HydMotorRunL =1;
       else sDO_HydMotorRunL =0;

       if(mDI_EmergencyStop)sDO_EmgStop =1;
       else sDO_EmgStop =0;

        rdebugVln("mDI_EmergencyStop : %u",mDI_EmergencyStop);
        rdebugVln("mDI_PumpStart : %u",mDI_PumpStart);
}

void setup() {
        ESP.wdtEnable(2000);
        Serial.begin(19200, SERIAL_8E1);
        runner.init();
        runner.addTask(t0);
        t0.enable();

        DI8_0.begin();
        DI8_1.begin();
        DI8_2.begin();
        DI8_3.begin();
        RO8_0.begin();
        RO8_1.begin();
        FaceBt.begin();

        pinMode(TXEN,OUTPUT);
        pinMode(_RXEN,OUTPUT);
        digitalWrite(TXEN, HIGH);
        digitalWrite(_RXEN, LOW);
        Wire.begin();
        Wire.setClock(100000L);
        WiFi.mode(WIFI_STA);
        WiFi.begin(OTAssid, OTApassword);
        otaSetup();
        String hostNameWifi = HOST_NAME;
        hostNameWifi.concat(".local");
        WiFi.hostname(hostNameWifi);
        MDNS.addService("telnet", "tcp", 23);
        Debug.begin(HOST_NAME); // Initiaze the telnet server
        Debug.setResetCmdEnabled(true); // Enable the reset command
}

void loop() {
        wdt_reset();
        runner.execute();
        ArduinoOTA.handle();
        httpServer.handleClient();
        Debug.handle();
}

void otaSetup() {
        // Port defaults to 8266
        // ArduinoOTA.setPort(8266);
        // Hostname defaults to esp8266-[ChipID]
        // ArduinoOTA.setHostname("myesp8266");
        // No authentication by default
        // ArduinoOTA.setPassword((const char *)"123");
        ArduinoOTA.onStart([]() {
                Serial.println("Start");
        });
        ArduinoOTA.onEnd([]() {
                Serial.println("\nEnd");
        });
        //ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        //  Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        //});
        ArduinoOTA.onError([](ota_error_t error) {
                Serial.printf("Error[%u]: ", error);
                if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
                else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
                else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
                else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
                else if (error == OTA_END_ERROR) Serial.println("End Failed");
        });
        ArduinoOTA.begin();
        Serial.println("Ready - OTA Success!!!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        MDNS.begin(host);
        httpUpdater.setup(&httpServer);
        httpServer.begin();
        MDNS.addService("http", "tcp", 80);
        String ipaddress = WiFi.localIP().toString();
        String chipID = String(ESP.getChipId(), HEX);
        char charChipID[10];
        chipID.toCharArray(charChipID, sizeof(charChipID));
        char charipaddress[16];
        ipaddress.toCharArray(charipaddress, sizeof(charipaddress));
        Serial.printf("Now open http://%s.local/update in your browser or \n", host);
        Serial.printf("http://%s/update or http://%s.lan/update if you prefer.\n", charipaddress, charChipID);
}
