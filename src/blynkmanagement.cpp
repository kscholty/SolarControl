
#include "debugManagement.h"

#define BLYNK_SEND_ATOMIC
#define BLYNK_PRINT if (Debug.isActive(Debug.DEBUG)) Debug


#include <BlynkApiArduino.h>
#include <Blynk/BlynkProtocol.h>
#include <Adapters/BlynkArduinoClient.h>
#include <WiFi.h>



#include "blynkmanagement.h"
#include "inverterManagement.h"
#include "chargeControllerManagement.h"
#include "bmsManagement.h"
#include "excessControlManagement.h"

char blynkTokenValue[BLYNK_STRLEN] = "";
char blynkServerValue[BLYNK_STRLEN] = BLYNK_DEFAULT_DOMAIN;
char blynkPortValue[NUMBER_LEN] = "80";
static long lastReconnectAttempt = 0;
static long blynkUpdateInterval = 5000;
static SimpleTimer blynkUpdateTimer;
static unsigned int actBms = 1;
static bool bmsChanged = true;

class BlynkWifi
    : public BlynkProtocol<BlynkArduinoClient>
{
    typedef BlynkProtocol<BlynkArduinoClient> Base;

public:
    BlynkWifi(BlynkArduinoClient &transp)
        : Base(transp)
    {
    }

    
    void config(const char *auth,
                const char *domain = BLYNK_DEFAULT_DOMAIN,
                uint16_t port = BLYNK_DEFAULT_PORT)
    {
        Base::begin(auth);
        this->conn.begin(domain, port);
    }

    void config(const char *auth,
                IPAddress ip,
                uint16_t port = BLYNK_DEFAULT_PORT)
    {
        Base::begin(auth);
        this->conn.begin(ip, port);
    }
};

static WiFiClient _blynkWifiClient;
static BlynkArduinoClient _blynkTransport(_blynkWifiClient);
BlynkWifi Blynk(_blynkTransport);


//#include <BlynkWidgets.h>

void blynkUpdateGrid()
{
DBG_SECT(
    DEBUG_V("Updating grid data on Blynk");
)
    Blynk.virtualWrite(BLYNK_VPIN_LEG_0, gGridLegValues[ValuePower][0]);
    Blynk.virtualWrite(BLYNK_VPIN_LEG_1, gGridLegValues[ValuePower][1]);
    Blynk.virtualWrite(BLYNK_VPIN_LEG_2, gGridLegValues[ValuePower][2]);
    Blynk.virtualWrite(BLYNK_VPIN_ALL_LEGS, gGridSumValues[ValuePower]);
}

void sendStatus(unsigned int pin, ChargerValues_t type, unsigned int value)
{
    char const *text = "Unknown";
    switch (type)
    {
    case BATTERY_STATUS:
        
        switch(value & 0x0F) {
            case 00: text = "Normal"; break;
            case 01: text = "Overvolt"; break;
            case 02: text = "Undervolt"; break;
            case 03: text = "Low Volt Disconnect"; break;
            case 04: text = "Fault"; break;
        }
        Blynk.virtualWrite(pin, text);        
        break;

    case CONTROLLER_STATUS:
        switch( (value&0x0C)>>2)
        {
            case 0: text = "No charging"; break;
            case 1: text = "Float charge"; break;
            case 2: text = "Boost charge"; break;
            case 3: text = "Equal. charge"; break;
        }
        Blynk.virtualWrite(pin, text); 
        break;
        
    default:
        Blynk.virtualWrite(pin, (float)value / 100.0f);
        break;
    }
}

void blynkUpdateChargeController()
{
    char text[30]; 
    for (int index = 0; index < NUM_CHARGERS; ++index)
    {

        if (gChargerValuesChanged[index])
        {

            DEBUG_V("Updating charge controller %d data on Blynk",index+1);            

            gChargerValuesChanged[index] = false;
            for (unsigned int i = 0; i < NUM_CHARGER_VALUES; ++i)
            {
                sendStatus(BLYNK_CHARGER_PIN(index, i), (ChargerValues_t)i, chargerValues[index][i]);
            }
        }
        unsigned int value = chargerValues[index][CHARGER_READ_ERROR];
        sprintf(text,"PV:%d T:%d S:%d",value >> 2*8,(value>>1*8)& 0xFF,value & 0xFF);        
        Blynk.virtualWrite(BLYNK_CHARGER_1_ERROR+index,text);
    }

    Blynk.virtualWrite(V37, gCurrentPowerCreated);
}

void blynkUpdateInverter()
{
     Blynk.virtualWrite(BLYNK_VPIN_INVERTER_POWER,gInverterPower);
     Blynk.virtualWrite(BLYNK_VPIN_INVERTER_CURRENT,gInverterCurrent);
     Blynk.virtualWrite(BLYNK_VPIN_INVERTER_VOLTAGE,gInverterVoltage);
     Blynk.virtualWrite(BLYNK_VPIN_INVERTER_POWER_FACTOR,gInverterPowerFactor);
     Blynk.virtualWrite(BLYNK_VPIN_INVERTER_TARGET,gInverterTarget);  
     Blynk.virtualWrite(BLYNK_VPIN_INVERTER_EXCESS,gExcessTarget);    

    // Update this one as well to make it look a little better in the app.
     Blynk.virtualWrite(BLYNK_VPIN_ALL_LEGS, gGridSumValues[ValuePower]);   
}

void blynkUpdateBattery() {
  
  static uint16_t oldBalancingStatus = 0xFFFF;

  int actPin = 0;
  BMS::CBmsBase *aBms = gBms[actBms - 1];
  
  

  if (!aBms || ( !gBmsUpdated && !bmsChanged)) {
    return;
  }

  Blynk.virtualWrite(BLYNK_VPIN_BLE_CONNECTED, !gBmsDisconnect);
  if (!gBmsDisconnect) {
    Blynk.setProperty(BLYNK_VPIN_BLE_CONNECTED, "color", BLYNK_GREEN);
  } else {
    Blynk.setProperty(BLYNK_VPIN_BLE_CONNECTED, "color", BLYNK_RED);
  }

  gBmsUpdated = false;

  actPin = FIRST_BATTERY_VPIN;
  const BMS::BmsBasicInfo_t &bmsBasicInfo = aBms->basicInfo();
  Blynk.virtualWrite(actPin++, (float)bmsBasicInfo.getTotalVoltage() / 100.0);
  Blynk.virtualWrite(actPin++, (float)bmsBasicInfo.getcurrent() / 100.0);
  Blynk.virtualWrite(actPin++, (float)bmsBasicInfo.getcapacityRemain() / 100);
  Blynk.virtualWrite(actPin++, (float)bmsBasicInfo.getnominalCapacity() / 100);
  Blynk.virtualWrite(actPin++, bmsBasicInfo.getcycleLife());
  Blynk.virtualWrite(actPin++, bmsBasicInfo.getstateOfCharge());
  Blynk.virtualWrite(actPin++,
                     (bmsBasicInfo.getfetControlStatus() & 0x01) * 255);
  Blynk.virtualWrite(actPin++,
                     ((bmsBasicInfo.getfetControlStatus() >> 1) & 0x01) * 255);
  if (bmsBasicInfo.getnumTempSensors() > 0) {
    Blynk.virtualWrite(actPin++, (float)bmsBasicInfo.getTemp(0) / 10);
  }

  if (bmsBasicInfo.getnumTempSensors() > 1) {
    Blynk.virtualWrite(actPin++, (float)bmsBasicInfo.getTemp(1) / 10);
  }

  const BMS::Alarms_t status = bmsBasicInfo.getprotectionStatus();
  Blynk.virtualWrite(actPin++, status.CellOvervoltage * 255);
  Blynk.virtualWrite(actPin++, status.CellUndervoltage * 255);
  Blynk.virtualWrite(actPin++, status.ChargingOvervoltage * 255);
  Blynk.virtualWrite(actPin++, status.DischargingUndervoltage * 255);
  Blynk.virtualWrite(actPin++, status.BmsOverTemperature * 255);
  Blynk.virtualWrite(actPin++, status.BmsOverTemperature * 255);
  Blynk.virtualWrite(actPin++, status.ChargingOvercurrent * 255);
  Blynk.virtualWrite(actPin++, status.DischargingOvercurrent * 255);

  uint16_t balanceStatus = bmsBasicInfo.getbalanceStatusLow();

  // BALANCE_STATUS(VAL,CELL) (((VAL)>>(CELL)) & 0x1)
  if (aBms && ( !gBmsDisconnect || bmsChanged)) {
    uint16_t minV = UINT16_MAX;
    uint16_t maxV = 0;
    const BMS::BmsCellInfo_t &bmsCellInfo = aBms->cellInfo();
    for (uint cell = 0; cell < bmsCellInfo.getNumOfCells(); ++cell) {
      maxV = max(maxV, bmsCellInfo.getCellVolt(cell));
      minV = min(minV, bmsCellInfo.getCellVolt(cell));
      Blynk.virtualWrite(FIRST_CELL_VOLTAGE + cell,
                         (float)bmsCellInfo.getCellVolt(cell) / 1000.0f);

      if (BALANCE_STATUS(oldBalancingStatus, cell) !=
          BALANCE_STATUS(balanceStatus, cell)|| bmsChanged) {
        char txt[8];
        if (BALANCE_STATUS(balanceStatus, cell)) {
          sprintf(txt, "%d »", cell + 1);
          Blynk.setProperty(FIRST_CELL_VOLTAGE + cell, "color", BLYNK_YELLOW);
        } else {
          sprintf(txt, "%d", cell + 1);
          Blynk.setProperty(FIRST_CELL_VOLTAGE + (int)cell, "color", BLYNK_GREEN);
        }
        Blynk.setProperty(FIRST_CELL_VOLTAGE + (int)cell, "label", txt);
      }
    }
    if(minV<maxV) {
        Blynk.virtualWrite(BLYNK_VPIN_CELLDIFF, (float)(maxV - minV) / 1000.0f);
    } else {
        Blynk.virtualWrite(BLYNK_VPIN_CELLDIFF, 0.0f);
    }
  }

  oldBalancingStatus = balanceStatus;
  bmsChanged = false;
}

void blynkSetup()
{
    if (strlen(blynkTokenValue) != 32)
    {
        blynkTokenValue[0] = 0;
        Serial.println("Disabling blynk");
    }
    else
    {
        Blynk.config(blynkTokenValue, blynkServerValue, atoi(blynkPortValue));
    }


    if (blynkUpdateTimer.setInterval(max(blynkUpdateInterval >> 1,1000l), blynkUpdateInverter) < 0)
    {
        Serial.println("Cannot create blynk blynkUpdateInverter update timer");
    }

    delay(300); 
    if (blynkUpdateTimer.setInterval(blynkUpdateInterval, blynkUpdateGrid) < 0)
    {
        Serial.println("Cannot create blynk grid update timer");
    }

    delay(300); 
    if (gChargerNumValidChargers > 0)
    {
        if (blynkUpdateTimer.setInterval(max(blynkUpdateInterval / NUM_CHARGERS,1000l), blynkUpdateChargeController) < 0)
        {
            Serial.println("Cannot create blynk charge controller 1 timer");
        }
    }    

    delay(300);
    if (blynkUpdateTimer.setInterval(blynkUpdateInterval, blynkUpdateBattery) < 0)
    {
        Serial.println("Cannot create blynk blynkUpdateBattery update timer");
    }
    
    blynkUpdateTimer.disableAll();
}

bool isValid() {
    return blynkTokenValue[0] != 0 && WiFi.isConnected();
}

BLYNK_CONNECTED()
{
    Blynk.virtualWrite(BLYNK_VPIN_BLE_CONNECTED, !gBmsDisconnect);
    Blynk.virtualWrite(BLYNK_SEL_BMS, actBms);
    
    Blynk.setProperty(BLYNK_SEL_BMS,"labels", gBmsType[0],gBmsType[1],gBmsType[2]);

    if (!gBmsDisconnect)
    {
        Blynk.setProperty(BLYNK_VPIN_BLE_CONNECTED, "color", BLYNK_GREEN);
    }
    else
    {
        Blynk.setProperty(BLYNK_VPIN_BLE_CONNECTED, "color", BLYNK_RED);
    }

    Blynk.virtualWrite(BLYNK_VPIN_PID_P,Kp);
    Blynk.virtualWrite(BLYNK_VPIN_PID_I,Ki);
    Blynk.virtualWrite(BLYNK_VPIN_PID_D,Kd);

    Serial.println("Blynk connected");
    blynkUpdateTimer.enableAll();
}

BLYNK_DISCONNECTED()
{
    Serial.println("Blynk disconnected");
    blynkUpdateTimer.disableAll();
}

void blynkReconnect()
{
    if (isValid())
    {
        DBG_SECT(
            DEBUG_I("Starting Blynk connect");)
        if(!Blynk.connect(0)) {
            DEBUG_E("Blynk connectioon failed");
        }
    }
}

void blynkLoop(unsigned long now)
{
    if (!isValid())
    {
        return;
    }

    if (!Blynk.connected())
    {
        if (now - lastReconnectAttempt > BLYNK_TIMEOUT_MS * 3)
        {
            lastReconnectAttempt = now;
            // Attempt to reconnect
            blynkReconnect();
        }
    } 
 
    Blynk.run();
    blynkUpdateTimer.run();
}

BLYNK_READ(BLYNK_VPIN_LEG_0)
{
    Blynk.virtualWrite(BLYNK_VPIN_LEG_0, gGridLegValues[ValuePower][0]);
}

BLYNK_READ(BLYNK_VPIN_LEG_1)
{
    Blynk.virtualWrite(BLYNK_VPIN_LEG_1, gGridLegValues[ValuePower][1]);
}

BLYNK_READ(BLYNK_VPIN_LEG_2)
{
    Blynk.virtualWrite(BLYNK_VPIN_LEG_2, gGridLegValues[ValuePower][2]);
}

BLYNK_READ(BLYNK_VPIN_ALL_LEGS)
{
    Blynk.virtualWrite(BLYNK_VPIN_ALL_LEGS, gGridSumValues[ValuePower]);
}

#if NEVER
#include "calibration.h"
BLYNK_WRITE(BLYNK_VPIN_CALIBRATE_ADC)
{
     if (param.asInt() == 1) 
        adcStartCalibration();
}
#endif

BLYNK_WRITE(V58)
{
    bmsEnable(param.asInt() == 1);
    if (!gBmsDisconnect)
    {
        Blynk.setProperty(BLYNK_VPIN_BLE_CONNECTED, "color", BLYNK_GREEN);
    }
    else
    {
        Blynk.setProperty(BLYNK_VPIN_BLE_CONNECTED, "color", BLYNK_RED);
    }
}

BLYNK_WRITE(BLYNK_SEL_BMS) {
    actBms = param.asInt();
    if(actBms > bmsCount) {
        actBms = 0;
    }
    gBmsUpdated = bmsChanged = true;
    blynkUpdateBattery();
}

DBG_SECT(

BLYNK_WRITE(BLYNK_VPIN_PID_P) {
    Kp = param.asDouble();
}

BLYNK_WRITE(BLYNK_VPIN_PID_I) {
    Ki = param.asDouble();
}

BLYNK_WRITE(BLYNK_VPIN_PID_D) {
    Kd = param.asDouble();
}

BLYNK_WRITE(BLYNK_VPIN_PID_OK) {
    if(param.asInt() == 1) {
        inverterActivatePidValues();
    }
}


BLYNK_WRITE(BLYNK_VPIN_ALL_LEGS)
{
    gGridSumValues[ValuePower] = param.asFloat();
    gInverterGridPowerUpdated();    
}

)