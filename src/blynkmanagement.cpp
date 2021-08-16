
#include "debugManagement.h"

#define BLYNK_SEND_ATOMIC
#define BLYNK_PRINT if (Debug.isActive(Debug.DEBUG)) Debug


#include <BlynkApiArduino.h>
#include <Blynk/BlynkProtocol.h>
#include <Adapters/BlynkArduinoClient.h>
#include <WiFi.h>



#include "blynkmanagement.h"
#include "inverterManagement.h"
#include "mqttmanagement.h"
#include "chargeControllerManagement.h"
#include "ssrManagement.h"
#include "bmsManagement.h"
#include "excessControlManagement.h"

char blynkTokenValue[BLYNK_STRLEN] = "";
char blynkServerValue[BLYNK_STRLEN] = BLYNK_DEFAULT_DOMAIN;
char blynkPortValue[NUMBER_LEN] = "80";
static long lastReconnectAttempt = 0;
static long blynkUpdateInterval = 5000;
static SimpleTimer blynkUpdateTimer;


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

void blynkUpdateBattery()
{

if(!gBmsBasicInfo || gBmsDisconnect) {
    return;
}


Blynk.virtualWrite(BLYNK_VPIN_TOTALVOLTAGE,(float)gBmsBasicInfo->getTotalVoltage() / 100.0);
Blynk.virtualWrite(BLYNK_VPIN_CURRENT, (float)gBmsBasicInfo->getcurrent() / 100.0);
Blynk.virtualWrite(BLYNK_VPIN_CAPACITYREMAIN, (float)gBmsBasicInfo->getcapacityRemain() / 100);
Blynk.virtualWrite(BLYNK_VPIN_NOMINALCAPACITY,(float)gBmsBasicInfo->getnominalCapacity() /100);
Blynk.virtualWrite(BLYNK_VPIN_CYCLELIFE,gBmsBasicInfo->getcycleLife());
Blynk.virtualWrite(BLYNK_VPIN_STATEOFCHARGE, gBmsBasicInfo->getstateOfCharge());
Blynk.virtualWrite(BLYNK_VPIN_FETSTATUSCHARGE, (gBmsBasicInfo->getfetControlStatus() & 0x01)*255);
Blynk.virtualWrite(BLYNK_VPIN_FETSTATUSDISCHARGE, ((gBmsBasicInfo->getfetControlStatus() >> 1) & 0x01)*255);
if(gBmsBasicInfo->getnumTempSensors() > 0) {
    Blynk.virtualWrite( BLYNK_VPIN_TEMP_1, (float)gBmsBasicInfo->getTemp(0) / 10);
}

if(gBmsBasicInfo->getnumTempSensors() > 1) {
    Blynk.virtualWrite( BLYNK_VPIN_TEMP_2, (float)gBmsBasicInfo->getTemp(1) / 10);
}

uint16_t status = gBmsBasicInfo->getprotectionStatus();
#define B(NUM) ((status>>NUM) & 1)

Blynk.virtualWrite(BLYNK_VPIN_OVCELL,B(0)*255);
Blynk.virtualWrite(BLYNK_VPIN_UVCELL,B(1)*255);
Blynk.virtualWrite(BLYNK_VPIN_OVBATTERY,B(2)*255);
Blynk.virtualWrite(BLYNK_VPIN_UVBATTERY,B(3)*255);
Blynk.virtualWrite(BLYNK_VPIN_OTCHARGE,B(4)*255);
Blynk.virtualWrite(BLYNK_VPIN_OTDISCHARGE,B(6)*255);
Blynk.virtualWrite(BLYNK_VPIN_OCCHARGE,B(8)*255);
Blynk.virtualWrite(BLYNK_VPIN_OCDISCHARGE,B(9)*255);

#undef B

Blynk.virtualWrite(BLYNK_VPIN_BLE_CONNECTED,!gBmsDisconnect);
if(!gBmsDisconnect) {
    Blynk.setProperty(BLYNK_VPIN_BLE_CONNECTED,"color",BLYNK_GREEN);
} else {
    Blynk.setProperty(BLYNK_VPIN_BLE_CONNECTED,"color",BLYNK_RED);
}

static uint16_t oldBalancingStatus = 0;
uint16_t balanceStatus = gBmsBasicInfo->getbalanceStatusLow();
//BALANCE_STATUS(VAL,CELL) (((VAL)>>(CELL)) & 0x1)
if(gBmsCellInfo) {
    uint16_t minV=UINT16_MAX;
    uint16_t maxV = 0;
    for(uint i = 0;i<gBmsCellInfo->getNumOfCells();++i) {
        maxV = max(maxV,gBmsCellInfo->getCellVolt(i));
        minV = min(minV,gBmsCellInfo->getCellVolt(i));
        Blynk.virtualWrite( BLYNK_VPIN_CELL_VOLTAGE(i), (float)gBmsCellInfo->getCellVolt(i)/1000.0f);
        if(BALANCE_STATUS(oldBalancingStatus,i) != BALANCE_STATUS(balanceStatus,i)) {
            char txt[8];
            if(BALANCE_STATUS(balanceStatus,i)) {
                sprintf(txt,"%d »",i+1);
                Blynk.setProperty(BLYNK_VPIN_CELL_VOLTAGE(i),"color",BLYNK_YELLOW);
            } else {
                sprintf(txt,"%d",i+1);
                Blynk.setProperty(BLYNK_VPIN_CELL_VOLTAGE(i),"color",BLYNK_GREEN);
            }
            Blynk.setProperty(BLYNK_VPIN_CELL_VOLTAGE(i),"label",txt);
            
        }
    }
    Blynk.virtualWrite(BLYNK_VPIN_CELLDIFF,(float)(maxV-minV) / 1000.0f);
}
oldBalancingStatus = balanceStatus;

}

void blynkSetup()
{
    if (strlen(blynkTokenValue) != 32)
    {
        blynkTokenValue[0] = 0;
    }
    else
    {
        Blynk.config(blynkTokenValue, blynkServerValue, atoi(blynkPortValue));
    }


    if (blynkUpdateTimer.setInterval(blynkUpdateInterval >> 1, blynkUpdateInverter) < 0)
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
        if (blynkUpdateTimer.setInterval(blynkUpdateInterval / NUM_CHARGERS, blynkUpdateChargeController) < 0)
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
DBG_SECT(
    Blynk.virtualWrite(BLYNK_VPIN_MQTT_ENABLE, mqttEnabled() ? 1 : 0);
)
    Blynk.virtualWrite(BLYNK_VPIN_BLE_CONNECTED, !gBmsDisconnect);
    if (!gBmsDisconnect)
    {
        Blynk.setProperty(BLYNK_VPIN_BLE_CONNECTED, "color", BLYNK_GREEN);
    }
    else
    {
        Blynk.setProperty(BLYNK_VPIN_BLE_CONNECTED, "color", BLYNK_RED);
    }
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
        Blynk.connect(0);
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

DBG_SECT(
BLYNK_WRITE(BLYNK_VPIN_MQTT_ENABLE)
{
    Serial.print("BLYNK_VPIN_MQTT_ENABLE changed: ");
    Serial.println(param.asInt());
    if (param.asInt() == 1) 
    {
        mqttEnable();
    }
    else {
        mqttDisable();
    }
}

BLYNK_WRITE(BLYNK_VPIN_ALL_LEGS)
{
    gGridSumValues[ValuePower] = param.asFloat();
    gInverterGridPowerUpdated();    
}

)