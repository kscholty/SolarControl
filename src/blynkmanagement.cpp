
#define BLYNK_SEND_ATOMIC

#if DEBUG
#define BLYNK_PRINT Serial
#endif 

#include <BlynkApiArduino.h>
#include <Blynk/BlynkProtocol.h>
#include <Adapters/BlynkArduinoClient.h>
#include <WiFi.h>

#include "blynkmanagement.h"
#include "inverterManagement.h"
#include "mqttmanagement.h"
#include "chargeControllerManagement.h"

char blynkTokenValue[BLYNK_STRLEN] = "";
char blynkServerValue[BLYNK_STRLEN] = BLYNK_DEFAULT_DOMAIN;
char blynkPortValue[BLYNK_STRLEN] = "80";
static long lastReconnectAttempt = 0;
static long blynkUpdateInterval = 20000;
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
#if DEBUG
    Serial.println("Updating grid data on Blynk");
#endif
    Blynk.virtualWrite(BLYNK_VPIN_LEG_0, gGridLegsPower[0]);
    Blynk.virtualWrite(BLYNK_VPIN_LEG_1, gGridLegsPower[1]);
    Blynk.virtualWrite(BLYNK_VPIN_LEG_2, gGridLegsPower[2]);
    Blynk.virtualWrite(BLYNK_VPIN_ALL_LEGS, gGridSumPower);
}

void blynkUpdateChargeController() {
    static int index = 0;

    if (gChargerValuesChanged[index])
    {
#if DEBUG
        Serial.print("Updating charge controller ");
        Serial.print(index + 1);
        Serial.println(" data on Blynk");
#endif
        gChargerValuesChanged[index] = false;
        for (unsigned int i = 0; i < NUM_CHARGER_VALUES; ++i)
        {
            Blynk.virtualWrite(BLYNK_CHARGER_PIN(index, i), chargerValues[index][i] / 100.0f);
        }
        do
        {
            index = (index + 1) % NUM_CHARGERS;
        } while (!chargerIsValid(index));
    }
}

void blynkSetup()
{
    if(strlen(blynkTokenValue) != 32) {
        blynkTokenValue[0]= 0;
    } else {
        Blynk.config(blynkTokenValue, blynkServerValue, atoi(blynkPortValue));
    }

    if (blynkUpdateTimer.setInterval(blynkUpdateInterval, blynkUpdateGrid) <0 ) {
        Serial.println("Cannot create blynk grid update timer");
    }

    if (gChargerNumValidChargers > 0)
        if (blynkUpdateTimer.setInterval(2500, blynkUpdateChargeController) < 0)
        {
            Serial.println("Cannot create blynk charge controller 1 timer");
        }
    
    blynkUpdateTimer.disableAll();
}


bool isValid() {
    return blynkTokenValue[0] != 0 && WiFi.isConnected();
}

BLYNK_CONNECTED() {
#if DEBUG
    Serial.println("Blynk connected");
    Blynk.virtualWrite(BLYNK_VPIN_MQTT_ENABLE, mqttEnabled() ? 1 : 0);
#endif
    blynkUpdateTimer.enableAll();
}

BLYNK_DISCONNECTED()
{
#if DEBUG
    Serial.println("Blynk disconnected");
#endif
    blynkUpdateTimer.disableAll();
}

void blynkReconnect()
{
    if(isValid()) {
#if DEBUG
        Serial.println("Starting Blynk connect");
#endif
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
    Blynk.virtualWrite(BLYNK_VPIN_LEG_0, gGridLegsPower[0]);
}

BLYNK_READ(BLYNK_VPIN_LEG_1)
{
    Blynk.virtualWrite(BLYNK_VPIN_LEG_1, gGridLegsPower[1]);
}

BLYNK_READ(BLYNK_VPIN_LEG_2)
{
    Blynk.virtualWrite(BLYNK_VPIN_LEG_2, gGridLegsPower[2]);
}

BLYNK_READ(BLYNK_VPIN_ALL_LEGS)
{
    Blynk.virtualWrite(BLYNK_VPIN_ALL_LEGS, gGridSumPower);
}

#if DEBUG
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
    gGridSumPower = param.asInt();
    xTaskNotifyGive(gInverterTaskHandle);
}

#endif