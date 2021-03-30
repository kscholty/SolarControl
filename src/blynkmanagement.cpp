#include "blynkmanagement.h"
#include "inverterManagement.h"

#define BLYNK_SEND_ATOMIC
#define BLYNK_PRINT Serial

#include <BlynkApiArduino.h>
#include <Blynk/BlynkProtocol.h>
#include <Adapters/BlynkArduinoClient.h>
#include <WiFi.h>
#include <esp_timer.h>

char blynkTokenValue[BLYNK_STRLEN] = "FO6obJ-X8BbNQdmoojfNKDKDM-lbsKwt";
char blynkServerValue[BLYNK_STRLEN] = BLYNK_DEFAULT_DOMAIN;
char blynkPortValue[BLYNK_STRLEN] = "80";
static long lastReconnectAttempt = 0;
static long lastUpdate = 0;
static long blynkUpdateInterval = 2000;
static esp_timer_handle_t updateTimerhandle;

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


#include <BlynkWidgets.h>

void blynkUpdateAll(void *)
{

    Serial.println("Updating Blynk");
    Blynk.virtualWrite(V0, gGridLegsPower[0]);
    Blynk.virtualWrite(V1, gGridLegsPower[1]);
    Blynk.virtualWrite(V2, gGridLegsPower[2]);
    Blynk.virtualWrite(V3, gGridSumPower);
}


void blynkSetup()
{
    if(strlen(blynkTokenValue) != 32) {
        blynkTokenValue[0]= 0;
    } else {
        Blynk.config(blynkTokenValue, blynkServerValue, atoi(blynkPortValue));
    }

    esp_timer_create_args_t aArgs;
    aArgs.callback = blynkUpdateAll;
    aArgs.arg = 0;
    aArgs.name = "BlynkUpdate";
    aArgs.dispatch_method = ESP_TIMER_TASK;

    esp_err_t result = esp_timer_create(&aArgs, &updateTimerhandle);
}


bool isValid() {
    return blynkTokenValue[0] != 0 && WiFi.isConnected();
}

BLYNK_CONNECTED() {
    Serial.println("Blynk connected");
    esp_timer_start_periodic(updateTimerhandle, blynkUpdateInterval*1000);
}

BLYNK_DISCONNECTED() {
    esp_timer_stop(updateTimerhandle);
}

void blynkReconnect()
{
    if(isValid()) {
        Serial.println("Starting Blynk connect");
        Blynk.connect(0);        
    }
}


void blynkLoop(long now)
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
}

BLYNK_READ(V0)
{    
    Blynk.virtualWrite(V0, gGridLegsPower[0]);
}
