
#include <PubSubClient.h>
#include <WiFi.h>

#include "common.h"
#include "inverterManagement.h"
#include "mqttmanagement.h"



char mqttServerValue[STRING_LEN] = "fhem.home.lan";
char mqttPortValue[STRING_LEN] = "1883";
char mqttUserNameValue[STRING_LEN];
char mqttUserPasswordValue[STRING_LEN];

char mqttEM3Name[STRING_LEN] = "shellyem3-8CAAB561991E";
int mqttEM3NameLength = 0;

char mqttEM3Topic[STRING_LEN] = "/emeter/+/power";
static int legPosInMessage = -1;

static float lastGridSumPower = 0.0;
#define NOTIFY_DELTA (float)5.0

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

static long lastReconnectAttempt = 0;

#if DEBUG
static bool _mqttEnabled = true;

void mqttDisable()
{
    if (_mqttEnabled)
    {
        Serial.println("Disabling mqtt");
        _mqttEnabled = false;
        mqttClient.disconnect();
    }
}

void mqttEnable() {
    if(!_mqttEnabled) {
        Serial.println("Enabling mqtt");
        _mqttEnabled = true;
        mqttReconnect();
    }
}

bool mqttEnabled() { return _mqttEnabled; }
#endif

bool mqttReconnect()
{
    if (WiFi.isConnected() && mqttClient.connect(thingName))
    {
        // Once connected, publish an announcement...
        mqttClient.publish(thingName, "connected");
        // ... and resubscribe
        String subscription("shellies/");
        subscription += mqttEM3Name;
        subscription += mqttEM3Topic;

        Serial.println("Subscribing to ");
        Serial.println(subscription);
        
        if (!mqttClient.subscribe(subscription.c_str()))
        {
            Serial.print("MQQT Subscribe failed\n Topic:");
            Serial.println(subscription);
        }
    }
    return mqttClient.connected();
}

static void parseEm3Result(const char *txt, const char *payload)
{
    
    int aLeg = txt[legPosInMessage] - 48;
    float aValue = atof(payload);

        if( aLeg <0 || aLeg > 2 || aValue == 0.0 )
        {
#if DEBUG
            Serial.println("Could not parse message from EM3");
            Serial.print(txt[legPosInMessage]);
            Serial.print(" : ");
            Serial.println(payload);
            Serial.print(aLeg);
            Serial.print(" : ");
            Serial.println(aValue);
#endif
        }
        else
        {
            gGridLegsPower[aLeg] = aValue;
            gGridSumPower = gGridLegsPower[0] + gGridLegsPower[1] + gGridLegsPower[2];
            gInverterLastUpdateReceived = millis();

#if DEBUG
            Serial.print("Leg ");
            Serial.print(aLeg);
            Serial.print(": ");
            Serial.print(aValue);
            Serial.print(" Sum: ");
            Serial.println(gGridSumPower);
#endif
        }
}

void mqttHandleMessage(const char *topic, byte *payload, unsigned int length)
{
#if DEBUG
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++)
    {        
        Serial.print((char)payload[i]);
    }
    Serial.println();
#endif

    String txt(topic);
    char value[length+1];
    int namePos = txt.indexOf(mqttEM3Name, 9);
    if(namePos >= 0 && length > 0 && legPosInMessage >= 0) {             
        memcpy(value,payload,length);
        value[length]=0;
        parseEm3Result(topic, value);
    } 
#if DEBUG
    else {
        Serial.println("Unknown Message");
    }
#endif

}

void mqttSetupMqtt() {
    int port = String(mqttPortValue).toInt();
    if(port == 0) port = 1883;
    mqttEM3NameLength = strlen(mqttEM3Name);
    const char *position = strchr(mqttEM3Topic, '+');
    if (position) {
        legPosInMessage = (position - mqttEM3Topic) + mqttEM3NameLength + 9; // 9 is the lenth of "shellies/"
    } else {
        legPosInMessage = -1;
    }

    mqttClient.setServer(mqttServerValue, port);
    mqttClient.setCallback(mqttHandleMessage);
    
}

static void mqttLoop()
{
    TickType_t previousTime = xTaskGetTickCount();
    while (true)
    {
        if (!mqttClient.connected())
            {
#if DEBUG
            if (_mqttEnabled)
#endif
            {
                unsigned long now = millis();
                if (now - lastReconnectAttempt > 5000)
                {
                    lastReconnectAttempt = now;
                    // Attempt to reconnect
                    if (mqttReconnect())
                    {
                        lastReconnectAttempt = 0;
                    }
                }
            }
        }
        else
        {
            do
            {
                mqttClient.loop();
            } while (wifiClient.available());

            if (gInverterTaskHandle && abs(gGridSumPower - lastGridSumPower) > NOTIFY_DELTA)
            {
                lastGridSumPower = gGridSumPower;
                xTaskNotifyGive(gInverterTaskHandle);
            }
        }
        vTaskDelayUntil(&previousTime, pdMS_TO_TICKS(500));
    }
}

static void mqttThradFunc(void *)
{
    mqttSetupMqtt();
    mqttLoop();
}

void mqttSetup()
{
    TaskHandle_t handle;
    BaseType_t result = xTaskCreate(mqttThradFunc, "mqtt", 2048, 0, 2, &handle);
    if (result != pdPASS)
    {
        Serial.print(" MQTT taskCreation failed with error ");
        Serial.println(result);
        gInverterTaskHandle = 0;
    }
}