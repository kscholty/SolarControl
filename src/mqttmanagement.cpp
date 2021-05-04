
#include <PubSubClient.h>
#include <WiFi.h>

#include "common.h"
#include "inverterManagement.h"
#include "mqttmanagement.h"



char mqttServerValue[STRING_LEN] = "fhem.home.lan";
char mqttPortValue[NUMBER_LEN] = "1883";
char mqttUserNameValue[STRING_LEN];
char mqttUserPasswordValue[STRING_LEN];

char mqttEM3Name[STRING_LEN] = "shellyem3-8CAAB561991E";
int mqttEM3NameLength = 0;

char mqttEM3Topic[STRING_LEN] = "/emeter/+/";
static int legPosInMessage = -1;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

static long lastReconnectAttempt = 0;

static const char *topics[ValueNumValues] = {"power","current","voltage", "pf"};

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
        String subscriptionBase("shellies/");
        subscriptionBase += mqttEM3Name;
        subscriptionBase += "/emeter/+/";

        for (int i = 0; i < ValueNumValues; ++i)
        {
            String subscription = subscriptionBase + topics[i];
#if DEBUG
            Serial.println("Subscribing to ");
            Serial.println(subscription);
#endif
            if (!mqttClient.subscribe(subscription.c_str()))
            {
                Serial.print("MQQT Subscribe failed\n Topic:");
                Serial.println(subscription);
            }
        }
    }
    return mqttClient.connected();
}

static void parseEm3Result(const char *txt, const char *payload)
{

    int aLeg = txt[legPosInMessage] - 48;
    float aValue = atof(payload);

    if (aLeg < 0 || aLeg > 2 || aValue == 0.0)
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
        const char *valueType = txt + legPosInMessage + 2;
        uint i;
        for (i = 0; i < ValueNumValues; ++i)
        {
            if (!strcmp(valueType, topics[i]))
            {
                gGridLegValues[i][aLeg] = aValue;
                gGridSumValues[i] = gGridLegValues[i][0] + gGridLegValues[i][1] + gGridLegValues[i][2];
                gInverterGridPowerUpdated();
                break;
            }
        }
#if DEBUG_ON

        if (i < ValueNumValues)
        {
            Serial.print("Leg ");
            Serial.print(aLeg);
            Serial.print(": ");
            Serial.print(aValue);
            Serial.print(" Sum: ");
            Serial.println(gGridSumValues[i]);
        }
        else
        {
            Serial.print("Unknown topic ");
            Serial.println(valueType);
        }
#endif        
    }
}

void mqttHandleMessage(const char *topic, byte *payload, unsigned int length)
{
#if DEBUG_ON
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
    const char *position = strchr("/emeter/+/", '+');
    if (position) {
        legPosInMessage = mqttEM3NameLength + 9 + 8; // 9 is the lenth of "shellies/" and "/emeter/"
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