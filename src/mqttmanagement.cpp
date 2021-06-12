
#include <PubSubClient.h>
#include <WiFi.h>

#include "common.h"
#include "debugManagement.h"
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

DBG_SECT(
static bool _mqttEnabled = true;

void mqttDisable()
{
    if (_mqttEnabled)
    {
        DEBUG_I("Disabling mqtt");
        _mqttEnabled = false;
        mqttClient.disconnect();
    }
}

void mqttEnable() {
    if(!_mqttEnabled) {
        DEBUG_I("Enabling mqtt");
        _mqttEnabled = true;
        mqttReconnect();
    }
}

bool mqttEnabled() { return _mqttEnabled; }
)

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
            DEBUG_I("Subscribing to %s",subscription.c_str());
            if (!mqttClient.subscribe(subscription.c_str()))
            {
                if (Debug.isActive(Debug.ERROR))
                {
                    Debug.print("MQQT Subscribe failed\n Topic:");
                    Debug.println(subscription);
                }
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
        DBG_SECT(
            if (Debug.isActive(Debug.ERROR))
            {
                Debug.println("Could not parse message from EM3");
                Debug.print(txt[legPosInMessage]);
                Debug.print(" : ");
                Debug.println(payload);
                Debug.print(aLeg);
                Debug.print(" : ");
                Debug.println(aValue);
            })
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
                if (i == ValuePowerFactor && gGridLegValues[i][aLeg] < 0)
                {
                    gGridLegValues[i][aLeg] = fabs(gGridLegValues[i][aLeg]);
                }
                gInverterGridPowerUpdated();
                break;
            }
        }
        DBG_SECT(
            if (Debug.isActive(Debug.VERBOSE))
            {
                if (i < ValueNumValues)
                {
                    Debug.print("Leg ");
                    Debug.print(aLeg);
                    Debug.print(": ");
                    Debug.print(aValue);
                    Debug.print(" Sum: ");
                    Debug.println(gGridSumValues[i]);
                }
                else
                {
                    Debug.print("Unknown topic ");
                    Debug.println(valueType);
                }
            })
    }
}

void mqttHandleMessage(const char *topic, byte *payload, unsigned int length)
{
    DBG_SECT(
        if (Debug.isActive(Debug.VERBOSE))
        {
            Debug.printf("Message arrived [%s]\n",topic);            
            for (int i = 0; i < length; i++)
            {
                Debug.print((char)payload[i]);
            }
            Debug.println();
        })

    String txt(topic);
    char value[length + 1];
    int namePos = txt.indexOf(mqttEM3Name, 9);
    if (namePos >= 0 && length > 0 && legPosInMessage >= 0)
    {
        memcpy(value,payload,length);
        value[length]=0;
        parseEm3Result(topic, value);
    }
DBG_SECT(
    else {
        DEBUG_W("MQTT: Unknown Message");
    }
)
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
DBG_SECT(
            if (_mqttEnabled)
)
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
        if (Debug.isActive(Debug.ERROR))
        {
            Debug.print(" MQTT taskCreation failed with error ");
            Debug.println(result);
        }
        gInverterTaskHandle = 0;
    }
}