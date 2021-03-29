
#include <PubSubClient.h>
#include <WiFi.h>

#include "common.h"
#include "inverterManagement.h"

char mqttServerValue[STRING_LEN] = "fhem.home.lan";
char mqttPortValue[STRING_LEN] = "1883";
char mqttUserNameValue[STRING_LEN];
char mqttUserPasswordValue[STRING_LEN];

char mqttEM3Name[STRING_LEN] = "shellyem3-8CAAB561991E";
int mqttEM3NameLength = 0;

char mqttEM3Topic[STRING_LEN] = "/emeter/+/power";

WiFiClient ethClient;
PubSubClient mqttClient(ethClient);

static long lastReconnectAttempt = 0;


boolean mqttReconnect()
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

static void parseEm3Result(const char *txt, const char *payload, int namePos)
{
    // We found the name of the EM3 in the topic
    // We now need the number in the topic
    // It should be at position name.length()+lengthOf("/emeter/")
    int aLeg = txt[namePos+mqttEM3NameLength + 8] - 48;
    float aValue = atof(payload);

        if( aLeg <0 || aLeg > 2 || aValue == 0.0 )
        {
#if DEBUG
            Serial.println("Could not parse message from EM3");
            Serial.print(txt[namePos + mqttEM3NameLength + 8]);
            Serial.print(" : ");
            Serial.println(payload);
            Serial.print(aLeg);
            Serial.print(" : ");
            Serial.println(aValue);
#endif
        } else {
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
        }
#endif
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
    if(namePos >= 0 && length > 0) {             
        memcpy(value,payload,length);
        value[length]=0;
        parseEm3Result(topic, value, namePos);
    } 
#if DEBUG
    else {
        Serial.println("Unknown Message");
    }
#endif

}

void mqttSetup() {
    int port = String(mqttPortValue).toInt();
    if(port == 0) port = 1883;
    mqttEM3NameLength = strlen(mqttEM3Name);

    mqttClient.setServer(mqttServerValue, port);
    mqttClient.setCallback(mqttHandleMessage);
}

void mqttLoop(long now) {
    if (!mqttClient.connected())
    {        
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
    else
    {
        // Client connected
        mqttClient.loop();        
    }

    
}