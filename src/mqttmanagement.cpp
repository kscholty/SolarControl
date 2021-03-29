
#include <PubSubClient.h>
#include <WiFi.h>

#include "common.h"

char mqttServerValue[STRING_LEN] = "fhem.home.lan";
char mqttPortValue[STRING_LEN] = "1883";
char mqttUserNameValue[STRING_LEN];
char mqttUserPasswordValue[STRING_LEN];

char mqttEM3Name[STRING_LEN] = "shellyem3-8CAAB561991E";
char mqttEM3Topic[STRING_LEN] = "/emeter/+/power";

WiFiClient ethClient;
PubSubClient mqttClient(ethClient);
long lastReconnectAttempt = 0;

boolean mqttReconnect()
{
    Serial.println("Entering mqqtReconnect");
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

void mqttHandleMessage(char *topic, byte *payload, unsigned int length)
{
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++)
    {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

void mqttSetup() {
    int port = String(mqttPortValue).toInt();
    if(port == 0) port = 1883;

    mqttClient.setServer(mqttServerValue, port);
    mqttClient.setCallback(mqttHandleMessage);
}

void mqttLoop() {
    if (!mqttClient.connected())
    {
        long now = millis();
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