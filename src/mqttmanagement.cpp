
#include <PubSubClient.h>
#include <WiFi.h>

#define STRING_LEN 128



char mqttServerValue[STRING_LEN] = "fhem.home.lan";
char mqttPortValue[STRING_LEN] = "1883";
char mqttUserNameValue[STRING_LEN];
char mqttUserPasswordValue[STRING_LEN];

WiFiClient ethClient;
PubSubClient mqttClient(ethClient);
long lastReconnectAttempt = 0;

boolean reconnect()
{
    if (mqttClient.connect("arduinoClient"))
    {
        // Once connected, publish an announcement...
        mqttClient.publish("outTopic", "hello world");
        // ... and resubscribe
        mqttClient.subscribe("inTopic");
    }
    return mqttClient.connected();
}

void mqttHandleMessage(char *topic, byte *payload, unsigned int length)
{
    // handle message arrived
}

void mqttSetup() {
    int port = String(mqttPortValue).toInt();
    if(port == 0) port = 1883;

    mqttClient.setServer(mqttServerValue, port);
    mqttClient.setCallback(mqttHandleMessage);
}

void mqttLoop() {
    if (!mqttClient.connected() && WiFi.isConnected())
    {
        long now = millis();
        if (now - lastReconnectAttempt > 5000)
        {
            lastReconnectAttempt = now;
            // Attempt to reconnect
            if (reconnect())
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