

#include <Arduino.h>
#include <esp_wifi.h>
#include <WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <WebServer.h>

#include <IotWebConf.h>
#include <IotWebConfUsing.h> // This loads aliases for easier class names.

#include "wifimanagement.h"
#include "blynkmanagement.h"
#include "mqttmanagement.h"
#include "inverterManagement.h"


// -- Initial password to connect to the Thing, when it creates an own Access Point.
const char wifiInitialApPassword[] = "123456";

#define STRING_LEN 128
#define NUMBER_LEN 32

// -- Configuration specific key. The value should be modified if config structure was changed.
#define CONFIG_VERSION "S1"

// -- When CONFIG_PIN is pulled to ground on startup, the Thing will use the initial
//      password to buld an AP. (E.g. in case of lost password)
#define CONFIG_PIN  13

// -- Status indicator pin.
//      First it will light up (kept LOW), on Wifi connection it will blink,
//      when connected to the Wifi it will turn off (kept HIGH).
#define STATUS_PIN LED_BUILTIN

// -- Method declarations.
void handleRoot();
// -- Callback methods.
void configSaved();
bool formValidator();

DNSServer dnsServer;
WebServer server(80);

bool needReset = false;



IotWebConf iotWebConf(thingName, &dnsServer, &server, wifiInitialApPassword, CONFIG_VERSION);

IotWebConfParameterGroup blynkGroup = IotWebConfParameterGroup("Blynk configuration");
IotWebConfTextParameter blynkTokenParam = IotWebConfTextParameter("Blynk Token", "blynkToken", blynkTokenValue, BLYNK_STRLEN);
IotWebConfTextParameter blynkServerParam = IotWebConfTextParameter("Blynk server", "blynkServer", blynkServerValue, STRING_LEN, blynkServerValue);
IotWebConfNumberParameter blynkPortParam = IotWebConfNumberParameter("Blynk port", "blynkPort", blynkPortValue, STRING_LEN, blynkPortValue);

IotWebConfParameterGroup mqttGroup = IotWebConfParameterGroup("MQTT configuration");
IotWebConfTextParameter mqttServerParam = IotWebConfTextParameter("MQTT server", "mqttServer", mqttServerValue, STRING_LEN, mqttServerValue);
IotWebConfNumberParameter mqttPortParam = IotWebConfNumberParameter("MQTT port", "mqttPort", mqttPortValue, STRING_LEN, mqttPortValue);
IotWebConfTextParameter mqttUserNameParam = IotWebConfTextParameter("MQTT user", "mqttUser", mqttUserNameValue, STRING_LEN);
IotWebConfPasswordParameter mqttUserPasswordParam = IotWebConfPasswordParameter("MQTT password", "mqttPass", mqttUserPasswordValue, STRING_LEN);
IotWebConfTextParameter mqttEm3NameParam = IotWebConfPasswordParameter("EM3 Name", "em3name", mqttEM3Name, STRING_LEN, mqttEM3Name);
IotWebConfTextParameter mqttEm3TopicParam = IotWebConfPasswordParameter("EM3 Topic", "em3topic", mqttEM3Topic, STRING_LEN, mqttEM3Topic);

IotWebConfParameterGroup inverterGroup = IotWebConfParameterGroup("Inverter configuration");
IotWebConfTextParameter inverterTargetValueParam = IotWebConfPasswordParameter("Inverter Target [W]", "invTarget", gInverterTimeoutValue, STRING_LEN, gInverterTimeoutValue);
IotWebConfTextParameter inverterTimeoutParam = IotWebConfPasswordParameter("Inverter Timeout [ms]", "invTimeout", gInverterTargetValue, STRING_LEN, mqttEM3Topic);

void wifiConnected()
{
  mqttReconnect();
  blynkReconnect();
}

void wifiSetup()
{

  blynkGroup.addItem(&blynkServerParam);
  blynkGroup.addItem(&blynkPortParam);
  blynkGroup.addItem(&blynkTokenParam);

  mqttGroup.addItem(&mqttServerParam);
  mqttGroup.addItem(&mqttPortParam);
  mqttGroup.addItem(&mqttUserNameParam);
  mqttGroup.addItem(&mqttUserPasswordParam);
  mqttGroup.addItem(&mqttEm3NameParam);
  mqttGroup.addItem(&mqttEm3TopicParam);

  inverterGroup.addItem(&inverterTargetValueParam);
  inverterGroup.addItem(&inverterTimeoutParam);

  iotWebConf.setStatusPin(STATUS_PIN);
  iotWebConf.setConfigPin(CONFIG_PIN);

  iotWebConf.addParameterGroup(&mqttGroup);
  iotWebConf.addParameterGroup(&blynkGroup);
  iotWebConf.addParameterGroup(&inverterGroup);

  iotWebConf.setConfigSavedCallback(&configSaved);
  iotWebConf.setWifiConnectionCallback(&wifiConnected);


  iotWebConf.setFormValidator(&formValidator);
  iotWebConf.getApTimeoutParameter()->visible = true;

  // -- Initializing the configuration.
  iotWebConf.init();
  

  // -- Set up required URL handlers on the web server.
  server.on("/", handleRoot);
  server.on("/config", [] { iotWebConf.handleConfig(); });
  server.onNotFound([]() { iotWebConf.handleNotFound(); });

  Serial.println("Ready.");
}

void wifiLoop(long now)
{
  // -- doLoop should be called as frequently as possible.
  iotWebConf.doLoop();

  if(needReset) {
    if (needReset)
    {
      Serial.println("Rebooting after 1 second.");
      iotWebConf.delay(1000);
      ESP.restart();
    }
  }
}

/**
 * Handle web requests to "/" path.
 */
void handleRoot()
{
  // -- Let IotWebConf test and handle captive portal requests.
  if (iotWebConf.handleCaptivePortal())
  {
    // -- Captive portal request were already served.
    return;
  }
  String s = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>";
  s += "<title>Solar controller</title></head><body>Values";
  s += "<ul>";
  s += "<li>MQTT server: ";
  s += mqttServerValue;
  s += "<li>Blynk server: ";
  s += blynkServerValue;
  s += "</ul>";
  s += "Go to <a href='config'>configure page</a> to change values.";
  s += "</body></html>\n";

  server.send(200, "text/html", s);
}

void configSaved()
{
  Serial.println("Configuration was updated.");
  needReset = true;
}

bool formValidator()
{
  Serial.println("Validating form.");
  bool valid = true;
  int l = 0;

  l = server.arg(mqttServerParam.getId()).length();
  if (l> 0 && l < 3)
  {
    mqttServerParam.errorMessage = "MQTT server should have a minimum of 3 characters!";
    valid = false;
  }

  l = server.arg(blynkServerParam.getId()).length();
  if (l>0 && l < 3)
  {
    mqttServerParam.errorMessage = "Blynk server should have a minimum of 3 characters!";
    valid = false;
  }

  l = server.arg(blynkTokenParam.getId()).length();
  if(l>0 && l!= 32) {
    mqttServerParam.errorMessage = "Blynk token has to have length 32";
    valid = false;
  }

  if (server.arg(inverterTargetValueParam.getId()).toInt() < 0)
  {
    mqttServerParam.errorMessage = "Inverter Target Value must be a positive number";
    valid = false;
  }

  if (server.arg(inverterTimeoutParam.getId()).toInt() < 10000)
  {
    mqttServerParam.errorMessage = "Inverter Timeout Value must be >= 10000";
    valid = false;
  }
  
  return valid;
}