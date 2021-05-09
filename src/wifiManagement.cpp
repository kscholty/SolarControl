

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <esp_wifi.h>
#include <WiFi.h>          //https://github.com/esp8266/Arduino

#include <time.h>
//needed for library
#include <DNSServer.h>
#include <WebServer.h>

#include <IotWebConf.h>
#include <IotWebConfUsing.h> // This loads aliases for easier class names.


#include "wifimanagement.h"
#include "blynkmanagement.h"
#include "mqttmanagement.h"
#include "inverterManagement.h"
#include "chargeControllerManagement.h"
#include "ssrManagement.h"


// -- Initial password to connect to the Thing, when it creates an own Access Point.
const char wifiInitialApPassword[] = "123456";

// -- Configuration specific key. The value should be modified if config structure was changed.
#define CONFIG_VERSION "S1"

// -- When CONFIG_PIN is pulled to ground on startup, the Thing will use the initial
//      password to buld an AP. (E.g. in case of lost password)
#define CONFIG_PIN  -1

// -- Status indicator pin.
//      First it will light up (kept LOW), on Wifi connection it will blink,
//      when connected to the Wifi it will turn off (kept HIGH).
#define STATUS_PIN LED_BUILTIN

// -- Method declarations.
void handleRoot();
// -- Callback methods.
void configSaved();
bool formValidator(iotwebconf::WebRequestWrapper*);

DNSServer dnsServer;
WebServer server(80);

bool needReset = false;



IotWebConf iotWebConf(thingName, &dnsServer, &server, wifiInitialApPassword, CONFIG_VERSION);

IotWebConfParameterGroup blynkGroup = IotWebConfParameterGroup("Blynk configuration");
IotWebConfTextParameter blynkTokenParam = IotWebConfTextParameter("Blynk Token", "blynkToken", blynkTokenValue, BLYNK_STRLEN);
IotWebConfTextParameter blynkServerParam = IotWebConfTextParameter("Blynk server", "blynkServer", blynkServerValue, STRING_LEN, blynkServerValue);
IotWebConfNumberParameter blynkPortParam = IotWebConfNumberParameter("Blynk port", "blynkPort", blynkPortValue, NUMBER_LEN, blynkPortValue);

IotWebConfParameterGroup mqttGroup = IotWebConfParameterGroup("MQTT configuration");
IotWebConfTextParameter mqttServerParam = IotWebConfTextParameter("MQTT server", "mqttServer", mqttServerValue, STRING_LEN, mqttServerValue);
IotWebConfNumberParameter mqttPortParam = IotWebConfNumberParameter("MQTT port", "mqttPort", mqttPortValue, NUMBER_LEN, mqttPortValue);
IotWebConfTextParameter mqttUserNameParam = IotWebConfTextParameter("MQTT user", "mqttUser", mqttUserNameValue, STRING_LEN);
IotWebConfPasswordParameter mqttUserPasswordParam = IotWebConfPasswordParameter("MQTT password", "mqttPass", mqttUserPasswordValue, STRING_LEN);
IotWebConfTextParameter mqttEm3NameParam = IotWebConfTextParameter("EM3 Name", "em3name", mqttEM3Name, STRING_LEN, mqttEM3Name);


IotWebConfParameterGroup inverterGroup = IotWebConfParameterGroup("Inverter configuration");
IotWebConfNumberParameter inverterLegParam = IotWebConfNumberParameter("Inverter leg", "invLeg", gInverterLegValue, STRING_LEN, gInverterLegValue);
IotWebConfNumberParameter inverterUpdateIntervalParam = IotWebConfNumberParameter("Inverter update interval [ms]", "invUp", gInverterUpdateIntervalValue, NUMBER_LEN, gInverterUpdateIntervalValue);
IotWebConfNumberParameter inverterTargetValueParam = IotWebConfNumberParameter("Inverter Offset [W]", "invOff", gInverterOffsetValue, NUMBER_LEN, gInverterOffsetValue);
IotWebConfNumberParameter inverterTimeoutParam = IotWebConfNumberParameter("Inverter Timeout [ms]", "invTimeout", gInverterTimeoutValue, NUMBER_LEN, gInverterTimeoutValue);
IotWebConfNumberParameter inverterEmergencyTargetValueParam = IotWebConfNumberParameter("Default output [W]", "EmOut", gInverterEmergencyTargetValue, NUMBER_LEN, gInverterEmergencyTargetValue);
IotWebConfCheckboxParameter inverterEcessValueParam = IotWebConfCheckboxParameter("Send excess power to grid","excess",gSendExcessToGrid,NUMBER_LEN,true);

IotWebConfParameterGroup chargerGroup = IotWebConfParameterGroup("Charge controller configuration");
IotWebConfNumberParameter charger1Id = IotWebConfNumberParameter("Charger 1 ModbusId","ch1modbus", gChargerModbusAdressesValue[0],4);
IotWebConfNumberParameter charger2Id = IotWebConfNumberParameter("Charger 2 ModbusId", "ch2modbus", gChargerModbusAdressesValue[1], 4);
IotWebConfNumberParameter chargerUdpateInterval = IotWebConfNumberParameter("Charger upd. interval [s]", "chupd", gChargerUpdateIntervalValue, sizeof(gChargerUpdateIntervalValue), gChargerUpdateIntervalValue);

IotWebConfParameterGroup dayNightGroup = IotWebConfParameterGroup("Values for handling activation of components");
IotWebConfTextParameter ntpServerNameParam = IotWebConfTextParameter("NTP server", "ntpserver", gNtpServerValue, STRING_LEN,gNtpServerValue);
IotWebConfTextParameter timezoneParam = IotWebConfTextParameter("Time offset to GMT [h]", "TZ", gTimezoneValue, NUMBER_LEN, gTimezoneValue);
IotWebConfTextParameter latitudeParam = IotWebConfTextParameter("Latitude", "lat", gLatitudeValue, STRING_LEN, gLatitudeValue);
IotWebConfTextParameter longitudeParam = IotWebConfTextParameter("Longitude", "lon", gLongitudeValue, STRING_LEN, gLongitudeValue);
IotWebConfTextParameter inveterShellyNameParam = IotWebConfTextParameter("inverterShelly", "invShell", ginverterShellyValue, STRING_LEN, ginverterShellyValue);




void wifiConnected()
{
  if(strlen(gNtpServerValue)) {
    configTime(atoi(gTimezoneValue)*3600, 0, gNtpServerValue);
  }
   ArduinoOTA.begin();
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

  inverterGroup.addItem(&inverterLegParam);
  inverterGroup.addItem(&inverterUpdateIntervalParam);
  inverterGroup.addItem(&inverterTargetValueParam);
  inverterGroup.addItem(&inverterTimeoutParam);
  inverterGroup.addItem(&inverterEmergencyTargetValueParam);
  inverterGroup.addItem(&inverterEcessValueParam);


  chargerGroup.addItem(&charger1Id);
  chargerGroup.addItem(&charger2Id);
  chargerGroup.addItem(&chargerUdpateInterval);


  dayNightGroup.addItem(&ntpServerNameParam);
  dayNightGroup.addItem(&timezoneParam);
  dayNightGroup.addItem(&latitudeParam);
  dayNightGroup.addItem(&longitudeParam);


  iotWebConf.setStatusPin(STATUS_PIN);
  iotWebConf.setConfigPin(CONFIG_PIN);

  iotWebConf.addParameterGroup(&mqttGroup);
  iotWebConf.addParameterGroup(&blynkGroup);
  iotWebConf.addParameterGroup(&inverterGroup);
  iotWebConf.addParameterGroup(&chargerGroup);
  iotWebConf.addParameterGroup(&dayNightGroup);

  iotWebConf.setConfigSavedCallback(&configSaved);
  iotWebConf.setWifiConnectionCallback(&wifiConnected);


  iotWebConf.setFormValidator(formValidator);
  iotWebConf.getApTimeoutParameter()->visible = true;

  // -- Initializing the configuration.
  iotWebConf.init();
  

  // -- Set up required URL handlers on the web server.
  server.on("/", handleRoot);
  server.on("/config", [] { iotWebConf.handleConfig(); });
  server.onNotFound([]() { iotWebConf.handleNotFound(); });

  Serial.println("Ready.");
}

void wifiLoop(unsigned long now)
{
  // -- doLoop should be called as frequently as possible.
  iotWebConf.doLoop();
  ArduinoOTA.handle();

  if(needReset) {
      Serial.println("Rebooting after 1 second.");
      iotWebConf.delay(1000);
      ESP.restart();
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

  struct tm timeinfo;
  char tod[128];
  if(getLocalTime(&timeinfo)){
    strftime(tod,128,"%A, %B %d %Y %H:%M:%S",&timeinfo) ;
  } else {
    strcpy(tod,"unknown");
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

  String s = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>";
  s += "<title>Solar controller</title></head><body>";
  s += "Current time: ";
  s += String(tod);
  s += "<br><br><b>Values</b> <ul>";
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

bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper)
{
  Serial.println("Validating form.");
  bool result = true;

  int l = 0;

  l = server.arg(mqttServerParam.getId()).length();
  if (l> 0 && l < 3)
  {
    mqttServerParam.errorMessage = "MQTT server should have a minimum of 3 characters!";
    result = false;
  } 

  l = server.arg(blynkServerParam.getId()).length();
  if (l>0 && l < 3)
  {
    blynkServerParam.errorMessage = "Blynk server should have a minimum of 3 characters!";
    result = false;
  }

  l = server.arg(blynkTokenParam.getId()).length();
  if(l>0 && l!= 32) {
    blynkTokenParam.errorMessage = "Blynk token has to have length 32";
    result = false;
  }

  
  
  if (server.arg(inverterUpdateIntervalParam.getId()).toInt() < 20)
  {
    inverterUpdateIntervalParam.errorMessage = "Inverter update interval must be >= 20";
    result = false;
  }

  l = server.arg(inverterEmergencyTargetValueParam.getId()).toInt();
  if ( l < 0 || l > 1000)
  {
    inverterEmergencyTargetValueParam.errorMessage = "Inverter default Target Value must be between 0 and 1000";
    result = false;
  }

  if (server.arg(inverterTimeoutParam.getId()).toInt() < 10000)
  {
    inverterTimeoutParam.errorMessage = "Inverter Timeout Value must be >= 10000";
    result = false;
  }

  l = server.arg(inverterLegParam.getId()).toInt();
  if (l < 1  || l> 3) {
    inverterLegParam.errorMessage = "Leg for inverter must be between 1 and 3'";
    result = false;
  }

  if (server.arg(chargerUdpateInterval.getId()).toInt() < 10) {
    chargerUdpateInterval.errorMessage = "Charger update interval must be > 10s";
    result = false;
  }

  String idString = server.arg(charger1Id.getId());
  idString.trim();
  if (!idString.isEmpty())
  {
    int chargerId = idString.toInt();
    if (chargerId < 1 || chargerId > 255)
    {
      charger1Id.errorMessage = "Charger 1 Modbus ID must be between 1 and 255";
      result = false;
    }
  }

  idString = server.arg(charger2Id.getId());
  idString.trim();

  if (!idString.isEmpty())
  {
    int chargerId = idString.toInt();
    if (chargerId < 1 || chargerId > 255)
    {
      charger2Id.errorMessage = "Charger 2 Modbus ID must be between 1 and 255";
      result = false;
    }
  }

  l = server.arg(timezoneParam.getId()).toInt();
  if(l<-12 || l>12 ) {
      timezoneParam.errorMessage = "Timezone offset must be between -12h and 12h";
      result = false;
  }
  

  return result;
  }