#include <Arduino.h>
#include <Esp8266Wifi.h>
#include "ShellyDevice.h"


#define SSID "your-SSID"
#define PASSWORD "your-password"

#define PM1_IP "192.168.100.164"
#define EM3_IP "192.168.100.12"

static ShellyDevice PM1(PM1_IP,2000);
static ShellyDevice EM3;

void setup() {
  Serial.begin(115200);
  Serial.println();

  WiFi.begin(SSID, PASSWORD);

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());

  //Set Delay of EM3
  EM3.setDelay(4000);
  

  // This will fail because no IP is set
  if(!EM3.updateRelay(0)) {
    Serial.print("3EM 1. Expected Relay Error: ");
    Serial.println(EM3.getLastErrorString());    
  }

  // Set IP
  EM3.setIp(EM3_IP);
  
  // Should fail because #of relay still unknown
  if(!EM3.updateRelay(0)) {
    Serial.print("3EM 2. Expected Relay Error: ");
    Serial.println(EM3.getLastErrorString());    
  }

  // Should succeed
  if(!EM3.updateNow()) {
    Serial.print("3EM Update Error: ");
    Serial.println(EM3.getLastErrorString());    
  }

  // Should succeed
  if(!EM3.updateRelay(0)) {
    Serial.print("3EM Relay Error: ");
    Serial.println(EM3.getLastErrorString());    
  }
  Serial.printf("EM3 Relay: %d\n",EM3.getRelayIson(0));

}

void loop() {
  bool read = false;
  if(PM1.doLoop()) {
    if(!PM1.getLastError()) {
      Serial.printf("1PM %s #meters %d, #emeters %d, #relays %d\n",PM1.getDeviceType().c_str(),PM1.getNumMeters(), PM1.getNumEmeters(), PM1.getNumRelays());
      Serial.printf("Power %dmW Relay: %d\n", PM1.getPower(0),PM1.getRelayIson(0));
    } else {
      Serial.print("1PM Error: ");
      Serial.println(PM1.getLastErrorString());
    }
      read = true;
  }

  if (EM3.doLoop()) {
    if (!EM3.getLastError()) {
      Serial.printf("3EM %s #meters %d, #emeters %d, #relays %d\n",
                    EM3.getDeviceType().c_str(), EM3.getNumMeters(),
                    EM3.getNumEmeters(), EM3.getNumRelays());
      Serial.printf("Power %dmW %dmW %dmW\n", EM3.getPower(0), EM3.getPower(1),
                    EM3.getPower(2));
      Serial.printf("Current %dmA %dmA %dmA\n", EM3.getCurrent(0),
                    EM3.getCurrent(1), EM3.getCurrent(2));
      Serial.printf("Voltage %dmV %dmV %dmV\n", EM3.getVoltage(0),
                    EM3.getVoltage(1), EM3.getVoltage(2));
      Serial.printf("Relay: %d\n", EM3.getRelayIson(0));
    } else {
      Serial.print("3EM Error: ");
      Serial.println(EM3.getLastErrorString());
    }
    read = true;
  }
  if (read) {
    Serial.println("--------------------------------------------\n");
  }
  read = false;

}