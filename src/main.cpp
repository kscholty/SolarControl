#include <Arduino.h>

#include "wifimanagement.h"
#include "blynkmanagement.h"
#include "mqttmanagement.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting up...");
  wifiSetup();
  mqttSetup();
  blynkSetup();
}

void loop() {
  wifiLoop();
  mqttLoop();
  blynkLoop();
  // put your main code here, to run repeatedly:
}