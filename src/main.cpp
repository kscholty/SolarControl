#include <Arduino.h>

#include "wifimanagement.h"
#include "blynkmanagement.h"
#include "mqttmanagement.h"
#include "invertermanagement.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting up...");
  inverterSetup();
  wifiSetup();
  mqttSetup();
  blynkSetup();
  inverterSetup();
}

void loop() {
  long now = millis();
  wifiLoop(now);
  mqttLoop(now);
  inverterLoop(now);
  blynkLoop(now);
  // put your main code here, to run repeatedly:
}