#include <Arduino.h>

#include "wifimanagement.h"
#include "blynkmanagement.h"
#include "invertermanagement.h"
#include "chargeControllerManagement.h"
#include "bmsManagement.h"
#include "excessControlManagement.h"
#include "debugManagement.h"
#include "modbusManagement.h"
#include "modbusClientManagement.h"


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.setRxBufferSize(512);
  Serial2.begin(115200);
  inverterPreInit();
  wifiSetup();
  modbusClientSetup();
  chargeControllerSetup();
  inverterSetup();  
  bmsSetup();
  setupExcessManagement(); // Has to be after the inverter
  blynkSetup(); // This should be last, in order to have all data available
  debugSetup();
  modbusSetup();  
}

// This one is executed on CPU 1
void loop() {
  unsigned long now = millis();
  wifiLoop(now);
  blynkLoop(now);
  modbusLoop(now);
  debugLoop(now);
}


