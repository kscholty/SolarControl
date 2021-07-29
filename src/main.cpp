#include <Arduino.h>

#include "wifimanagement.h"
#include "blynkmanagement.h"
#include "mqttmanagement.h"
#include "invertermanagement.h"
#include "chargeControllerManagement.h"
#include "bmsManagement.h"
#include "ssrManagement.h"
#include "excessControlManagement.h"
#include "debugManagement.h"


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  inverterPreInit();
  wifiSetup();
  mqttSetup();
  chargeControllerSetup();
  inverterSetup();  
  bmsSetup();
  setupExcessManagement(); // Has to be after the inverter
  blynkSetup(); // This should be last, in order to have all data available
  debugSetup();
  //ssrSetup();
}

// This one is executed on CPU 1
void loop() {
  unsigned long now = millis();
  wifiLoop(now);
  blynkLoop(now);
  debugLoop(now);
  //ssrLoop(now);
}


