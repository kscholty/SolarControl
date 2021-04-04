#include <Arduino.h>

#include "wifimanagement.h"
#include "blynkmanagement.h"
#include "mqttmanagement.h"
#include "invertermanagement.h"
#include "chargeControllerManagement.h"


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  Serial.print("Starting up at core");
  Serial.println(xPortGetCoreID());

  inverterPreInit();
  wifiSetup();
  mqttSetup();
  inverterSetup();  
  chargeControllerSetup();
  blynkSetup(); // This should be last, in order to have all data available
}

// This one is executed on CPU 1
void loop() {
  unsigned long now = millis();
  wifiLoop(now);
  blynkLoop(now);
}


