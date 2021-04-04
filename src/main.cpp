#include <Arduino.h>

#include "wifimanagement.h"
#include "blynkmanagement.h"
#include "mqttmanagement.h"
#include "invertermanagement.h"
#include "chargeControllerManagement.h"

void setup0();
void loop0();

void threadFunc(void*) {
  setup0();
  while (true)
  {
    loop0();
  }
}

// Create a task that is running on the other CPU core
// Than the calling thread.
void threadSetup()
{
  TaskHandle_t taskId;
  BaseType_t coreId = (xPortGetCoreID() + 1) % 2;
  xTaskCreatePinnedToCore(threadFunc, "loop0", 2048, 0, 1, &taskId, coreId);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  Serial.print("Starting up at core");
  Serial.println(xPortGetCoreID());

  inverterPreInit();
  wifiSetup();
  inverterSetup();
  threadSetup();
  chargeControllerSetup();
  blynkSetup(); // This should be last, in order to have all data available
}

// This one is executed on CPU 1
void loop() {
  unsigned long now = millis();
  wifiLoop(now);
  blynkLoop(now);
}

// Executed once before loop0
// starts
void setup0()
{
  // initialize your stuff here
  mqttSetup();
 
}

// This one is executed on 
// CPU 0
void loop0() {
    unsigned long now = millis();    
    mqttLoop(now);
    delay(100);
}


