
#include <Arduino.h>

#include "common.h"
#include "debugManagement.h"
#include "bmsManagement.h"
#include "modbusManagement.h"
#include "Battery.h"

BMS::CBmsBase* gBms[bmsCount] = {0,0,0};

char gBmsUpdateIntervalValue[NUMBER_LEN] = "3000";
char gBmsDummyValue[STRING_LEN];


bool gBmsDisconnect = true;
bool gBmsUpdated = false;

static TaskHandle_t taskId;

static unsigned long bmsUpdateShortIntervalMilis = 3000;
static unsigned long bmsUpdateLongIntervalMilis = 6000;

static bool bmsSetupBms() {
  bmsUpdateShortIntervalMilis = atoi(gBmsUpdateIntervalValue);
  if (bmsUpdateShortIntervalMilis < 500) {
    bmsUpdateShortIntervalMilis = 3000;
  }
  bmsUpdateLongIntervalMilis = 3 * bmsUpdateShortIntervalMilis;
  gBms[0] = new JKBMS_t(Serial2, gSerial2Mutex);
  gBms[1] = new QUCCBMS_t(Serial2, gSerial2Mutex);
  gBmsUpdated = false;
  for (int i = 0; i < bmsCount; ++i) {
    if (gBms[i]) {
      gBms[i]->setLowFrequencyDelay(bmsUpdateLongIntervalMilis);
      gBms[i]->setHighFrequencyDelay(bmsUpdateShortIntervalMilis);
      gBms[i]->setup();
    }
  }
  return true;
}

void bmsEnable(bool on) {
    gBmsDisconnect = !on;
}

void bmsLoop(void*) {
#define FORCEINTERVAL 150000
  TickType_t previousTime = xTaskGetTickCount();
  unsigned long lastRead = 0;
  for (int i = 0; i < bmsCount; ++i) {
    if (gBms[i]) {
      gBmsUpdated |= gBms[i]->doLoop();
    }
  }
  while (true) {
    vTaskDelayUntil(&previousTime, pdMS_TO_TICKS(bmsUpdateShortIntervalMilis));

    if (!gBmsDisconnect) {
      // Query the BMS
      for (int i = 0; i < bmsCount; ++i) {
        if (gBms[i]) {
          gBmsUpdated |= gBms[i]->doLoop();
        }
      }
    } else {
      // Once in a while we update the basic data nevertheless.
      unsigned long now = previousTime * portTICK_PERIOD_MS;
      if (now - lastRead > FORCEINTERVAL) {
        DEBUG_I("Forcing BMS basic data update\n");
        for (int i = 0; i < bmsCount; ++i) {
          if (gBms[i]) {
            gBmsUpdated |= gBms[i]->doLoop();            
          }
          lastRead = now;          
        }
      }
    }
  }
}

void bmsSetup()
{
    if(bmsSetupBms()) {
        BaseType_t result = xTaskCreate(bmsLoop, "bms", 2048, 0, 1, &taskId);

        if (result != pdPASS)
        {
            rprintE(" BMS taskCreation failed with error ");
            rprintEln(result);
        }
    }
}
