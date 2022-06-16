
#include <Arduino.h>

#include "common.h"

#include "debugManagement.h"
#include "bmsManagement.h"
#include "modbusManagement.h"
#include "Battery.h"


// Make sure the order of thise strings eqauls the enum BMS_TYPE
const char gBmsNames[NUM_BMS_TYPES][STRING_LEN] = { "None", "JBD/QUCC", "JK BMS", "Seplos" };

BMS::CBmsBase* gBms[bmsCount] = {0,0,0};

char gBmsUpdateIntervalValue[NUMBER_LEN] = "3000";
char gBmsType[bmsCount][STRING_LEN] = {"None", "None", "None"};

static BMS_TYPE bmsTypes[bmsCount] = {NONE,NONE,NONE};
bool gBmsDisconnect = true;
bool gBmsUpdated = false;

static TaskHandle_t taskId;

static unsigned long bmsUpdateShortIntervalMilis = 3000;
static unsigned long bmsUpdateLongIntervalMilis = 6000;


void parseBmsTypes() {
  for(int bms = 0;bms<bmsCount;++bms) {
    bmsTypes[bms] = NONE;
    for(int name = 0; name<NUM_BMS_TYPES;++name) {
      if(strcmp(gBmsNames[name],gBmsType[bms])== 0) {
        bmsTypes[bms] = (BMS_TYPE)name;        
        Serial.printf("BMS %d is of type %d (%s)\n",bms,name,gBmsType[bms]);
        break;
      }
    }
  }
}

static bool bmsSetupBms() {
  bmsUpdateShortIntervalMilis = atoi(gBmsUpdateIntervalValue);
  if (bmsUpdateShortIntervalMilis < 500) {
    bmsUpdateShortIntervalMilis = 3000;
  }
  bmsUpdateLongIntervalMilis = 3 * bmsUpdateShortIntervalMilis;
  parseBmsTypes();
  for(int i = 0;i<bmsCount;++i) {
    switch(bmsTypes[i]) {
      case NONE: gBms[i] = new BMS::CDummyBms;  break;
      case JBD:  gBms[i] = new QUCCBMS_t(Serial2, gSerial2Mutex); break;
      case JK:  gBms[i] = new JKBMS_t(Serial2, gSerial2Mutex); break;
      case SEPLOS: gBms[i] = new SeplosBMS_t(Serial2, gSerial2Mutex); break;
      default: gBms[i] = 0; break;

    }
  }
 
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
