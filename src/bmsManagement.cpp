
#include <Arduino.h>

#include "common.h"
#include "debugManagement.h"
#include "bmsManagement.h"
#include "modbusManagement.h"
#include "Battery.h"


BMS_t *gBms = 0;

char gBmsUpdateIntervalValue[NUMBER_LEN] = "3000";
char gBmsDummyValue[STRING_LEN];


bool gBmsDisconnect = false;
bool gBmsUpdated = false;

static TaskHandle_t taskId;

static unsigned long bmsUpdateShortIntervalMilis = 3000;
static unsigned long bmsUpdateLongIntervalMilis = 6000;//300000;




static bool bmsSetupBms() {
   
   bmsUpdateShortIntervalMilis = atoi(gBmsUpdateIntervalValue);
   if(bmsUpdateShortIntervalMilis < 500) {
       bmsUpdateShortIntervalMilis = 3000;
   }
   gBms = new BMS_t(Serial2, gSerial2Mutex);
   if(gBms) {
       gBms->setLowFrequencyDelay(bmsUpdateLongIntervalMilis);
       gBms->setHighFrequencyDelay(bmsUpdateShortIntervalMilis);
   }
   return gBms != 0;
}



void bmsEnable(bool on) {
    gBmsDisconnect = !on;
}

void bmsLoop(void *)
{
    #define FORCEINTERVAL  150000
    TickType_t previousTime = xTaskGetTickCount();
    unsigned long lastRead = 0;
    gBms->doLoop();
    while (true)
    {
        vTaskDelayUntil(&previousTime, pdMS_TO_TICKS(bmsUpdateShortIntervalMilis));

        if (!gBmsDisconnect)
        {
            // Query the BMS
            if(gBms->doLoop()) {
                gBmsUpdated = true;
            }
            //readSingleData();
        } else {
            // Once in a while we update the basic data nevertheless.            
            unsigned long now = previousTime * portTICK_PERIOD_MS; 
            if(now - lastRead > FORCEINTERVAL) {
                DEBUG_I("Forcing BMS basic data update\n");
                if(gBms->doLoop()) {
                    lastRead = now;
                    gBmsUpdated = true;
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
