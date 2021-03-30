

#include <Arduino.h>

#include "inverterManagement.h"

#define INVERTERPIN 4 

char gInverterTargetValue[STRING_LEN]="30";
char gInverterTimeoutValue[STRING_LEN] = "60000";
long gInverterLastUpdateReceived = 0;
bool gInverterShutdown = true;

float gGridLegsPower[3] = {0.0, 0.0, 0.0};
float gGridSumPower = 0;    

static int inverterTarget = 30;
static long inverterTimeout = 60000;
static uint8_t pinValue;

void inverterPreInit() 
{
    pinMode(INVERTERPIN, OUTPUT);
    digitalWrite(INVERTERPIN, LOW);
    inverterLock();
}

void inverterSetup() {

    inverterPreInit();
    inverterTarget = atoi(gInverterTargetValue);
    inverterTimeout = atol(gInverterTimeoutValue);
    if(inverterTimeout < 10000) {
        inverterTimeout = 10000;
    }
}

void inverterLoop(long now) {

    
    // Make sure the inverter stops producing energy if
    // connection to grid power detector is lost.
    if (now - gInverterLastUpdateReceived > inverterTimeout)
    {
        inverterLock();
    }
    else
    {
        inverterUnlock();
    }

    if (!inverterLocked())
    {
        // Here the whole magic happenes :-)
        pinValue = gGridSumPower > inverterTarget ? LOW : HIGH;
    }
    else
    {
        pinValue = LOW;
    }
    digitalWrite(INVERTERPIN, pinValue);
}

