

#include <Arduino.h>

#include "inverterManagement.h"

#if DEBUG
#include "mqttmanagement.h"

int lastModification = 0;
#endif 

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

void inverterLoop(unsigned long now)
{

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
        pinValue = (int)gGridSumPower > inverterTarget ? HIGH : LOW;
    }
    else
    {
        pinValue = LOW;
    }
    digitalWrite(INVERTERPIN, pinValue);

#if DEBUG
    if(!mqttEnabled() && (now-lastModification> 19)) {        
        lastModification = now;
        // If pin is low, the consumption on the grid is ecpected to rise
        // If it i high, if is high, expected to decrease, since the inverter produces more.
        gGridSumPower = (pinValue == LOW) ? (gGridSumPower + 1.3) : (gGridSumPower - 1.3);
        
        if(now-gInverterLastUpdateReceived > 999) {
        Serial.print(inverterTarget);
        Serial.print(" : ");
        Serial.print(gGridSumPower);
        Serial.print(" : ");
        Serial.println((int)pinValue);
        gInverterLastUpdateReceived = now;
        }
    }
#endif
}
