
#include <Arduino.h>
#include "common.h"
#include "ssrManagement.h"
#include "inverterManagement.h"
#include "chargeControllerManagement.h"

bool gBatteryFull = false;
bool gBatteryEmpty = false;

char gNtpServerValue[STRING_LEN]="pool.ntp.org";
char gTimezoneValue[NUMBER_LEN] = "1";
char gLatitudeValue[NUMBER_LEN] = "50.937531"; // Köln...
char gLongitudeValue[NUMBER_LEN] = "6.960279";


bool chargerEnabled[NUM_CHARGERS] = {false,false};
int chargerPin[NUM_CHARGERS] = {SSR_PIN_1,SSR_PIN_2};


bool chargerStatus(CHARGERS charger) {
    return chargerEnabled[charger];
}

bool ssrSwitch(CHARGERS charger, bool on)
{
    bool ret = chargerEnabled[charger];
    chargerEnabled[charger] = on;
    digitalWrite(chargerPin[charger], on?HIGH:LOW);  
    return ret;
}


void ssrSetup() {

    for(int i=0;i<NUM_CHARGERS;++i) {
        pinMode(chargerPin[i],OUTPUT);
        if(chargerIsValid(i)) {
            chargerEnabled[i] = true;
            digitalWrite(chargerPin[i], HIGH);  
        } else {
            chargerEnabled[i] = false;
            digitalWrite(chargerPin[i], LOW);  
        }
    }  
}

void handleBatteryFull()
{
    if (!gInverterExcessToGrid)
    {
        for(int i=0;i<NUM_CHARGERS;++i) {       
            chargerEnabled[i] = false;
            digitalWrite(chargerPin[i], LOW);  
        }
    }
}

void handleBatteryEmpty() {
    
}

void ssrLoop(unsigned long now)  {
    if(gBatteryFull) {
        handleBatteryFull();
    } else if(gBatteryEmpty) {
        handleBatteryEmpty();
    }
}

