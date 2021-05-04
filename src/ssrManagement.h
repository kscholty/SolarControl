
#pragma once

#include "chargeControllerManagement.h"

#define SSR_PIN_1 21
#define SSR_PIN_2 25


extern bool gBatteryFull;
extern bool gBatteryEmpty;

extern bool ssrSwitch(CHARGERS charger, bool on);
extern bool chargerStatus(CHARGERS charger);

extern void ssrSetup();
extern void ssrLoop(unsigned long);


extern char gNtpServerValue[STRING_LEN];
extern char gTimezoneValue[NUMBER_LEN];
extern char gLatitudeValue[NUMBER_LEN];
extern char gLongitudeValue[NUMBER_LEN];

