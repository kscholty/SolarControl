
#pragma once

#include "common.h"

enum CHARGERS {
    CHARGER_1=0,
    CHARGER_2=1,
    NUM_CHARGERS
};


enum ChargerValues_t
{
    PV_CURRENT = 0,
    PV_VOLTAGE,
    PV_POWER,
    BATTERY_TEMP,
    BATTERY_EXTERNAL_TEMP,
    BATTERY_VOLTAGE,
    BATTERY_STATUS,
    CONTROLLER_INTERNAL_TEMP,
    CONTROLLER_POWER_COMP_TEMP,
    CONTROLLER_STATUS,
    BATTERY_CHARGE_CURRENT,
    BATTERY_CHARGE_VOLTAGE,
    BATTERY_CHARGE_POWER,
    NUM_CHARGER_VALUES
};

enum ChargerFlags_t
{
    LOAD_ENABLED = 0
};

extern char gChargerModbusAdressesValue[NUM_CHARGERS][4];
extern char gChargerUpdateIntervalValue[6];
extern unsigned int chargerValues[NUM_CHARGERS][NUM_CHARGER_VALUES];
extern bool gChargerValuesChanged[NUM_CHARGERS];
extern unsigned int gChargerNumValidChargers;
extern unsigned long gChargerUpdateIntervalMilis;

extern bool chargerReadPvAndBattery(uint index);
extern void chargeControllerSetup();
extern bool chargerIsValid(CHARGERS charger);
inline bool chargerIsValid(int i) { return chargerIsValid((CHARGERS)i);}

