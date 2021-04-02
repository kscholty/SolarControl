
#pragma once

#include "common.h"

#define NUM_CHARGERS 2
enum ChargerValues_t
{
    PV_CURRENT = 0,    
    PV_VOLTAGE,
    PV_POWER,
    LOAD_CURRENT,
    LOAD_VOLTAGE,
    LOAD_POWER,
    BATT_TEMP,
    BATT_VOLTAGE,
    BATT_REMAIN,
    CONTROLLER_TEMP,
    BATTERY_CHARGE_CURRENT,
    BATTERY_CHARGE_VOLTAGE,
    BATTERY_CHARGE_POWER,
    BATTERY_OVERALL_CURRENT,
    NUM_CHARGER_VALUES
};

enum ChargerFlags_t
{
    LOAD_ENABLED = 0
};

extern unsigned int chargerValues[NUM_CHARGERS][NUM_CHARGER_VALUES];

extern void chargeControllerSetup();