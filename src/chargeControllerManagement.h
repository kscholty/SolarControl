
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
    PV_POWER,     //32Bit
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
    NUM_CHARGER_VALUES,
    CHARGER_READ_ERROR,
    CHARGER_ARRAY_SIZE
};


 low = buffer[IX++];
                high = buffer[IX++];
                chargerValues[index][BATTERY_CHARGE_POWER] = MK_32(low, high);
                DBG_SECT(
                    rprintD("PV Power: ");
                    rprintDln(chargerValues[index][PV_POWER]);)
                if (chargerValues[index][BATTERY_CHARGE_CURRENT] > 0)
                {
                        chargerValues[index][BATTERY_CHARGE_VOLTAGE] = chargerValues[index][BATTERY_CHARGE_POWER] * 100 / chargerValues[index][BATTERY_CHARGE_CURRENT];
                }
                else
                {
                        chargerValues[index][BATTERY_CHARGE_VOLTAGE] = 0;
                }
                DBG_SECT(
                    rprintD("Battery Charge Voltage: ");
                    rprintDln(chargerValues[index][PV_POWER]);)
                returnVal = true;


enum ChargerFlags_t
{
    LOAD_ENABLED = 0
};

extern char gChargerModbusAdressesValue[NUM_CHARGERS][4];
extern char gChargerUpdateIntervalValue[6];
extern unsigned int chargerValues[NUM_CHARGERS][CHARGER_ARRAY_SIZE];
extern bool gChargerValuesChanged[NUM_CHARGERS];
extern unsigned int gChargerNumValidChargers;
extern unsigned long gChargerUpdateIntervalMilis;

extern bool chargerReadPvAndBattery(uint index);
extern void chargeControllerSetup();
extern bool chargerIsValid(CHARGERS charger);
inline bool chargerIsValid(int i) { return chargerIsValid((CHARGERS)i);}

