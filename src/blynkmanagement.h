
#pragma once

#include "common.h"
#include "debugManagement.h"
#define BLYNK_STRLEN 64

#define BLYNK_GREEN     "#23C48E"
#define BLYNK_BLUE      "#04C0F8"
#define BLYNK_YELLOW    "#ED9D00"
#define BLYNK_RED       "#D3435C"
#define BLYNK_DARK_BLUE "#5F7CD8"

DBG_SECT(
#define BLYNK_VPIN_MQTT_ENABLE V100
#define BLYNK_VPIN_PID_P V101
#define BLYNK_VPIN_PID_I V102
#define BLYNK_VPIN_PID_D V103
#define BLYNK_VPIN_PID_OK V104

)

#define BLYNK_VPIN_LEG_0 V0
#define BLYNK_VPIN_LEG_1 V1
#define BLYNK_VPIN_LEG_2 V2
#define BLYNK_VPIN_ALL_LEGS V3

#define FIRST_CHARGER_VPIN V4

#define BLYNK_VPIN_INVERTER_POWER        (NUM_CHARGERS*NUM_CHARGER_VALUES+FIRST_CHARGER_VPIN+1)
#define BLYNK_VPIN_INVERTER_CURRENT      (BLYNK_VPIN_INVERTER_POWER+1)
#define BLYNK_VPIN_INVERTER_VOLTAGE      (BLYNK_VPIN_INVERTER_CURRENT+1)
#define BLYNK_VPIN_INVERTER_POWER_FACTOR (BLYNK_VPIN_INVERTER_VOLTAGE+1)
#define BLYNK_VPIN_INVERTER_TARGET       (BLYNK_VPIN_INVERTER_POWER_FACTOR+1)
#define BLYNK_VPIN_INVERTER_EXCESS       (BLYNK_VPIN_INVERTER_TARGET+1)


#define FIRST_BATTERY_VPIN V40
#define BLYNK_VPIN_TOTALVOLTAGE FIRST_BATTERY_VPIN
#define BLYNK_VPIN_CURRENT (FIRST_BATTERY_VPIN+1)
#define BLYNK_VPIN_CAPACITYREMAIN (FIRST_BATTERY_VPIN+2)
#define BLYNK_VPIN_NOMINALCAPACITY (FIRST_BATTERY_VPIN+3)
#define BLYNK_VPIN_CYCLELIFE  (FIRST_BATTERY_VPIN+4)
#define BLYNK_VPIN_STATEOFCHARGE (FIRST_BATTERY_VPIN+5)
#define BLYNK_VPIN_FETSTATUSCHARGE (FIRST_BATTERY_VPIN+6)
#define BLYNK_VPIN_FETSTATUSDISCHARGE (FIRST_BATTERY_VPIN+7)
#define BLYNK_VPIN_TEMP_1  (FIRST_BATTERY_VPIN+8)  
#define BLYNK_VPIN_TEMP_2 (FIRST_BATTERY_VPIN+9)

#define BLYNK_VPIN_OVCELL  (BLYNK_VPIN_TEMP_2+1)
#define BLYNK_VPIN_UVCELL  (BLYNK_VPIN_TEMP_2+2)
#define BLYNK_VPIN_OVBATTERY  (BLYNK_VPIN_TEMP_2+3)
#define BLYNK_VPIN_UVBATTERY  (BLYNK_VPIN_TEMP_2+4)
#define BLYNK_VPIN_OTCHARGE  (BLYNK_VPIN_TEMP_2+5)
#define BLYNK_VPIN_OTDISCHARGE  (BLYNK_VPIN_TEMP_2+6)
#define BLYNK_VPIN_OCCHARGE  (BLYNK_VPIN_TEMP_2+7)
#define BLYNK_VPIN_OCDISCHARGE  (BLYNK_VPIN_TEMP_2+8)

#define BLYNK_VPIN_BLE_CONNECTED (BLYNK_VPIN_OCDISCHARGE+1)

#define BALANCE_STATUS(VAL,CELL) (((VAL)>>(CELL)) & 0x1)
#define BLYNK_VPIN_CELLDIFF V59

#define FIRST_CELL_VOLTAGE V60
#define BLYNK_VPIN_CELL_VOLTAGE(CELL) (FIRST_CELL_VOLTAGE+(CELL))

    
// Charger ID can be either 0 or 1
// VPIN is the index of the pin of the controller (0...n) to address.
#define BLYNK_CHARGER_PIN(charger_id, VPIN) (((charger_id)*NUM_CHARGER_VALUES) + (VPIN) + FIRST_CHARGER_VPIN)
#define BLYNK_CHARGER_1_ERROR V38
#define BLYNK_CHARGER_2_ERROR V39

extern char blynkTokenValue[BLYNK_STRLEN];
extern char blynkServerValue[BLYNK_STRLEN];
extern char blynkPortValue[NUMBER_LEN];

extern void blynkSetup();
extern void blynkReconnect();
extern void blynkLoop(unsigned long);