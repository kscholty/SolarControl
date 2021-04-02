
#include <ModbusMaster.h>
#include "chargeControllerManagement.h"


#define BOUDRATE 115200
#define MK_32(LOW,HIGH) (( ((int32_t)(HIGH)) <<16)|(LOW))

static const uint8_t chargerModbusAdresses[NUM_CHARGERS] = {1, 2};
ModbusMaster charger[NUM_CHARGERS];

// How often should the chargers be queried
// Since we query the chargers intermittendly
// This number multiplied with the number of chargers
// will be the real interval
// So NUM_CAHRGERS*updateIntervalMilis is the frequency for each one.
static unsigned long updateIntervalMilis = 1000;
static unsigned int nextController = 0;

unsigned int chargerValues[NUM_CHARGERS][NUM_CHARGER_VALUES];

void chargeControllerSetupCotroller()
{
    Serial.begin(BOUDRATE);
    for (int i = 0; i < NUM_CHARGERS; ++i)
    {
        charger[i].begin(chargerModbusAdresses[i], Serial2);
        memset(chargerValues[i], 0, NUM_CHARGER_VALUES * sizeof(chargerValues[0][0]));
    }
}

void updateController(uint index)
{
#if DEBUG
    Serial.print("Updating controller ");
    Serial.println(index + 1);
#endif

    uint8_t result = charger[index].readInputRegisters(0x3100, 8);

    if (result == charger[index].ku8MBSuccess)
    {

        chargerValues[index][PV_VOLTAGE] = charger[index].getResponseBuffer(0x00);
#if DEBUG
        Serial.print("PV Voltage: ");
        Serial.println(chargerValues[index][PV_VOLTAGE]);
#endif
        chargerValues[index][PV_CURRENT] = charger[index].getResponseBuffer(0x01);
#if DEBUG
        Serial.print("PV Current: ");
        Serial.println(chargerValues[index][PV_CURRENT]);
#endif
        chargerValues[index][PV_POWER] = MK_32(charger[index].getResponseBuffer(0x02),charger[index].getResponseBuffer(0x03));
#if DEBUG
        Serial.print("PV Power: ");
        Serial.println(chargerValues[index][PV_POWER]);
#endif
        chargerValues[index][BATT_VOLTAGE] = charger[index].getResponseBuffer(0x04);
#if DEBUG
        Serial.print("Battery Voltage: ");
        Serial.println(chargerValues[index][BATT_VOLTAGE]);
#endif
        chargerValues[index][BATTERY_CHARGE_CURRENT] = charger[index].getResponseBuffer(0x05);
#if DEBUG
        Serial.print("Battery Charge Current: ");
        Serial.println(chargerValues[index][BATTERY_CHARGE_CURRENT]);
#endif
        chargerValues[index][BATTERY_CHARGE_POWER] = MK_32(charger[index].getResponseBuffer(0x06), charger[index].getResponseBuffer(0x07));
#if DEBUG
        Serial.print("PV Power: ");
        Serial.println(chargerValues[index][PV_POWER]);
#endif
    }
}


void chargeControllerThradFunc(void*) {
    TickType_t previousTime = xTaskGetTickCount();
    chargeControllerSetupCotroller();
    while(true) {
        vTaskDelayUntil(&previousTime, updateIntervalMilis / portTICK_PERIOD_MS);
        updateController(nextController);
        nextController = (nextController + 1) % NUM_CHARGERS;
    }
}

void chargeControllerSetup()
{
    TaskHandle_t taskId;
    xTaskCreate(chargeControllerThradFunc, "chrgCntrl", 1024, 0, 1, &taskId);
}


