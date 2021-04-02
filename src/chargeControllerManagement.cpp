
#include <ModbusMaster.h>
#include "chargeControllerManagement.h"


#define BOUDRATE 115200

static const uint8_t chargerModbusAdresses[NUM_CHARGERS] = {1, 2};
ModbusMaster charger[NUM_CHARGERS];

// How often should the chargers be queried
// Since we query the chargers intermittendly
// This number multiplied with the number of chargers
// will be the real interval
// So NUM_CAHRGERS*updateIntervalMilis is the frequency for each one.
static unsigned long updateIntervalMilis = 1000;
static unsigned long lastUpdateAt = 0;
static unsigned int nextController = 0;

unsigned int chargerValues[NUM_CHARGERS][NUM_CHARGER_VALUES];

void chargeControllerSetup()
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
}

void chargeControllerLoop(unsigned long now)
{
    if (now - lastUpdateAt > updateIntervalMilis) {
        lastUpdateAt = now;
        updateController(nextController);
        nextController = (nextController + 1) % NUM_CHARGERS;
    }
}
