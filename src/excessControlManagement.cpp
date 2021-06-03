#include <Arduino.h>
#include "common.h"

#include "excessControlManagement.h"
#include "chargeControllerManagement.h"
#include "inverterManagement.h"


// The purpose of this code is to ramp up the inverter
// As long as the PV can produce more power.
// If the battery cannot take all the energy the pV creates an we have currently no need for all the power
// we produce it anyway and send it to the grid. 
// This is an option that can be switched on and off.

// We check every 10 seconds
#define HOLDTIME 10000
#define EXCESSSTEP 10
#define EPSILON 3

int gExcessTarget = 0.0;
TaskHandle_t gExcessTaskId = 0;
int gCurrentPowerCreated = 0;

int calculateInstantPower()
{
    unsigned int result = 0;
    for (int i = 0; i < NUM_CHARGERS; ++i)
    {
        if (chargerIsValid(i))
        {
            result += chargerValues[i][BATTERY_CHARGE_POWER];
        }
    }
    return (int)result / 100;
}

void excessManagementLoop(void *)
{
    // Wait some time to give the system time to reach a good state.
    vTaskDelay(pdMS_TO_TICKS(10000));

    int lastPowerCreated = calculateInstantPower();
    int diffPower;

    int laststep = EXCESSSTEP;
    gExcessTarget = lastPowerCreated > gInverterTarget ? gInverterTarget + EXCESSSTEP : gInverterTarget;

    while (1)
    {
        ulTaskNotifyTake(0, portMAX_DELAY);

        gCurrentPowerCreated = calculateInstantPower();
        if (gCurrentPowerCreated < gInverterTarget)
        {
            // PV creates even less than required...
            // Don't request anything. This will lead to the inverterTarget to be used.
            gExcessTarget = gInverterTarget;
            laststep = 0;
        }
        else
        {
            diffPower = lastPowerCreated - gCurrentPowerCreated;
            if (abs(diffPower) < EPSILON)
            {
                // Nothing major changed
                // Undo the last step made
                // And assume we reached the MPP
                gExcessTarget -= laststep;
                laststep = 0;
            }
            else
            {
                if (laststep == 0)
                {
                    laststep = EXCESSSTEP;
                }
                // Relevant change detected
                if (diffPower > 0)                
                {
                    // Less production
                    // Move into the opposite direction                    
                    laststep *= -1;
                }
                gExcessTarget += laststep;
                lastPowerCreated = gCurrentPowerCreated;                
            }

            if (gExcessTarget < gInverterTarget)
            {
                // Just to be on the safe side...
                // Go back to the default limit.
                gExcessTarget = gInverterTarget;
                laststep = 0;
            }            
        }
        inverterSetRealTarget();
    }
}

void setupExcessManagement()
{

    if (gInverterExcessToGrid)
    {
        BaseType_t result = xTaskCreate(excessManagementLoop, "excess", 2048, 0, 2, &gExcessTaskId);
        if (result != pdPASS)
        {
            Serial.print("excess taskCreation failed with error ");
            Serial.println(result);
            gExcessTaskId = 0;
        }
    }
}
