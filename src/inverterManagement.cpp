

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
TaskHandle_t gInverterTaskHandle = 0;

float gGridLegsPower[3] = {0.0, 0.0, 0.0};
float gGridSumPower = 0.0;


static int inverterTarget = 30;
static long inverterTimeout = 60000;
static uint8_t pinValue;

void inverterPreInit() 
{
    pinMode(INVERTERPIN, OUTPUT);
    digitalWrite(INVERTERPIN, LOW);
    inverterLock();
}

void inverterSetupInverter() {

    inverterPreInit();
    inverterTarget = atoi(gInverterTargetValue);
    inverterTimeout = atol(gInverterTimeoutValue);
    if(inverterTimeout < 10000) {
        inverterTimeout = 10000;
    }
}

static void inverterLoop()
{

    while (true)
    {
        // Wait until the data has been updated.
        // Check for timeout to ensure that we don't crank the
        // inverter up to maximum when we lose network connection
        if (!ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(inverterTimeout)))
        {
            // We had a timeout
            inverterLock();
            Serial.println("Inverter detected timeout. MQTT not running");
        }
        else
        {
            // Seems to work fine...
            Serial.println("Inverter got notified about new values");
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
    }
}

static void inverterThradFunc(void *)
{
    inverterSetupInverter();
    inverterLoop();
}

void inverterSetup()
{
    BaseType_t result = xTaskCreate(inverterThradFunc, "inverter", 1024, 0, 2, &gInverterTaskHandle);
    if (result != pdPASS)
    {
        Serial.print(" Charge Controller taskCreation failed with error ");
        Serial.println(result);
        gInverterTaskHandle = 0;
    }
}