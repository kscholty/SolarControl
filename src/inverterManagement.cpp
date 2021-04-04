

#include <Arduino.h>
#include <ModbusMaster.h>
#include "inverterManagement.h"

#if DEBUG
#include "mqttmanagement.h"

int lastModification = 0;
#endif

#define SERIAL1_TX 23
#define SERIAL1_RX 22

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
static ModbusMaster inverterMaster;

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

void initRs232()
{
    Serial1.begin(9600, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);
    inverterMaster.begin(2, Serial1);
}

void ReadRegisterTest()
{
    #define NUMREGS 1
    uint8_t result = inverterMaster.readHoldingRegisters(0x0, NUMREGS);

    Serial.println("Querying Inverter");
    if (result == inverterMaster.ku8MBSuccess)
    {
        Serial.println("Got Answer");
        for (int i = 0; i < NUMREGS; ++i)
        {
            Serial.print(i);
            Serial.print(":");
            Serial.println(inverterMaster.getResponseBuffer(i));
        }
    }
    else
    {
        Serial.print("Failed with error: ");
        Serial.println(result);
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
#if DEBUG
            Serial.println("Inverter detected timeout. MQTT not running");
#endif            
        }
        else
        {

            // Seems to work fine...
#if DEBUG
            Serial.println("Inverter got notified about new values");
#endif
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
    initRs232();
    inverterSetupInverter();
    inverterLoop();
}

void inverterSetup()
{
    BaseType_t result = xTaskCreate(inverterThradFunc, "inverter", 1024, 0, 2, &gInverterTaskHandle);
    if (result != pdPASS)
    {
        Serial.print("Inverter taskCreation failed with error ");
        Serial.println(result);
        gInverterTaskHandle = 0;
    }
}

