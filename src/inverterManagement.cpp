

#include <Arduino.h>
#include "excessControlManagement.h"
#include "inverterManagement.h"
#include "adc.h"

#if DEBUG
#include "mqttmanagement.h"

int lastModification = 0;
#endif

#define SERIAL_TX 23
#define SERIAL_RX 22
//#define BATTERY_FULL

// This is the distance we want to stay away from 0 grid consumption
// This is used to comensate for overshooting

char gInverterOffsetValue[NUMBER_LEN]="30";
char gInverterTimeoutValue[NUMBER_LEN] = "60000";
char gInverterEmergencyTargetValue[NUMBER_LEN] = "100";
char gSendExcessToGrid[NUMBER_LEN] = "selected";
char gInverterUpdateIntervalValue[NUMBER_LEN] = "2000";
char gInverterLegValue[NUMBER_LEN] = "2";

bool gInverterShutdown = true;


TaskHandle_t gInverterTaskHandle = 0;


float gGridLegValues[ValueNumValues][3] = {{0,0,0},{0,0,0},{231,231,231},{1,1,1}};
float gGridSumValues[ValueNumValues] = {0,0,0,0};


double gInverterPower = 0.0;
float gInverterCurrent = 0.0;
float gInverterVoltage = 0.0;
float gInverterPowerFactor = 0.0;
float gInverterTarget = 0.0f;

bool gInverterExcessToGrid = false;

static float realTarget = 0.0f;
static float inverterOffset = 0;
static uint inverterLeg = 2;

static long lastGridUpdateReceived = 0;

static long inverterTimeout = 60000;
static float inverterEmergencyTarget=100;
static unsigned int inverterUpdateInterval = 200;

static uint8_t pinValue;

static bool useInverterOutput = true;



void inverterIdle() {
    vTaskDelay(10);
}

void inverterPreInit() 
{    
    pinMode(INVERTERPIN, OUTPUT);
    digitalWrite(INVERTERPIN, LOW);        
    inverterLock();
}

static void inverterSetupInverter()
{
    inverterPreInit();
    inverterOffset = atoi(gInverterOffsetValue);
    inverterTimeout = atol(gInverterTimeoutValue);
    gInverterExcessToGrid = (strncmp(gSendExcessToGrid, "selected", sizeof(gSendExcessToGrid)) == 0);

    inverterEmergencyTarget = atoi(gInverterEmergencyTargetValue);    
    inverterUpdateInterval = atoi(gInverterUpdateIntervalValue);
    inverterLeg = atoi(gInverterLegValue)-1;

    useInverterOutput = true;

    if (inverterUpdateInterval < 10)
    {
        inverterUpdateInterval = 10;
    }

    gInverterTarget = inverterEmergencyTarget;
    gInverterVoltage = gGridLegValues[ValueVoltage][inverterLeg];
    gInverterPowerFactor = gGridLegValues[ValuePowerFactor][inverterLeg];

    if (inverterTimeout < 10000)
    {
        inverterTimeout = 10000;
    }

    if (useInverterOutput)
    {      
        adcInit();        
    }
    else
    {
        // We have no access to the inverter
        // So our grid target is always inverterOffset
        gInverterTarget = inverterOffset;
    }
}

void inverterSetRealTarget()
{
    realTarget = (float)gExcessTarget;
}

void gInverterGridPowerUpdated()
{
    // New grid power use detected.
    // Now our target is this value plus the current inverter production.
    // Minus an offest to avoid overproduction.

    gInverterVoltage = gGridLegValues[ValueVoltage][inverterLeg];
    gInverterPowerFactor = gGridLegValues[ValuePowerFactor][inverterLeg];
    lastGridUpdateReceived = millis();
    gInverterTarget = gGridSumValues[ValuePower] + gInverterPower - inverterOffset;
    if (gExcessTaskId)
    {
        // Let's re-calculate the excess values...
        xTaskNotifyGive(gExcessTaskId);
    } else {
        realTarget = gInverterTarget;
    }
}

bool ReadInverter() {
    gInverterCurrent =  adcGetCurrent();    
    //gInverterPower = gInverterCurrent*gInverterVoltage * gInverterPowerFactor; 
    gInverterPower = gInverterCurrent*gInverterVoltage;  

    return true;   
}

static void inverterLoop()
{
    TickType_t previousTime = xTaskGetTickCount();
    while (true)
    {

        if (millis() - lastGridUpdateReceived > inverterTimeout)
        {
            // MQTT has a problem it seems...
            // So let's go to our  default output
            gInverterTarget = inverterEmergencyTarget;
#if DEBUG
            //Serial.println("Inverter detected timeout. MQTT not running");
#endif
        }
        // Seems to work fine...
        inverterUnlock();

        if (!inverterLocked())
        {
            if (useInverterOutput)
            {
                // Here the whole magic happenes :-)
                // Let's try to match the output power to the the target
                ReadInverter();
                if (gInverterPower < realTarget)
                {
                    pinValue = HIGH;
                }
                else
                {
                    pinValue = LOW;
                }
            }
            else
            {
                // Since we can't read the inverter's output
                // We try to bring the grid usage down to offset
                pinValue = gGridSumValues[ValuePower] > inverterOffset ? HIGH : LOW;
            }
        }
        else
        {
            pinValue = LOW;
        }
        digitalWrite(INVERTERPIN, pinValue);

#if DEBUG
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 1000)
        {
            lastPrint = millis();
            Serial.print(" Z: ");
            Serial.print(gInverterTarget);
            Serial.print(" I: ");
            Serial.print(gInverterPower);
            Serial.print(" C ");
            Serial.println(gInverterCurrent);
        }
#endif
        vTaskDelayUntil(&previousTime, pdMS_TO_TICKS(inverterUpdateInterval));
    }
}

static void inverterThradFunc(void *)
{
    inverterSetupInverter();
    inverterLoop();
}

void inverterSetup()
{    
    BaseType_t result = xTaskCreate(inverterThradFunc, "inverter", 4096, 0, 2, &gInverterTaskHandle);
    if (result != pdPASS)
    {
        Serial.print("Inverter taskCreation failed with error ");
        Serial.println(result);
        gInverterTaskHandle = 0;
    }    
}

