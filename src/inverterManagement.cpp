

#include <Arduino.h>
#include <PID_v1.h>
#include "debugManagement.h"
#include "excessControlManagement.h"
#include "inverterManagement.h"
#include "adc.h"
#include "modbusManagement.h"
#include "bmsManagement.h"
#include "chargeControllerManagement.h"

#if DBUG_ON
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
double gInverterTarget = 0.0;


bool gInverterExcessToGrid = false;

static float realTarget = 0.0f;
static uint inverterLeg = 2;

static double inverterOffset = 0;
static double currentPowerUsed = 0;

static long lastGridUpdateReceived = 0;

static long inverterTimeout = 60000;
static float inverterEmergencyTarget=100;
static unsigned int inverterUpdateInterval = 200;

static uint8_t pinValue;

static bool useInverterOutput = true;

//Specify the links and initial tuning parameters
double Kp=0.32, Ki=0.3, Kd=0.1;
static PID aPID(&currentPowerUsed, &gInverterTarget, &inverterOffset, Kp, Ki, Kd, REVERSE);

void inverterActivatePidValues() {
    aPID.SetTunings(Kp,Ki,Kd);
}

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

    if (inverterUpdateInterval < 1)
    {
        inverterUpdateInterval = 1;
    }


    aPID.SetSampleTime(500);
    aPID.SetOutputLimits(0,1000);
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

    aPID.SetMode(AUTOMATIC);

}

void inverterSetRealTarget()
{
    if( (gBms[0]->basicInfo().getTotalVoltage() < 5000) && (gCurrentPowerCreated > 80)) {
        // If we are below 49V, dont use more than the PV produces
        // If this is more than required, put the rest into the battery.
        realTarget = min((double)gCurrentPowerCreated,gInverterTarget);
    } else {
        realTarget = max((double)gExcessTarget,gInverterTarget);
    }
    
}

void gInverterGridPowerUpdated()
{
    // New grid power use detected.
    // Now our target is this value plus the current inverter production.
    // Minus an offest to avoid overproduction.
    
    //gInverterVoltage = gGridLegValues[ValueVoltage][inverterLeg];
    gInverterPowerFactor = gGridLegValues[ValuePowerFactor][inverterLeg];
    lastGridUpdateReceived = millis();
    //gInverterTarget = gGridSumValues[ValuePower] + gInverterPower - inverterOffset;
   
    currentPowerUsed = gGridSumValues[ValuePower];

    if (currentPowerUsed < 0 || fabs(currentPowerUsed - inverterOffset) > 2*inverterOffset)
    {
        aPID.SetTunings(Kp, Ki, Kd);
    }
    else
    {
        aPID.SetTunings(Kp / 2, Ki, Kd);
    }
    aPID.Compute();
    inverterSetRealTarget();
    
}

bool ReadInverter() {
    gInverterCurrent =  adcGetCurrent();    
    gInverterVoltage = adcGetVoltage();
    //gInverterPower = gInverterCurrent*gInverterVoltage * gInverterPowerFactor; 
    //gInverterPower = gInverterCurrent*gInverterVoltage;  
    gInverterPower = adcGetPower();  
    return true;   
}

static void inverterLoop()
{
    TickType_t previousTime = xTaskGetTickCount();
    while (true)
    {        
        // Seems to work fine...
        inverterUnlock();

        if (!inverterLocked())
        {
            if (useInverterOutput)
            {
                // Here the whole magic happenes :-)
                // Let's try to match the output power to the the target
                ReadInverter();
                aPID.Compute();
                if (millis() - lastGridUpdateReceived > inverterTimeout)
                {
                    // MQTT has a problem it seems...
                    // So let's go to our  default output
                    gInverterTarget = inverterEmergencyTarget;
                    inverterSetRealTarget();

                    // Come back here in 500ms if no update occured inbetween to update the 
                    // realTarget.
                    lastGridUpdateReceived = millis() + 500 - inverterTimeout;                     
                    //DEBUG_W("Inverter detected timeout. MQTT not running");
                } 

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
        DBG_SECT(
            if (Debug.isActive(Debug.DEBUG))
            {
                static unsigned long lastPrint = 0;
                if (millis() - lastPrint > 1000)
                {
                    lastPrint = millis();
                    Debug.print(" Z: ");
                    Debug.print(gInverterTarget);
                    Debug.print(" I: ");
                    Debug.print(gInverterPower);
                    Debug.print(" C ");
                    Debug.println(gInverterCurrent);
                }
            })
        vTaskDelayUntil(&previousTime, pdMS_TO_TICKS(inverterUpdateInterval));
        //vTaskDelayUntil(&previousTime, pdMS_TO_TICKS(10));
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
        if (Debug.isActive(Debug.ERROR)) {
            Debug.print("Inverter taskCreation failed with error ");
            Debug.println(result);
        }
        gInverterTaskHandle = 0;
    }    
}

