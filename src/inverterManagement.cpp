

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
char gInverterOutputMax[NUMBER_LEN] = "2200";
char gInverterOutputMin[NUMBER_LEN] = "-800";

bool gInverterShutdown = false;


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
static  int inverterOutputMax = 2000;
static  int inverterOutputMin = 0;
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
    inverterOutputMax = atoi(gInverterOutputMax);
    inverterOutputMin = atoi(gInverterOutputMin);
    useInverterOutput = true;

    if (inverterUpdateInterval < 1)
    {
        inverterUpdateInterval = 1;
    }


    aPID.SetSampleTime(500-2);
    aPID.SetOutputLimits(inverterOutputMin,inverterOutputMax);
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
    #if 0
    if( (gBms[0]->basicInfo().getTotalVoltage() < 5000) && (gCurrentPowerCreated > 80)) {
        // If we are below 49V, dont use more than the PV produces
        // If this is more than required, put the rest into the battery.
        realTarget = min((double)gCurrentPowerCreated,gInverterTarget);
    } else {
        realTarget = max((double)gExcessTarget,gInverterTarget);
    }
    #else
        realTarget = max((double)gExcessTarget,gInverterTarget);
    #endif
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

static void inverterLoop() {
  TickType_t previousTime = xTaskGetTickCount();
  while (true) {
    // Seems to work fine...
    inverterUnlock();

    if (!inverterLocked()) {
      // Here the whole magic happenes :-)
      // Let's try to match the output power to the the target
      ReadInverter();
      aPID.Compute();
      if (millis() - lastGridUpdateReceived > inverterTimeout) {
        // MQTT has a problem it seems...
        // So let's go to our  default output
        gInverterTarget = inverterEmergencyTarget;
        inverterSetRealTarget();

        // Come back here in 500ms if no update occured inbetween to update
        // the realTarget.
        lastGridUpdateReceived = millis() + 500 - inverterTimeout;
        // DEBUG_W("Inverter detected timeout. MQTT not running");
      }

      // The factor is roughly equivalent to the power
      // The only real important thing is that it's positive if we
      // need more power and negative if we need less.
      if (realTarget > 0) {
        gVoltageFactor = realTarget - gInverterPower;
        if (gVoltageFactor > gMaxVoltageFactor) {
          gVoltageFactor = gMaxVoltageFactor;
        } else if (gVoltageFactor < gMinVoltageFactor) {
          gVoltageFactor = gMinVoltageFactor;
        }
        pinValue = gVoltageFactor > 0 ? HIGH : LOW;
      } else {
        // Real Target is negative
        // Make sure we don't produce anything
        pinValue = LOW;
        gVoltageFactor = gMinVoltageFactor;
      }
    }
    digitalWrite(INVERTERPIN, pinValue);
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
        if (Debug.isActive(Debug.ERROR)) {
            Debug.print("Inverter taskCreation failed with error ");
            Debug.println(result);
        }
        gInverterTaskHandle = 0;
    }    
}

