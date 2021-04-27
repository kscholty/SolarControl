

#include <Arduino.h>
#include <ModbusMaster.h>
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
char gInverterModbusId[NUMBER_LEN] = "1";

bool gInverterShutdown = true;


TaskHandle_t gInverterTaskHandle = 0;

float gGridLegsPower[3] = {0.0, 0.0, 0.0};
float gGridSumPower = 0.0;
double gInverterPower = 0.0;
float gInverterCurrent = 0.0;
float gInverterVoltage = 0.0;
float gInverterPowerFactor = 0.0;
SemaphoreHandle_t gSerial2Mutex = NULL;;

static int inverterOffset = 0;
static double inverterTarget = 0.0f;
static long lastGridUpdateReceived = 0;

static long inverterTimeout = 60000;
static unsigned int inverterEmergencyTarget=100;
static unsigned int inverterUpdateInterval = 200;

static uint8_t pinValue;
static ModbusMaster inverterMaster;
static bool excessToGrid = false;
static bool useInverterOutput = true;



void inverterIdle() {
    vTaskDelay(10);
}

void stopPID() {

}

void startPID() {

}

bool PIDEnabled() {
    return true;
}

void inverterPreInit() 
{
    pinMode(INVERTERPIN, OUTPUT);
    digitalWrite(INVERTERPIN, LOW);    
    Serial2.begin(9600);
    //inverterSerial.begin(9600, SWSERIAL_8N1, SERIAL_RX, SERIAL_TX, false, 95, 11);
    inverterLock();
}

static void inverterSetupInverter()
{
    inverterPreInit();
    inverterOffset = atoi(gInverterOffsetValue);
    inverterTimeout = atol(gInverterTimeoutValue);
    excessToGrid = (strncmp(gSendExcessToGrid, "selected", sizeof(gSendExcessToGrid)) == 0);

    inverterEmergencyTarget = atoi(gInverterEmergencyTargetValue);    
    inverterUpdateInterval = atoi(gInverterUpdateIntervalValue);
    
    useInverterOutput = true;

    if (inverterUpdateInterval < 10)
    {
        inverterUpdateInterval = 10;
    }

    inverterTarget = inverterEmergencyTarget;

    if (inverterTimeout < 10000)
    {
        inverterTimeout = 10000;
    }

    if (useInverterOutput)
    {
        //inverterMaster.idle(inverterIdle);
        //inverterMaster.begin(inverterModbusId, Serial2);
        adcInit();        
    }
    else
    {
        // We have no access to the inverter
        // So our grid target is always inverterOffset
        inverterTarget = inverterOffset;
    }
}

void gInverterGridPowerUpdated() {
    // New grid power use detected.
    // Now our target is this value plus the current inverter production.
    // Minus an offest to avoid overproduction.
    inverterTarget = gGridSumPower+gInverterCurrent-inverterOffset;
    #if DEBUG
    Serial.print("Inverter Target: ");
    Serial.println(inverterTarget);
    #endif
    lastGridUpdateReceived = millis();
    xTaskNotifyGive(gInverterTaskHandle);
}


bool ReadInverter() {
    gInverterCurrent =  adcGetCurrent();
    gInverterVoltage = 230;
    gInverterPowerFactor = 1;
    gInverterPower = gInverterCurrent*gInverterVoltage / gInverterPowerFactor;  // Use grid values later on

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
            inverterTarget = inverterEmergencyTarget;
#if DEBUG
            Serial.println("Inverter detected timeout. MQTT not running");
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
                if (gInverterPower < inverterTarget)
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
                pinValue = gGridSumPower > inverterOffset ? HIGH : LOW;
            }
        }
        else
        {
            pinValue = LOW;
        }
        digitalWrite(INVERTERPIN, pinValue);

#if DEBUG
        static int count = 0;
        if(count == 0) {
        Serial.print(" Z: ");
        Serial.print(inverterTarget);
        Serial.print(" I: ");
        Serial.print(gInverterPower);
        Serial.print(" C ");
        Serial.println(gInverterCurrent);
        } else { 
            count = (count+1) % 100;
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

#if 0
#define MK_32(LOW,HIGH) (( ((int32_t)(HIGH)) <<16)|(LOW))

bool ReadInverter()
{
    // Critical section here access to Serial2
    uint8_t result = 255;
    // Start critical section here
    //  if (xSemaphoreTake(gSerial2Mutex, pdMS_TO_TICKS(inverterUpdateInterval)))
    {
        result = inverterMaster.readInputRegisters(0, 10);
        //    xSemaphoreGive(gSerial2Mutex);
    }
    // End critical section here
    if (result == inverterMaster.ku8MBSuccess)
    {
        gInverterPower = MK_32(inverterMaster.getResponseBuffer(3), inverterMaster.getResponseBuffer(4)) / 10.0f;

        gInverterCurrent = MK_32(inverterMaster.getResponseBuffer(1), inverterMaster.getResponseBuffer(2)) / 1000.0f;
        gInverterVoltage = inverterMaster.getResponseBuffer(0) / 10.0f;
        gInverterPowerFactor = inverterMaster.getResponseBuffer(8) / 100.0f;

        static float lastPower = 0;
        
        if (lastPower != gInverterPower)
        {
            xTaskNotifyGive(gInverterTaskHandle);
            lastPower = gInverterPower;
#if DEBUG
            // Serial.print("New power from inverter: ");
            // Serial.println(gInverterPower);
#endif
        }
    }
#if DEBUG
    else
    {
        Serial.print("Inverter query Failed with error: ");
        Serial.println(result);
    }
#endif
    return (result == inverterMaster.ku8MBSuccess);
}
#endif


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

