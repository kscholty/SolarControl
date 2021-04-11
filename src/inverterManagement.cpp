

#include <Arduino.h>
#include <ModbusMaster.h>
#include "inverterManagement.h"

#if DEBUG
#include "mqttmanagement.h"

int lastModification = 0;
#endif

#define SERIAL1_TX 23
#define SERIAL1_RX 22
//#define BATTERY_FULL

// This is the distance we want to stay away from 0 grid consumption
// This is used to comensate for overshooting

char gInverterOffsetValue[NUMBER_LEN]="30";
char gInverterTimeoutValue[NUMBER_LEN] = "60000";
char gInverterEmergencyTargetValue[NUMBER_LEN] = "100";
char gSendExcessToGrid[NUMBER_LEN] = "selected";
char gInverterUpdateIntervalValue[NUMBER_LEN] = "2000";
char gInverterModbusId[NUMBER_LEN] = "0";

bool gInverterShutdown = true;


TaskHandle_t gInverterTaskHandle = 0;

float gGridLegsPower[3] = {0.0, 0.0, 0.0};
float gGridSumPower = 0.0;
float gInverterPower = 0.0;
SemaphoreHandle_t gSerial2Mutex = NULL;;

static int inverterOffset = 0;
static float inverterTarget = 0.0f;
static long lastGridUpdateReceived = 0;

static long inverterTimeout = 60000;
static unsigned int inverterEmergencyTarget=100;
static unsigned int inverterUpdateInterval = 2000;
static unsigned int inverterModbusId = 3;

static uint8_t pinValue;
static ModbusMaster inverterMaster;
static bool excessToGrid = false;
static bool useInverterOutput = true;

static void inverterModbusThreadFunc(void *);


void inverterIdle() {
    vTaskDelay(10);
}

void inverterPreInit() 
{
    pinMode(INVERTERPIN, OUTPUT);
    digitalWrite(INVERTERPIN, LOW);    
    Serial2.begin(9600);
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
    inverterModbusId = atoi(gInverterModbusId);

    useInverterOutput = (inverterModbusId > 0);

    if (inverterUpdateInterval < 1000)
    {
        inverterUpdateInterval = 1200;
    }

    inverterTarget = inverterEmergencyTarget;

    if (inverterTimeout < 10000)
    {
        inverterTimeout = 10000;
    }

    if (useInverterOutput)
    {
        inverterMaster.idle(inverterIdle);
        inverterMaster.begin(inverterModbusId, Serial2);
        
        TaskHandle_t modbusThread;
        BaseType_t result = xTaskCreate(inverterModbusThreadFunc, "invModbus", 1024, 0, 2, &modbusThread);
        if (result != pdPASS)
        {
            useInverterOutput = false;
            Serial.print("Inverter task Creation failed with error ");
            Serial.println(result);
        }
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
    inverterTarget = gGridSumPower+gInverterPower-inverterOffset;
    #if DEBUG
    Serial.print("Inverter Target: ");
    Serial.println(inverterTarget);
    #endif
    lastGridUpdateReceived = millis();
    xTaskNotifyGive(gInverterTaskHandle);
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
            // We had a timeout. Nobody is talking anymore
            // So we are careful and bring the
            // output to 0.
            inverterLock();
#if DEBUG
            Serial.println("Inverter detected timeout. MQTT and Inverter not running");
#endif            
        }
        else
        {
            if(millis() - lastGridUpdateReceived > inverterTimeout) {
                // MQTT has a problem it seems...
                // So let's go to our  default output
                inverterTarget = inverterEmergencyTarget;
            } 
            // Seems to work fine...
            inverterUnlock();            
        }

        if (!inverterLocked())
        {
            if (useInverterOutput)
            {
                // Here the whole magic happenes :-)
                // Let's try to match the output power to the the target
                pinValue = gInverterPower < inverterTarget ? HIGH : LOW;
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
    }
}

static void inverterThradFunc(void *)
{
    inverterSetupInverter();
    inverterLoop();
}

#define MK_32(LOW,HIGH) (( ((int32_t)(HIGH)) <<16)|(LOW))

bool ReadInverter()
{
    // Critical section here access to Serial2    
#if DEBUG
    Serial.println("Querying Inverter");
#endif
    uint8_t result = 255;
    inverterMaster.clearTransmitBuffer();
    inverterMaster.clearResponseBuffer();
// Start critical section here
    if (xSemaphoreTake(gSerial2Mutex, pdMS_TO_TICKS(inverterUpdateInterval)))
    {
        #define NUMVALUES(LOW,HIGH) (HIGH-LOW+1)
        result = inverterMaster.readHoldingRegisters(86, NUMVALUES(86,87));
        xSemaphoreGive(gSerial2Mutex);
    }
// End critical section here
    if (result == inverterMaster.ku8MBSuccess)
    {
        // Do update here....
        // End critical section here
        int32_t currentPower = MK_32(inverterMaster.getResponseBuffer(0),inverterMaster.getResponseBuffer(1));
        gInverterPower = currentPower / 10.0f;
        xTaskNotifyGive(gInverterTaskHandle);   
        #if DEBUG
        for(int i= 0;i<NUMVALUES(86,87);++i) {
        Serial.print(i+86);
        Serial.print(" : ");
        uint16_t res = inverterMaster.getResponseBuffer(i);
        Serial.print(res); Serial.print(" "); Serial.println(makeWord(res & 0xFF,res>>8));
        
        }
        Serial.print("New power from inverter: ");
        Serial.println(currentPower);
        #endif
    }
#if DEBUG
    else
    {
        Serial.print("Failed with error: ");
        Serial.println(result);        
    }
#endif    
    return (result == inverterMaster.ku8MBSuccess);
}


static void inverterModbusThreadFunc(void *) {

    TickType_t previousTime = xTaskGetTickCount();
    while (true)
    {
        if(!ReadInverter()) {
            // On failure we wait some more time...
            previousTime = xTaskGetTickCount();
        }
        vTaskDelayUntil(&previousTime, pdMS_TO_TICKS(inverterUpdateInterval));
    }
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

