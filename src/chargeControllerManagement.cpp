
#include <ModbusRTU.h>
#include "debugManagement.h"
#include "chargeControllerManagement.h"
#include "excessControlManagement.h"
#include "modbusManagement.h"


#define BAUDRATE 115200
#define MK_32(LOW,HIGH) (( ((int32_t)(HIGH)) <<16)|(LOW))
#define chargerValid(INDEX) (chargerModbusAdresses[(INDEX)] > 0) 
#define MAXRETRIES 2


static bool chargerReadPvAndBattery(uint16_t,uint16_t *,uint16_t );
static bool readTemps(uint16_t,uint16_t *,uint16_t );
static bool readExternalTemps(uint16_t,uint16_t *,uint16_t );
static bool readStates(uint16_t,uint16_t *,uint16_t );

enum STATE {
        START = -1,        
        R_PV_BAT=0,        
        R_TEMPS = 1,    
        R_EXT_TEMPS = 2,    
        R_STATES = 3,
        TASK,
        NEXTCHARGER,
        SLEEP,
        NUMSTATES       
};

// Parameters are index, buffer, size of buffer
typedef bool (*ParserFunc_t)(uint16_t,uint16_t *,uint16_t );
struct StateData {
 uint16_t offset;
 uint16_t numRegisters;
 ParserFunc_t parser;
 STATE nextState;
 uint8_t resent;
};

static uint16_t recvBuffer[16];
static StateData states[NUMSTATES] = {
        {0x3100,8,chargerReadPvAndBattery,R_TEMPS,0}, 
        {0x3110,3,readTemps,R_STATES /*R_EXT_TEMPS*/,0}, 
        {0x311B,1,readExternalTemps,R_STATES,0}, 
        {0x3200,2,readStates,NEXTCHARGER,0}
};

static STATE currentState = START;
static STATE lastState = SLEEP;
static uint16_t currentIndex  = 0; 

char gChargerModbusAdressesValue[NUM_CHARGERS][4];
char gChargerUpdateIntervalValue[6] = "30";
unsigned int gChargerNumValidChargers = 0;
bool gChargerValuesChanged[NUM_CHARGERS];

static uint8_t chargerModbusAdresses[NUM_CHARGERS];

unsigned long gChargerUpdateIntervalMilis = 15000;


static ModbusRTU modbusClient;

SemaphoreHandle_t gSerial2Mutex = 0;
StaticSemaphore_t xSemaphoreBuffer;

unsigned int chargerValues[NUM_CHARGERS][CHARGER_ARRAY_SIZE];

bool chargerIsValid(CHARGERS charger) {
        return chargerValid(charger);
}

static unsigned int calculateChargerIds() {
        unsigned int count = 0;
        for (int i = 0; i < NUM_CHARGERS; ++i)
        {
                chargerModbusAdresses[i] = atoi(gChargerModbusAdressesValue[i]);
                gChargerValuesChanged[i] = false;
                if (chargerValid(i))
                {                       
                        ++count;
                }
        }
        return count;
}

static void chargeControllerSetupController()
{        
        Serial2.begin(BAUDRATE);
        modbusClient.begin(&Serial2);    
        modbusClient.master();    
        for (int i = 0; i < NUM_CHARGERS; ++i)
        {
                memset(chargerValues[i], 0, NUM_CHARGER_VALUES * sizeof(chargerValues[0][0]));
        }
}

void changeState(STATE newState) {
        DEBUG_V("CC: Switching state from %d to %d\n", currentState,newState);
        lastState = currentState;
        currentState = newState;
}


bool hRegCallback(Modbus::ResultCode event, uint16_t, void *)
{
        xSemaphoreGive(gSerial2Mutex);
        StateData *state = &states[lastState];

        if (event == Modbus::EX_SUCCESS)
        {
                state->parser(currentIndex, recvBuffer, state->numRegisters);
                gChargerValuesChanged[currentIndex] = true;
        }
        else 
        {
                DEBUG_E("Charger query for %hu in state %hu failed with %hu\n", currentIndex, lastState, event);
                if ( (state->resent < MAXRETRIES) && (event == Modbus::EX_TIMEOUT))
                {
                        // If we got a timeout we try it again.
                        changeState(lastState);
                        state->resent++;
                        vTaskDelay(pdMS_TO_TICKS(200));
                        return false;
                } else {
                        // Failed finally.
                        // Call the parser to handle the error
                        state->parser(currentIndex, 0, 0);
                }
        }

        state->resent = 0;
        changeState(state->nextState);        
        return true;
}

static bool sendRequest()
{
        bool result = true;

        if (currentState <= START || currentState >= TASK)
        {
                return false;
        }

        DEBUG_V("CC: Sending request in state %d\n", currentState);
        if (xSemaphoreTake(gSerial2Mutex, pdMS_TO_TICKS(gChargerUpdateIntervalMilis / 3 * NUM_CHARGERS)) == pdTRUE)
        {
                StateData *state = &states[currentState];

                if (Serial2.baudRate() != BAUDRATE)
                {
                        Serial2.updateBaudRate(BAUDRATE);
                }
                
                changeState(TASK);
                if (!modbusClient.readIreg(chargerModbusAdresses[currentIndex], state->offset, recvBuffer, state->numRegisters, hRegCallback))
                {
                        xSemaphoreGive(gSerial2Mutex);
                        changeState(state->nextState);
                        result = false;
                }
        }

        return result;
}


void setErrorCnt(uint mask, uint shift, bool result) {        
        unsigned int cnt=0;
        unsigned int value = chargerValues[currentIndex][CHARGER_READ_ERROR];
        if(!result) {                
                cnt = value >> shift;
                ++cnt;                
        }
        chargerValues[currentIndex][CHARGER_READ_ERROR] = (value & mask) | (cnt <<shift);        
}

static bool chargerReadPvAndBattery(uint16_t index, uint16_t *buffer, uint16_t bufSize)
{
        bool returnVal = false;

        if (bufSize >= 8)
        {
                uint IX = 0;
                chargerValues[index][PV_VOLTAGE] = buffer[IX++];
                DBG_SECT(
                    rprintD("PV Voltage: ");
                    rprintDln(chargerValues[index][PV_VOLTAGE]);)
                chargerValues[index][PV_CURRENT] = buffer[IX++];
                DBG_SECT(
                    rprintD("PV Current: ");
                    rprintDln(chargerValues[index][PV_CURRENT]);)

                uint16_t low = buffer[IX++];
                uint16_t high = buffer[IX++];
                chargerValues[index][PV_POWER] = MK_32(low, high);
                DBG_SECT(
                    rprintD("PV Power: ");
                    rprintDln(chargerValues[index][PV_POWER]);)
                chargerValues[index][BATTERY_VOLTAGE] = buffer[IX++];
                DBG_SECT(
                    rprintD("Battery Voltage: ");
                    rprintDln(chargerValues[index][BATTERY_VOLTAGE]);)
                chargerValues[index][BATTERY_CHARGE_CURRENT] = buffer[IX++];
                DBG_SECT(
                    rprintD("Battery Charge Current: ");
                    rprintDln(chargerValues[index][BATTERY_CHARGE_CURRENT]);)
                low = buffer[IX++];
                high = buffer[IX++];
                chargerValues[index][BATTERY_CHARGE_POWER] = MK_32(low, high);
                DBG_SECT(
                    rprintD("PV Power: ");
                    rprintDln(chargerValues[index][PV_POWER]);)
                if (chargerValues[index][BATTERY_CHARGE_CURRENT] > 0)
                {
                        chargerValues[index][BATTERY_CHARGE_VOLTAGE] = chargerValues[index][BATTERY_CHARGE_POWER] * 100 / chargerValues[index][BATTERY_CHARGE_CURRENT];
                }
                else
                {
                        chargerValues[index][BATTERY_CHARGE_VOLTAGE] = 0;
                }
                DBG_SECT(
                    rprintD("Battery Charge Voltage: ");
                    rprintDln(chargerValues[index][PV_POWER]);)
                returnVal = true;
        }
        
        setErrorCnt(0x00FFFF,16, returnVal);        

        return returnVal;
}

static bool readTemps(uint16_t index, uint16_t *buffer, uint16_t bufSize)
{
        bool returnVal = false;

        if (bufSize >= 3)
        {
                uint IX = 0;
                chargerValues[index][BATTERY_TEMP] = buffer[IX++];
                DBG_SECT(
                    rprintD("Battery Temp: ");
                    rprintDln(chargerValues[index][BATTERY_TEMP]);)
                chargerValues[index][CONTROLLER_INTERNAL_TEMP] = buffer[IX++];
                DBG_SECT(
                    rprintD("Controller internal Temp: ");
                    rprintDln(chargerValues[index][CONTROLLER_INTERNAL_TEMP]);)
                chargerValues[index][CONTROLLER_POWER_COMP_TEMP] = buffer[IX++];
                DBG_SECT(
                    rprintD("Controller Power component Temp: ");
                    rprintDln(chargerValues[index][CONTROLLER_POWER_COMP_TEMP]);)
                
                IX++; // Skip the next value
                chargerValues[index][BATTERY_EXTERNAL_TEMP] = buffer[IX++];
                DBG_SECT(
                    rprintD("Battery external Temp: ");
                    rprintDln(chargerValues[index][BATTERY_EXTERNAL_TEMP]);)
                returnVal = true;
        }

        setErrorCnt(0xFF00FF,8, returnVal);        
        return returnVal;
}

static bool readExternalTemps(uint16_t index, uint16_t *buffer, uint16_t bufSize)
{
        bool returnVal = false;

        if (bufSize >= 1)
        {
                uint IX = 0;                
                chargerValues[index][BATTERY_EXTERNAL_TEMP] = buffer[IX++];
                DBG_SECT(
                    rprintD("Battery external Temp: ");
                    rprintDln(chargerValues[index][BATTERY_EXTERNAL_TEMP]);)
                returnVal = true;
        }

        //setErrorCnt(0xFF00FF,8, returnVal);        
        return returnVal;
}


static bool readStates(uint16_t index, uint16_t *buffer, uint16_t bufSize)
{
        bool returnVal = false;

        if (bufSize >= 2)
        {
                uint IX = 0;
                chargerValues[index][BATTERY_STATUS] = buffer[IX++];
                chargerValues[index][CONTROLLER_STATUS] = buffer[IX++];
                returnVal = true;
        }

        setErrorCnt(0xFFFF00,0, returnVal);        
        return returnVal;
}

void setUpdateMillis()
{
        gChargerUpdateIntervalMilis = atol(gChargerUpdateIntervalValue) * 1000;
        if (gChargerUpdateIntervalMilis < 5000)
        {
                gChargerUpdateIntervalMilis = 5000;
        }
}


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


bool nextIndex() {
        uint oldIndex = currentIndex;
        do {
                currentIndex = (currentIndex+1) % NUM_CHARGERS;
        } while (!chargerIsValid(currentIndex));
        return currentIndex <= oldIndex;
}


void chargeControllerThradFunc(void *)
{
        if (gChargerNumValidChargers == 0)
        {
                DBG_SECT(
                    rprintDln("Charge controller: No valid controllers");

                )
                // That is an error and shouldn't happen!
                vTaskDelete(xTaskGetCurrentTaskHandle());
        }

        TickType_t previousTime = xTaskGetTickCount();
        uint iteration = 0;
        while (true)
        {                       
                switch (currentState)
                {
                // First entry into the loop
                case START:
                        chargeControllerSetupController();
                        currentState = NEXTCHARGER;
                        break;
                // Look for next charger to update
                case NEXTCHARGER:
                        if (nextIndex()) {
                                // We had a turn-around, therefore all chargers have been updated                                
                                gCurrentPowerCreated = calculateInstantPower();                                
                                if (gExcessTaskId)
                                {
                                        // Let's re-calculate the excess values...
                                        xTaskNotifyGive(gExcessTaskId);
                                }
                                changeState(SLEEP);
                        }
                        else {
                                // Update the next charger
                                changeState(R_PV_BAT);
                                vTaskDelay(pdMS_TO_TICKS(100));
                        }
                        break;
                // Wait some time for the next update
                case SLEEP:                        
                        vTaskDelayUntil(&previousTime, pdMS_TO_TICKS(gChargerUpdateIntervalMilis - 100));
                        changeState(R_PV_BAT);
                        iteration = (iteration+1)%3;
                        break;
                // Request has been sent. Wait for answers;
                // Next state will be set by the callback function.
                case TASK:
                        modbusClient.task();
                        break;
                case R_PV_BAT:        
                        sendRequest();
                        break;
                case R_TEMPS:
                case R_EXT_TEMPS:
                        if(iteration==0) {
                                sendRequest();
                        } else {
                                changeState(states[currentState].nextState);
                        }                        
                        break;
                case R_STATES:
                        if(iteration==1) {
                                sendRequest();
                        } else {
                                changeState(states[currentState].nextState);
                        }
                        break;  
                case NUMSTATES:
                        DEBUG_E("Reaching illegal state %d\n",currentState);
                        break;              
                }
        }
}

void chargeControllerSetup()
{
        TaskHandle_t taskId;

        gSerial2Mutex = xSemaphoreCreateMutexStatic(&xSemaphoreBuffer);
        gChargerNumValidChargers = calculateChargerIds();
        setUpdateMillis();           
        DBG_SECT(
            rprintD("Num controllers : ");
            rprintDln(gChargerNumValidChargers);)
        if (gChargerNumValidChargers > 0)
        {
                BaseType_t result = xTaskCreate(chargeControllerThradFunc, "chrgCntrl", 2048, 0, 1, &taskId);
                if (result != pdPASS)
                {
                        rprintE(" Charge Controller taskCreation failed with error ");
                        rprintEln(result);
                }
        }
}
