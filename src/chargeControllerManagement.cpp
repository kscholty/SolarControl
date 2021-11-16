
#include <ArduinoModbus.h>
#include "debugManagement.h"
#include "chargeControllerManagement.h"
#include "excessControlManagement.h"


#define BAUDRATE 115200
#define MK_32(LOW,HIGH) (( ((int32_t)(HIGH)) <<16)|(LOW))
#define chargerValid(INDEX) (chargerModbusAdresses[(INDEX)] > 0) 

char gChargerModbusAdressesValue[NUM_CHARGERS][4];
char gChargerUpdateIntervalValue[6] = "30";
unsigned int gChargerNumValidChargers = 0;
bool gChargerValuesChanged[NUM_CHARGERS];

static uint8_t chargerModbusAdresses[NUM_CHARGERS];

unsigned long gChargerUpdateIntervalMilis = 15000;

static RS485Class rs485Interface(Serial2,17,-1,-1);
static ModbusRTUClientClass modbusClient(rs485Interface);

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
        for (int i = 0; i < NUM_CHARGERS; ++i)
        {
//                if (chargerValid(i))
//                {
//                        charger[i].begin(chargerModbusAdresses[i], Serial2);
//                }
                memset(chargerValues[i], 0, NUM_CHARGER_VALUES * sizeof(chargerValues[0][0]));
        }
}

bool chargerReadPvAndBattery(uint index)
{
        bool returnVal = false;
        if (xSemaphoreTake(gSerial2Mutex, pdMS_TO_TICKS(gChargerUpdateIntervalMilis / 3 * NUM_CHARGERS)) == pdTRUE)
        {
                bool result;
                uint count = 0;
                
                if (Serial2.baudRate() != BAUDRATE)
                {
                        Serial2.updateBaudRate(BAUDRATE);
                }

                do
                {
                        result = modbusClient.requestFrom(chargerModbusAdresses[index], INPUT_REGISTERS, 0x3100, 8);                        
                        xSemaphoreGive(gSerial2Mutex);
                        if (result && modbusClient.available() >= 8)
                        {

                                chargerValues[index][PV_VOLTAGE] = modbusClient.read();
DBG_SECT(
                                rprintD("PV Voltage: ");
                                rprintDln(chargerValues[index][PV_VOLTAGE]);
)
                                chargerValues[index][PV_CURRENT] = modbusClient.read();
DBG_SECT(
                                rprintD("PV Current: ");
                                rprintDln(chargerValues[index][PV_CURRENT]);

)                               int32_t low = modbusClient.read();
                                int32_t high = modbusClient.read();
                                chargerValues[index][PV_POWER] = MK_32(low, high);
DBG_SECT(
                                rprintD("PV Power: ");
                                rprintDln(chargerValues[index][PV_POWER]);
)
                                chargerValues[index][BATTERY_VOLTAGE] = modbusClient.read();
DBG_SECT(
                                rprintD("Battery Voltage: ");
                                rprintDln(chargerValues[index][BATTERY_VOLTAGE]);
)
                                chargerValues[index][BATTERY_CHARGE_CURRENT] = modbusClient.read();
DBG_SECT(
                                rprintD("Battery Charge Current: ");
                                rprintDln(chargerValues[index][BATTERY_CHARGE_CURRENT]);
)
                                low = modbusClient.read();
                                high = modbusClient.read();
                                chargerValues[index][BATTERY_CHARGE_POWER] = MK_32(modbusClient.read(), modbusClient.read());
DBG_SECT(
                                rprintD("PV Power: ");
                                rprintDln(chargerValues[index][PV_POWER]);
)
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
                                rprintDln(chargerValues[index][PV_POWER]);
)
                                returnVal = true;
                        } else {
                                DEBUG_E("Reading charger PV/Bat failed with %s\n", modbusClient.lastError());
                                vTaskDelay(pdMS_TO_TICKS(200));
                        }
                        
                } while (!returnVal && ++count < 3);
        }
        return returnVal;
}

static bool readTemps(uint index)
{
        bool returnVal = false;
        if (xSemaphoreTake(gSerial2Mutex, pdMS_TO_TICKS(gChargerUpdateIntervalMilis / 3 * NUM_CHARGERS)) == pdTRUE)
        {
                if (Serial2.baudRate() != BAUDRATE)
                {
                        Serial2.updateBaudRate(BAUDRATE);
                }

                bool result = modbusClient.requestFrom(chargerModbusAdresses[index], INPUT_REGISTERS, 0x3110, 3);                        
                                
                if (result && modbusClient.available() >= 3)
                {
                        chargerValues[index][BATTERY_TEMP] = modbusClient.read();
DBG_SECT(
                        rprintD("Battery Temp: ");
                        rprintDln(chargerValues[index][BATTERY_TEMP]);
)
                        chargerValues[index][CONTROLLER_INTERNAL_TEMP] = modbusClient.read();
DBG_SECT(
                        rprintD("Controller internal Temp: ");
                        rprintDln(chargerValues[index][CONTROLLER_INTERNAL_TEMP]);
)
                        chargerValues[index][CONTROLLER_INTERNAL_TEMP] = modbusClient.read();
DBG_SECT(
                        rprintD("Controller Power component Temp: ");
                        rprintDln(chargerValues[index][CONTROLLER_POWER_COMP_TEMP]);
)
                        returnVal = true;
                }
                DBG_SECT( else {DEBUG_E("Reading charger TMP failed with %s\n", modbusClient.lastError());})                
                result = modbusClient.requestFrom(chargerModbusAdresses[index], INPUT_REGISTERS, 0x3110, 3);
                xSemaphoreGive(gSerial2Mutex);
                if (result && modbusClient.available() >= 3)
                {

                        chargerValues[index][BATTERY_EXTERNAL_TEMP] = modbusClient.read();
DBG_SECT(
                        rprintD("Battery external Temp: ");
                        rprintDln(chargerValues[index][BATTERY_EXTERNAL_TEMP]);
)
                        returnVal = true;
                }
                 DBG_SECT( else {DEBUG_E("Reading charger external TMP failed with %s\n", modbusClient.lastError());})
        }
        return returnVal;
}

static bool readStates(uint index)
{
        bool returnVal = false;
        if (xSemaphoreTake(gSerial2Mutex, pdMS_TO_TICKS(gChargerUpdateIntervalMilis / 3 * NUM_CHARGERS)) == pdTRUE)
        {
                if (Serial2.baudRate() != BAUDRATE)
                {
                        Serial2.updateBaudRate(BAUDRATE);
                }

                bool result = modbusClient.requestFrom(chargerModbusAdresses[index], INPUT_REGISTERS,0x3200, 2);
                xSemaphoreGive(gSerial2Mutex);
                if (result && modbusClient.available() >= 2)
                {
                        chargerValues[index][BATTERY_STATUS] = modbusClient.read();
                        chargerValues[index][CONTROLLER_STATUS] = modbusClient.read();
                        returnVal = true;
                } 
                 DBG_SECT( else {DEBUG_E("Reading charger states failed with %s\n", modbusClient.lastError());})
        }
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

void updateController(uint index)
{       
        bool result = false;
        bool newResult = false;
        unsigned int cnt;
        static int iteration = 0;

        DEBUG_D("Updating controller %d",index+1);
               
        vTaskDelay(pdMS_TO_TICKS(100));
        result = chargerReadPvAndBattery(index);
        unsigned int value = chargerValues[index][CHARGER_READ_ERROR];
        cnt = value >> 2*8;
        cnt = result ? 0 : cnt+1;
        chargerValues[index][CHARGER_READ_ERROR] = (value & 0x00FFFF) | (cnt <<2*8);

        if (!iteration)
        {
                // Read only each 3rd time
                vTaskDelay(pdMS_TO_TICKS(100));
                newResult = readTemps(index);
                value = chargerValues[index][CHARGER_READ_ERROR];
                cnt = (value >> 1 * 8) & 0xFF;
                cnt = newResult ? 0 : cnt + 1;
                chargerValues[index][CHARGER_READ_ERROR] = (value & 0xFF00FF) | (cnt << 1 * 8);
                result = newResult || result;
        }
        else
        {
                if (iteration == 1)
                {
                        vTaskDelay(pdMS_TO_TICKS(100));
                        newResult = readStates(index) || result;
                        value = chargerValues[index][CHARGER_READ_ERROR];
                        cnt = value & 0xFF;
                        cnt = newResult ? 0 : cnt + 1;
                        chargerValues[index][CHARGER_READ_ERROR] = (value & 0xFFFF00) | cnt;
                        result = newResult || result;
                }
        }
        iteration = (iteration+1) % 3;
        gChargerValuesChanged[index] = result;
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
        chargeControllerSetupController();
        TickType_t previousTime = xTaskGetTickCount();
        while (true)
        {
                vTaskDelayUntil(&previousTime, pdMS_TO_TICKS(gChargerUpdateIntervalMilis - 100));

                for (int i = 0; i < NUM_CHARGERS; ++i)
                {
                        if (chargerValid(i))
                        {                                
                                updateController(i);
                        }
                }
                gCurrentPowerCreated = calculateInstantPower();
                if (gExcessTaskId)
                {
                        // Let's re-calculate the excess values...
                        xTaskNotifyGive(gExcessTaskId);
                }
        }
}



void chargeControllerSetup()
{
        TaskHandle_t taskId;

        gSerial2Mutex = xSemaphoreCreateMutexStatic(&xSemaphoreBuffer);
        gChargerNumValidChargers = calculateChargerIds();
        setUpdateMillis();   
        modbusClient.begin(BAUDRATE);    
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
