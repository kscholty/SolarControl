
#include <ModbusMaster.h>
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
static ModbusMaster charger[NUM_CHARGERS];
unsigned long gChargerUpdateIntervalMilis = 15000;
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
                if (chargerValid(i))
                {
                        charger[i].begin(chargerModbusAdresses[i], Serial2);
                }
                memset(chargerValues[i], 0, NUM_CHARGER_VALUES * sizeof(chargerValues[0][0]));
        }
}

bool chargerReadPvAndBattery(uint index)
{
        bool returnVal = false;
        if (xSemaphoreTake(gSerial2Mutex, pdMS_TO_TICKS(gChargerUpdateIntervalMilis / 3 * NUM_CHARGERS)) == pdTRUE)
        {
                uint8_t result;
                uint count = 0;
                do
                {
                        result = charger[index].readInputRegisters(0x3100, 8);
                        xSemaphoreGive(gSerial2Mutex);
                        if (result == charger[index].ku8MBSuccess)
                        {

                                chargerValues[index][PV_VOLTAGE] = charger[index].getResponseBuffer(0x00);
#if DEBUG
                                Serial.print("PV Voltage: ");
                                Serial.println(chargerValues[index][PV_VOLTAGE]);
#endif
                                chargerValues[index][PV_CURRENT] = charger[index].getResponseBuffer(0x01);
#if DEBUG
                                Serial.print("PV Current: ");
                                Serial.println(chargerValues[index][PV_CURRENT]);
#endif
                                chargerValues[index][PV_POWER] = MK_32(charger[index].getResponseBuffer(0x02), charger[index].getResponseBuffer(0x03));
#if DEBUG
                                Serial.print("PV Power: ");
                                Serial.println(chargerValues[index][PV_POWER]);
#endif
                                chargerValues[index][BATTERY_VOLTAGE] = charger[index].getResponseBuffer(0x04);
#if DEBUG
                                Serial.print("Battery Voltage: ");
                                Serial.println(chargerValues[index][BATTERY_VOLTAGE]);
#endif
                                chargerValues[index][BATTERY_CHARGE_CURRENT] = charger[index].getResponseBuffer(0x05);
#if DEBUG
                                Serial.print("Battery Charge Current: ");
                                Serial.println(chargerValues[index][BATTERY_CHARGE_CURRENT]);
#endif
                                chargerValues[index][BATTERY_CHARGE_POWER] = MK_32(charger[index].getResponseBuffer(0x06), charger[index].getResponseBuffer(0x07));
#if DEBUG
                                Serial.print("PV Power: ");
                                Serial.println(chargerValues[index][PV_POWER]);
#endif
                                if (chargerValues[index][BATTERY_CHARGE_CURRENT] > 0)
                                {
                                        chargerValues[index][BATTERY_CHARGE_VOLTAGE] = chargerValues[index][BATTERY_CHARGE_POWER] * 100 / chargerValues[index][BATTERY_CHARGE_CURRENT];
                                }
                                else
                                {
                                        chargerValues[index][BATTERY_CHARGE_VOLTAGE] = 0;
                                }
#if DEBUG
                                Serial.print("Battery Charge Voltage: ");
                                Serial.println(chargerValues[index][PV_POWER]);
#endif
                                returnVal = true;
                        } else {
                                vTaskDelay(pdMS_TO_TICKS(200));
                        }
                } while (!returnVal  && ++count < 3);
        }
        return returnVal;
}

static bool readTemps(uint index)
{
        bool returnVal = false;
        if (xSemaphoreTake(gSerial2Mutex, pdMS_TO_TICKS(gChargerUpdateIntervalMilis / 3 * NUM_CHARGERS)) == pdTRUE)
        {
                uint8_t result = charger[index].readInputRegisters(0x3110, 3);
                
                if (result == charger[index].ku8MBSuccess)
                {
                        chargerValues[index][BATTERY_TEMP] = charger[index].getResponseBuffer(0x00);
#if DEBUG
                        Serial.print("Battery Temp: ");
                        Serial.println(chargerValues[index][BATTERY_TEMP]);
#endif
                        chargerValues[index][CONTROLLER_INTERNAL_TEMP] = charger[index].getResponseBuffer(0x01);
#if DEBUG
                        Serial.print("Controller internal Temp: ");
                        Serial.println(chargerValues[index][CONTROLLER_INTERNAL_TEMP]);
#endif
                        chargerValues[index][CONTROLLER_INTERNAL_TEMP] = charger[index].getResponseBuffer(0x02);
#if DEBUG
                        Serial.print("Controller Power component Temp: ");
                        Serial.println(chargerValues[index][CONTROLLER_POWER_COMP_TEMP]);
#endif
                        returnVal = true;
                }
                vTaskDelay(pdMS_TO_TICKS(200));
                result = charger[index].readInputRegisters(0x311B, 1);
                xSemaphoreGive(gSerial2Mutex);
                if (result == charger[index].ku8MBSuccess)
                {

                        chargerValues[index][BATTERY_EXTERNAL_TEMP] = charger[index].getResponseBuffer(0x00);
#if DEBUG
                        Serial.print("Battery external Temp: ");
                        Serial.println(chargerValues[index][BATTERY_EXTERNAL_TEMP]);
#endif
                        returnVal = true;
                }
        }
        return returnVal;
}

static bool readStates(uint index)
{
        bool returnVal = false;
        if (xSemaphoreTake(gSerial2Mutex, pdMS_TO_TICKS(gChargerUpdateIntervalMilis / 3 * NUM_CHARGERS)) == pdTRUE)
        {
                uint8_t result = charger[index].readInputRegisters(0x3200, 2);
                xSemaphoreGive(gSerial2Mutex);
                if (result == charger[index].ku8MBSuccess)
                {
                        chargerValues[index][BATTERY_STATUS] = charger[index].getResponseBuffer(0x00);
                        chargerValues[index][CONTROLLER_STATUS] = charger[index].getResponseBuffer(0x01);
                        returnVal = true;
                }
        }
        return returnVal;
}

void updateController(uint index)
{       
        bool result = false;
        bool newResult = false;
        unsigned int cnt;
#if DEBUG
        Serial.print("Updating controller ");
        Serial.println(index + 1);
#endif
        
        vTaskDelay(pdMS_TO_TICKS(200));
        result = chargerReadPvAndBattery(index);
        unsigned int value = chargerValues[index][CHARGER_READ_ERROR];
        cnt = value >> 2*8;
        cnt = result ? 0 : cnt+1;
        chargerValues[index][CHARGER_READ_ERROR] = (value & 0x00FFFF) | (cnt <<2*8);
        
        vTaskDelay(pdMS_TO_TICKS(200));
        newResult = readTemps(index) ;
        value = chargerValues[index][CHARGER_READ_ERROR];  
        cnt = (value >> 1*8 ) & 0xFF;
        cnt = newResult ? 0 : cnt+1;      
        chargerValues[index][CHARGER_READ_ERROR] = (value & 0xFF00FF) | (cnt <<1*8);
        result = newResult || result;

        vTaskDelay(pdMS_TO_TICKS(200));
        newResult = readStates(index) || result;                
        value = chargerValues[index][CHARGER_READ_ERROR];
        cnt = value & 0xFF;
        cnt = newResult ? 0 : cnt+1;              
        chargerValues[index][CHARGER_READ_ERROR] = (value & 0xFFFF00) | cnt;

        result = newResult || result;
        gChargerValuesChanged[index] = result;
}

void chargeControllerThradFunc(void *)
{
        if (gChargerNumValidChargers == 0)
        {
#if DEBUG
                Serial.println("Charge controller: No valid controllers");

#endif
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
        
        gSerial2Mutex = xSemaphoreCreateMutexStatic( &xSemaphoreBuffer );
        gChargerNumValidChargers = calculateChargerIds();
        gChargerUpdateIntervalMilis = atol(gChargerUpdateIntervalValue) * 1000;
        if (gChargerUpdateIntervalMilis < 5000)
        {
                gChargerUpdateIntervalMilis = 5000;
        }
        #if DEBUG
        Serial.print("Num controllers : ");
        Serial.println(gChargerNumValidChargers);
#endif
        if (gChargerNumValidChargers > 0)
        {
                BaseType_t result = xTaskCreate(chargeControllerThradFunc, "chrgCntrl", 2048, 0, 1, &taskId);
                if (result != pdPASS)
                {
                        Serial.print(" Charge Controller taskCreation failed with error ");
                        Serial.println(result);
                }
        }
}
