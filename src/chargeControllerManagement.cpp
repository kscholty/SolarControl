
#include <ModbusMaster.h>
#include "chargeControllerManagement.h"


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


unsigned int chargerValues[NUM_CHARGERS][NUM_CHARGER_VALUES];

bool chargerIsValid(int index) {
        return chargerValid(index);
}

static unsigned int calculateChargerIds() {
        unsigned int count = 0;
        for (int i = 0; i < NUM_CHARGERS; ++i)
        {
                chargerModbusAdresses[i] = atoi(gChargerModbusAdressesValue[i]);
                gChargerValuesChanged[i] = false;
                if (chargerValid(i))
                {
                        charger[i].begin(chargerModbusAdresses[i], Serial2);
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

static bool readPvAndBattery(uint index)
{
        uint8_t result = charger[index].readInputRegisters(0x3100, 8);
        bool returnVal = false;

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
                chargerValues[index][BATTERY_CHARGE_VOLTAGE] = chargerValues[index][BATTERY_CHARGE_POWER] / chargerValues[index][BATTERY_CHARGE_CURRENT];
#if DEBUG
                Serial.print("Battery Charge Voltage: ");
                Serial.println(chargerValues[index][PV_POWER]);
#endif
                returnVal = true;
        }
        return returnVal;
}

static bool readTemps(uint index)
{
        uint8_t result = charger[index].readInputRegisters(0x3110, 3);
        bool returnVal = false;

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

        result = charger[index].readInputRegisters(0x311B, 1);

        if (result == charger[index].ku8MBSuccess)
        {
      
                chargerValues[index][BATTERY_EXTERNAL_TEMP] = charger[index].getResponseBuffer(0x00);
#if DEBUG
                Serial.print("Battery external Temp: ");
                Serial.println(chargerValues[index][BATTERY_EXTERNAL_TEMP]);
#endif
                returnVal = true;
        }
        return returnVal;
}

static bool readStates(uint index) {
        bool returnVal = false;
        uint8_t result = charger[index].readInputRegisters(0x3200, 2);

        if (result == charger[index].ku8MBSuccess)
        {
                chargerValues[index][BATTERY_STATUS] = charger[index].getResponseBuffer(0x00);
                chargerValues[index][CONTROLLER_STATUS] = charger[index].getResponseBuffer(0x01);
                returnVal = true;
        }
        return returnVal;
}

void updateController(uint index)
{       
        bool result = false;
#if DEBUG
        Serial.print("Updating controller ");
        Serial.println(index + 1);
#endif
        result = readPvAndBattery(index) || result;
        result = readTemps(index) || result;
        result = readStates(index) || result;
        gChargerValuesChanged[index] = result;
}

void chargeControllerThradFunc(void *)
{
        if (gChargerNumValidChargers == 0)
        {
                // That is an error and shouldn't happen!
                vTaskDelete(xTaskGetCurrentTaskHandle());
        }        
        chargeControllerSetupController();
        TickType_t previousTime = xTaskGetTickCount();
        while (true)
        {
                vTaskDelayUntil(&previousTime, pdMS_TO_TICKS(gChargerUpdateIntervalMilis-100));
                for(int i = 0; i<NUM_CHARGERS;++i) {
                        if(chargerValid(i)) {
                                vTaskDelay(pdMS_TO_TICKS(100));
                                updateController(i);
                        }
                }
                
        }
}

void chargeControllerSetup()
{
        TaskHandle_t taskId;
        gChargerNumValidChargers = calculateChargerIds();
        gChargerUpdateIntervalMilis = atol(gChargerUpdateIntervalValue) * 1000;
        if (gChargerUpdateIntervalMilis < 10000)
        {
                gChargerUpdateIntervalMilis = 10000;
        }
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
