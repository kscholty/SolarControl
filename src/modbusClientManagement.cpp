#include <WiFi.h>
#include <netdb.h>
#include <ModbusTCP.h>
#include "ButterworthLPF.h"
#include "debugManagement.h"
#include "modbusClientManagement.h"
#include "inverterManagement.h"


static const int CONSUMPTION_REGS_OFFSET = 69;               // Modbus Hreg Offset
static const int NUMCREGS = 6;
static uint16_t consumptionRegisters[NUMCREGS];

static const int GRID_REGS_OFFSET = 105;               // Modbus Hreg Offset
static const int NUMGREGS = 3;
static uint16_t gridRegisters[NUMGREGS];

//static ButterworthLPF lpfGrid(2, 20, 100);
//static ButterworthLPF lpfBattery(2, 20, 100);

ModbusTCP mb;  //ModbusTCP object
IPAddress S10Address;


char ginverterShellyValue[STRING_LEN] = "";
char S10Name[STRING_LEN] = "192.168.100.11";

static ShellyDevice shelly3EM;
static bool isInit = false;
static const int delayMs = 500;

bool resolveHostname(char hostname[STRING_LEN]) {
  struct hostent *host;

  if (WiFi.isConnected()) {   
    host = gethostbyname(hostname);
    if (host) {      
      char IP[16];
      S10Address = (uint8_t*)host->h_addr_list[0];
      sprintf(IP,"%d.%d.%d.%d",host->h_addr_list[0][0], 
                               host->h_addr_list[0][1], 
                               host->h_addr_list[0][2],
                               host->h_addr_list[0][3] );
      Serial.printf("Resolved: %s\n",IP);
      DEBUG_I("Resolved IP is %s\n",IP);
      
      return true;
    }
  }
  DEBUG_E("Could not resolve hostname %s\n",hostname);
  return false;
}

void modbusClientInit() {
  if (!isInit) {
    isInit = resolveHostname(S10Name);
    if (isInit) {
      mb.client();
      mb.connect(S10Address, 502);  // Try to connect if no connection
    }
  }
}

static bool modbusGridTransactionCallback(
    Modbus::ResultCode event, uint16_t transactionId,
    void* data) {                     // Modbus Transaction callback
  if (event != Modbus::EX_SUCCESS) {  // If transaction got an error
    debugE("Modbus grid transaction failed: %02X\n",
           event);  // Display Modbus error code
  } else {
    gGridLegValues[ValuePower][0] = (int16_t)gridRegisters[0];
    gGridLegValues[ValuePower][1] = (int16_t)gridRegisters[1];
    gGridLegValues[ValuePower][2] = (int16_t)gridRegisters[2];    
  }
  return true;
}

static bool modbusConsumptionTransactionCallback(
    Modbus::ResultCode event, uint16_t transactionId,
    void* data) {                     // Modbus Transaction callback
  if (event != Modbus::EX_SUCCESS) {  // If transaction got an error
    debugE("Modbus consumption transaction failed: %02X\n",
           event);  // Display Modbus error code
  } else {
    // Serial.printf("0x%x 0x%x  0x%x 0x%x  0x%x
    // 0x%x\n",registers[0],registers[1],registers[2],registers[3],registers[4],registers[5]);
    int32_t batteryPower =
        (uint32_t)(consumptionRegisters[1]) << 16 | consumptionRegisters[0];
    int32_t gridPower =
        (uint32_t)(consumptionRegisters[5]) << 16 | consumptionRegisters[4];
    // Serial.printf("Battery: %d, Grid %d\n", batteryPower,gridPower);


   // int32_t gridPower = lpfGrid.update(gridPowerRaw);
   // int32_t batteryPower = lpfBattery.update(batteryPowerRaw);

    //debugD("gP: %d (%d) bP: %d (%d)\n",gridPower,gridPowerRaw, batteryPower, batteryPowerRaw);

    gGridSumValues[ValuePower] = gridPower - batteryPower;
    gInverterGridPowerUpdated();
  }
  return true;
}

void modbusClientLoop(void*) {
  static unsigned long counter = 0;
  static const unsigned long RESOLVEDELAY = (1000 / delayMs * 60 * 15);
  uint16_t transG = 0;
  TickType_t previousTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&previousTime, pdMS_TO_TICKS(delayMs));
    if (!isInit) {
      modbusClientInit();
    } else {
      if (mb.isConnected(S10Address)) {  // Check if connection to Modbus Slave
                                         // is established        
        uint16_t transG = mb.readHreg(
            S10Address, GRID_REGS_OFFSET, gridRegisters, NUMGREGS,
            modbusGridTransactionCallback);  // Initiate Reading Grid Counter
        uint16_t transC = mb.readHreg(
            S10Address, CONSUMPTION_REGS_OFFSET, consumptionRegisters, NUMCREGS,
            modbusConsumptionTransactionCallback);  // Initiate Read Hreg from
                                                    // Modbus Server   
        do {                                 // Check if transaction is active
          vTaskDelay(pdMS_TO_TICKS(2));
          mb.task();
        } while (mb.isTransaction(transC) || mb.isTransaction(transG));
      } else {
        mb.connect(S10Address, 502);  // Try to connect if no connection
        mb.task();
      }
    }

    if (++counter > RESOLVEDELAY) {
      // Repeat name resolution from time to time if
      // ip changes.
      if (resolveHostname(S10Name)) {
        counter = 0;
      }
    }
  }
}

void modbusClientSetup()
{
    TaskHandle_t handle;
    String name(S10Name);
    name.trim();
    if(!name.length()) {
      Debug.print(" modbusClient no name given.  ");
      return;
    }
    BaseType_t result = xTaskCreate(modbusClientLoop, "shelly", 4096, 0, 2, &handle);
    if (result != pdPASS)
    {
        if (Debug.isActive(Debug.ERROR))
        {
            Debug.print(" modbusClient taskCreation failed with error ");
            Debug.println(result);
        }
        gInverterTaskHandle = 0;
    }
}