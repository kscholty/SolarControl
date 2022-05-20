#include <WiFi.h>
#include <netdb.h>
#include <ModbusTCP.h>

static const int CONSUMPTION_REGS_OFFSET = 69;               // Modbus Hreg Offset
static const int NUMCREGS = 6;
static uint16_t consumptionRegisters[NUMCREGS];

static const int GRID_REGS_OFFSET = 105;               // Modbus Hreg Offset
static const int NUMGREGS = 3;
static uint16_t gridRegisters[NUMGREGS];



ModbusTCP mb;  //ModbusTCP object
IPAddress S10Address;

#include "debugManagement.h"
#include "modbusClientManagement.h"
#include "inverterManagement.h"

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

    if(!isInit) {
        isInit = resolveHostname(S10Name);          
        if(isInit) {
          mb.client();
          mb.connect(S10Address,502);  // Try to connect if no connection  
        }
    }
}

void modbusClientLoop(void *) {
  static unsigned long counter = 0;
  static const unsigned long RESOLVEDELAY = (1000 / delayMs * 60 * 15);
  uint16_t transG= 0;
  TickType_t previousTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&previousTime, pdMS_TO_TICKS(delayMs));
    if (!isInit) {
      modbusClientInit();
    } else {
      if (mb.isConnected(S10Address)) {  // Check if connection to Modbus Slave
                                         // is established
        uint16_t transC =
            mb.readHreg(S10Address, CONSUMPTION_REGS_OFFSET, consumptionRegisters,
                        NUMCREGS);  // Initiate Read Hreg from Modbus Server
            transG = mb.readHreg(S10Address, GRID_REGS_OFFSET, gridRegisters,
                        NUMGREGS); // Initiate Reading Grid Counter
        while (mb.isTransaction(transC)) {  // Check if transaction is active          
          vTaskDelay(pdMS_TO_TICKS(5));
          mb.task();
        }
        
        // At this point res is filled with response value
        bool powerUpdated = false;
        //Serial.printf("0x%x 0x%x  0x%x 0x%x  0x%x 0x%x\n",registers[0],registers[1],registers[2],registers[3],registers[4],registers[5]);
        int32_t batteryPower = (uint32_t)(consumptionRegisters[1]) << 16 | consumptionRegisters[0];
        int32_t gridPower = (uint32_t)(consumptionRegisters[5]) << 16 | consumptionRegisters[4];
        //Serial.printf("Battery: %d, Grid %d\n", batteryPower,gridPower);

        gGridSumValues[ValuePower] = gridPower - batteryPower;        
        gInverterGridPowerUpdated();

        while (mb.isTransaction(transG)) {  // Check if transaction is active
          vTaskDelay(pdMS_TO_TICKS(5));
          mb.task();         
        }
        gGridLegValues[ValuePower][0] = (int16_t)gridRegisters[0];
        gGridLegValues[ValuePower][1] = (int16_t)gridRegisters[1];
        gGridLegValues[ValuePower][2] = (int16_t)gridRegisters[2];

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
    BaseType_t result = xTaskCreate(modbusClientLoop, "shelly", 4096, 0, 2, &handle);
    if (result != pdPASS)
    {
        if (Debug.isActive(Debug.ERROR))
        {
            Debug.print(" Shelly taskCreation failed with error ");
            Debug.println(result);
        }
        gInverterTaskHandle = 0;
    }
}