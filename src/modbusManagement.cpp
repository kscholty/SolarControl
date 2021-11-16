
#include <WiFi.h>
#include <WiFiServer.h>
#include "common.h"
#include "debugManagement.h"
#include "modbusManagement.h"
#include "ModbusTCPServer.h"
#include "excessControlManagement.h"
#include "inverterManagement.h"
#include "bmsManagement.h"

static WiFiServer wifiServer(502);
static ModbusTCPServer modbusServer;

static bool needsReconnect = true;

//The Holding registers are:
// 0: Battery Voltage. Unit: 10mV
// 1: Battery Current . Unit 10mA
// 2: SOC. Unit Percent
// 3: Charge power. Unit: 100mW
// 4: InverterPower. Unit: 100mW

void modbusUpdateChargerValues() {
  modbusServer.holdingRegisterWrite(3, gCurrentPowerCreated*10);
}

void modbusUpdateInverterValues() {
  modbusServer.holdingRegisterWrite(4,(int16_t)(gInverterPower*10));
}

void modbusUpdateBMSValues() {
  modbusServer.holdingRegisterWrite(0,gBmsBasicInfo->getTotalVoltage());
  modbusServer.holdingRegisterWrite(1,gBmsBasicInfo->getcurrent());
  modbusServer.holdingRegisterWrite(2,gBmsBasicInfo->getstateOfCharge());
}

void modbusSetup() {
  needsReconnect = true;
  modbusServer.configureHoldingRegisters(0,5);
}

void modbusReconnect()
{
    DEBUG_I("Connecting modbusServer");
    wifiServer.end();
    wifiServer.begin();
    modbusServer.begin();    
    needsReconnect = false;
}

void modbusLoop(unsigned long)
{

  if (WiFi.isConnected())
  {
    if (needsReconnect)
    {
      modbusReconnect();
    }
    // listen for incoming clients
    WiFiClient client = wifiServer.available();

    if (client)
    {
      DEBUG_I("Modbus client connected");
      // let the Modbus TCP accept the connection
      modbusServer.accept(client);

      while (client.connected())
      {
        // poll for Modbus TCP requests, while client connected
        modbusServer.poll();
      }
      DEBUG_I("Modbus client disconnected");
    }
  }
  else
  {
    needsReconnect = true;
  }
}
