
#include <WiFi.h>
#include <ModbusTCP.h>
#include "common.h"
#include "debugManagement.h"
#include "modbusManagement.h"
#include "excessControlManagement.h"
#include "inverterManagement.h"
#include "bmsManagement.h"


 
static ModbusTCP *modbusServer = 0;

static bool needsReconnect = true;
enum REGISTERS {
  REG_BATTERY_VOLTAGE=0,
  REG_BATTERY_CURRENT=1,
  REG_BATTERY_SOC = 2,
  REG_CHARGER_POWER = 3,
  REG_INVERTER_POWER = 4,
  REG_NUM_REGISTERS
};

//The Holding registers are:
// 0: Battery Voltage. Unit: 10mV
// 1: Battery Current . Unit 10mA
// 2: SOC. Unit Percent
// 3: Charge power. Unit: 100mW
// 4: InverterPower. Unit: 100mW

uint16_t getter(TRegister *reg, uint16_t val)
{
  REGISTERS regNum = (REGISTERS)(reg->address.address);

  switch (regNum)
  {
  case REG_BATTERY_VOLTAGE:
    if(gBmsBasicInfo)  return gBmsBasicInfo->getTotalVoltage();
    break;
  case REG_BATTERY_CURRENT:
    if(gBmsBasicInfo)  return gBmsBasicInfo->getcurrent();
    break;
  case REG_BATTERY_SOC:
    if(gBmsBasicInfo)  return gBmsBasicInfo->getstateOfCharge();
    break;
  case REG_CHARGER_POWER:
    return gCurrentPowerCreated * 10;
    break;
  case REG_INVERTER_POWER:
    return (int16_t)(gInverterPower * 10);
    break;
  default:
    return UINT16_MAX;
  }
  return UINT16_MAX;
}

bool onConnect(IPAddress addr) {  
  DEBUG_I("Client connected from %s\n",addr.toString().c_str());
  return true;
}

bool onDisConnect(IPAddress addr) {
  DEBUG_I("Client disconnected  %s\n",addr.toString().c_str());
  return true;
}


void modbusSetup()
{
  needsReconnect = true;
  
}

void modbusReconnect()
{
  DEBUG_I("Connecting modbusServer");
  if (modbusServer)
  {
    delete modbusServer;
  }
  modbusServer = new ModbusTCP;
  //Config Modbus IP
  modbusServer->server(502);
  modbusServer->cbEnable(true);
  modbusServer->addIreg(0, 0, REG_NUM_REGISTERS);
  modbusServer->onGet(IREG(0), getter, REG_NUM_REGISTERS);
  modbusServer->onConnect(onConnect);
  modbusServer->onDisconnect(onDisConnect);

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
    // poll for Modbus TCP requests, while client connected
    modbusServer->task();
  }
  else
  {
    needsReconnect = true;
  }
}
