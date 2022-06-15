
#include <WiFi.h>
#include <ModbusTCP.h>
#include "common.h"
#include "debugManagement.h"
#include "modbusManagement.h"
#include "excessControlManagement.h"
#include "inverterManagement.h"
#include "bmsManagement.h"
#include "adc.h"


 
static ModbusTCP *modbusServer = 0;

static bool needsReconnect = true;
enum IREGISTERS {
  REG_BATTERY_VOLTAGE=0,
  REG_BATTERY_CURRENT=1,
  REG_BATTERY_SOC = 2,
  REG_CHARGER_POWER = 3,
  REG_INVERTER_POWER = 4,
  REG_NUM_IREGISTERS
};

enum HREGISTERS {
  VOLTAGE_OFFSET = 0,
  PID_P = 1,
  PID_I = 2,
  PID_D = 3,
  INVFACTOR = 4,
  DEMANDPOWER = 5,
  MEASUREDPOWER = 6,
  INVTARGET = 7,
  STOPCTRL = 8,
  REG_NUM_HREGISTERS
};

//The Holding registers are:
// 0: Battery Voltage. Unit: 10mV
// 1: Battery Current . Unit 10mA
// 2: SOC. Unit Percent
// 3: Charge power. Unit: 100mW
// 4: InverterPower. Unit: 100mW

uint16_t getterI(TRegister *reg, uint16_t val)
{
  IREGISTERS regNum = (IREGISTERS)(reg->address.address);

  switch (regNum)
  {
  case REG_BATTERY_VOLTAGE:
    if(gBms)  return gBms[0]->basicInfo().getTotalVoltage();
    break;
  case REG_BATTERY_CURRENT:
    if(gBms)  return gBms[0]->basicInfo().getcurrent();
    break;
  case REG_BATTERY_SOC:
    if(gBms)  return gBms[0]->basicInfo().getstateOfCharge();
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

uint16_t getterH(TRegister *reg, uint16_t val)
{
  HREGISTERS regNum = (HREGISTERS)(reg->address.address);

  switch (regNum) {
    case VOLTAGE_OFFSET:
      return gVoltageOffset;
      break;
    case PID_P:
      return (uint16_t)(Kp * 1000);
      break;
    case PID_I:
      return (uint16_t)(Ki * 1000);
      break;
    case PID_D:
      return (uint16_t)(Kd * 1000);
      break;
    case INVFACTOR:
      return gVoltageFactor;
      break;
    case DEMANDPOWER:
      return (int16_t)gGridSumValues[ValuePower];
      break;
    case MEASUREDPOWER:
      return (int16_t)gInverterPower;
      break;
    case INVTARGET:
      return (int16_t) gInverterTarget;
      break;
      case STOPCTRL:
       return inverterLocked();
      break;
      
    default:
      return UINT16_MAX;
  }
  return UINT16_MAX;
}

uint16_t setterH(TRegister *reg, uint16_t val)
{

  HREGISTERS regNum = (HREGISTERS)(reg->address.address);

  DEBUG_I("Client writes %d to %d\n", val,reg->address.address);

  switch (regNum) {
    case VOLTAGE_OFFSET:
      gVoltageOffset = val;
      return gVoltageOffset;
      break;
    case PID_P:
      Kp = val/1000.0;
      return (uint16_t)(Kp * 1000);
      break;
    case PID_I:
      Ki = val/1000.0;
      return (uint16_t)(Ki * 1000);
      break;
    case PID_D:
      Kd = val/1000.0;
      return (uint16_t)(Kd * 1000);
      break;
    case INVFACTOR:
      gVoltageFactor = val;
      return gVoltageFactor;
      break;
    case DEMANDPOWER:
      return (int16_t)gGridSumValues[ValuePower];
      break;
    case MEASUREDPOWER:
      return (int16_t)gInverterPower;
      break;
    case INVTARGET:
      return (int16_t) gInverterTarget;
      break;
    case STOPCTRL:
      if(val != 0) {
        inverterLock();
      } else {
        inverterUnlock();
      }
      return inverterLocked();
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
  modbusServer->addIreg(0, 0, REG_NUM_IREGISTERS);
  modbusServer->addHreg(0, 0, REG_NUM_HREGISTERS);
  modbusServer->onGet(IREG(0), getterI, REG_NUM_IREGISTERS);
  modbusServer->onGet(HREG(0), getterH, REG_NUM_HREGISTERS);
  modbusServer->onSet(HREG(0), setterH, REG_NUM_HREGISTERS);
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
