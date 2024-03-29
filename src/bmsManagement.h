#pragma once

#include "common.h"
#include "Battery.h"


enum BMS_TYPE {
  NONE = 0,
  JBD,
  JK,
  SEPLOS,
  NUM_BMS_TYPES
};

extern const char gBmsNames[NUM_BMS_TYPES][STRING_LEN];
extern char gBmsUpdateIntervalValue[NUMBER_LEN];
static const int bmsCount = 3;


extern BMS::CBmsBase* gBms[bmsCount];
extern char gBmsType[bmsCount][STRING_LEN];

#pragma pack(push,1)

struct Alarms_t
{
    int Lowcapacity : 1;
    int BmsOverTemperature : 1;
    int ChargingOvervoltage : 1;
    int DischargingUndervoltage : 1;
    int BatteryOverTemperature : 1;
    int ChargingOvercurrent : 1;
    int DischargingOvercurrent : 1;
    int CellPressureDifference : 1;
    int OvertemperatureAlarmBatteryBox : 1;
    int BatteryLowTemperature : 1;
    int CellOvervoltage : 1;
    int CellUndervoltage : 1;
    int Protection309_A : 1;
    int Protection309_B : 1;
    int Reserved : 2;
    int Reserved2 : 1;
};

union AlarmStorage_t {
    Alarms_t alarms;
    uint16_t rawValue; 
};

struct BatteryStatus_t
{
    int ChargingEnabled : 1;
    int DischargingEnabled : 1;
    int BalancerEnabled : 1;
    int BatteryDropped : 1;
    int reseved : 12;
};

union BatteryStatusStorage_t {
    BatteryStatus_t status;
    uint16_t rawValue;
};
struct BmsBasicInfo_t
{
    public:
    uint16_t getTotalVoltage() {return totalVoltage;} // unit 10mV
	int16_t getcurrent() {return current;}   // unit 10mA
	uint16_t getcapacityRemain() {return capacityRemain;} // unit 10mAh
    uint16_t getnominalCapacity() {return nominalCapacity;} // unit 10mAh
	uint16_t getcycleLife() {return cycleLife;} 
    uint16_t getproductDate() {return productDate;}
    uint16_t getfetControlStatus() { return (0x0 | (batteryStatus.status.DischargingEnabled<<1) | batteryStatus.status.ChargingEnabled );}
    Alarms_t getprotectionStatus() {return alarmsStatus.alarms;}
    BatteryStatus_t getStatus() { return batteryStatus.status;}
    uint8_t getversion() {return version;}
    uint8_t getstateOfCharge(){ return stateOfCharge;} // in percent    
    uint8_t getcellsInSeries() {return cellsInSeries;}
    uint8_t getnumTempSensors() {return numTempSensors;}
    int16_t getTemp(unsigned int i) {
        if(i<numTempSensors) {
            return temps[i];
        } else { return -2731;} 
    }

    size_t size() { return sizeof(*this);}
    
	uint16_t totalVoltage; // unit 10mV
	int32_t current;   // unit 10mA
	uint16_t capacityRemain; // unit 10mAh
    uint16_t nominalCapacity; // unit 10mAh
	uint16_t cycleLife; 
    uint16_t productDate;    
    AlarmStorage_t alarmsStatus;
    uint8_t version;
    uint8_t stateOfCharge; // in percent
    BatteryStatusStorage_t batteryStatus;
    uint8_t cellsInSeries;
    uint8_t numTempSensors;
    int16_t temps[3];	
} ;

struct BmsCellInfo_t
{
    
    public:
    uint8_t getNumOfCells() {return numCells;}
    uint16_t getCellVolt(size_t i) { if (i<getNumOfCells()) { return cellVolt[i];} else { return 0;} }
    void setVoltage(size_t i, uint16_t value) { if(i<numCells) cellVolt[i] = value; }
    void setNumCells(size_t count) { numCells = count;}
    private:
	uint8_t numCells;
	uint16_t cellVolt[0]; // This in mV, hence devide it by 1000
} ;

#pragma pack(pop)

extern bool gBmsDisconnect;
extern bool gBmsUpdated;


extern void bmsSetup();
extern void bmsEnable(bool on);