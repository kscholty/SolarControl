#pragma once

#include "common.h"

extern char gBmsDummyValue[STRING_LEN];
extern char gBmsUpdateIntervalValue[NUMBER_LEN];

#pragma pack(push,1)
struct BmsBasicInfo_t
{
    public:
    uint16_t getTotalVoltage() {return __builtin_bswap16(totalVoltage);} // unit 10mV
	int16_t getcurrent() {return __builtin_bswap16(current);}   // unit 10mA
	uint16_t getcapacityRemain() {return __builtin_bswap16(capacityRemain);} // unit 10mAh
    uint16_t getnominalCapacity() {return __builtin_bswap16(nominalCapacity);} // unit 10mAh
	uint16_t getcycleLife() {return __builtin_bswap16(cycleLife);} 
    uint16_t getproductDate() {return __builtin_bswap16(productDate);}
    uint16_t getbalanceStatusLow() {return __builtin_bswap16(balanceStatusLow);}
    uint16_t getbalanceStatusHigh() {return __builtin_bswap16(balanceStatusHigh);}
    uint16_t getprotectionStatus() {return __builtin_bswap16(protectionStatus);}
    uint8_t getversion() {return version;}
    uint8_t getstateOfCharge(){ return stateOfCharge;} // in percent
    uint8_t getfetControlStatus() {return fetControlStatus;}
    uint8_t getcellsInSeries() {return cellsInSeries;}
    uint8_t getnumTempSensors() {return numTempSensors;}
    int16_t getTemp(unsigned int i) {
        if(i<numTempSensors) {
            return __builtin_bswap16(temps[i])-2731;
        } else { return -2731;} 
    }

    size_t size() { return sizeof(*this)+getnumTempSensors()*2;}
    
    private:
	uint16_t totalVoltage; // unit 10mV
	int16_t current;   // unit 10mA
	uint16_t capacityRemain; // unit 10mAh
    uint16_t nominalCapacity; // unit 10mAh
	uint16_t cycleLife; 
    uint16_t productDate;
    uint16_t balanceStatusLow;
    uint16_t balanceStatusHigh;
    uint16_t protectionStatus;
    uint8_t version;
    uint8_t stateOfCharge; // in percent
    uint8_t fetControlStatus;
    uint8_t cellsInSeries;
    uint8_t numTempSensors;
    uint16_t temps[0];	// Its in 0,1 Kelvin (calculate (value-2731)/10 = deg C)
} ;

struct BmsCellInfo_t
{
    public:
    uint8_t getNumOfCells() {return numOfCells>>1;}
    uint16_t getCellVolt(unsigned int i) { if (i<numOfCells) {return __builtin_bswap16(cellVolt[i]);} else { return 0;} }
    private:
	uint8_t numOfCells;
	uint16_t cellVolt[0]; // This in mV, hence devide it by 1000
} ;

#pragma pack(pop)

extern BmsBasicInfo_t *gBmsBasicInfo;
extern BmsCellInfo_t *gBmsCellInfo;

extern bool gBmsDisconnect;
extern bool gBmsUpdated;


extern void bmsSetup();
extern void bmsEnable(bool on);