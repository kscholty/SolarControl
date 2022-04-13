
#include "common.h"

#if QUCCBMS

#include "debugManagement.h"
#include "quccbms_impl.h"

namespace BMS { 

#define STARTBYTE 0xDD
#define STOPBYTE 0x77

#define MAXMESSAGESIZE 64
#define BAUDRATE 9600



#pragma pack(push,1)
struct QUCCBasicInfo_t
{
    public:
    uint16_t getTotalVoltage() const {return __builtin_bswap16(totalVoltage);} // unit 10mV
	int16_t getcurrent() const {return __builtin_bswap16(current);}   // unit 10mA
	uint16_t getcapacityRemain() const {return __builtin_bswap16(capacityRemain);} // unit 10mAh
    uint16_t getnominalCapacity() const {return __builtin_bswap16(nominalCapacity);} // unit 10mAh
	uint16_t getcycleLife() const {return __builtin_bswap16(cycleLife);} 
    uint16_t getproductDate() const {return __builtin_bswap16(productDate);}
    uint16_t getbalanceStatusLow() const {return __builtin_bswap16(balanceStatusLow);}
    uint16_t getbalanceStatusHigh() const {return __builtin_bswap16(balanceStatusHigh);}
    uint16_t getprotectionStatus() const {return __builtin_bswap16(protectionStatus);}
    uint8_t getversion() const {return version;}
    uint8_t getstateOfCharge() const { return stateOfCharge;} // in percent
    uint8_t getfetControlStatus() const {return fetControlStatus;}
    uint8_t getcellsInSeries() const {return cellsInSeries;}
    uint8_t getnumTempSensors() const {return numTempSensors;}

    int16_t getTemp(unsigned int i) const {
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

struct QUCCCellInfo_t
{
    public:
    uint8_t getNumOfCells() const {return numOfCells>>1;}
    uint16_t getCellVolt(unsigned int i) const { if (i<numOfCells) {return __builtin_bswap16(cellVolt[i]);} else { return 0;} }
    private:
	uint8_t numOfCells;
	uint16_t cellVolt[0]; // This in mV, hence divide it by 1000
} ;

#pragma pack(pop)




bool QUCCBms::setup() {
    return true;
}

 bool QUCCBms::getHighFrequRequest(const uint8_t **request,size_t *length) const {
    static const uint8_t message[] = {0xdd, 0xa5, 0x03, 0x00, 0xff, 0xfd, 0x77};
    *request = message;
    *length = sizeof(message);
    return true;
 }

 bool QUCCBms::getLowFrequRequest(const uint8_t **request,size_t *length) const {
    static const uint8_t message[]  = {0xdd, 0xa5, 0x4, 0x0, 0xff, 0xfc, 0x77};
    *request = message;
    *length = sizeof(message);
    return true;
 }

 size_t QUCCBms::messageHeaderSize() const {
     return sizeof(MessageHeader_t);
 }

size_t QUCCBms::getBodySize(const uint8_t *messagebuffer,size_t length) const {
    const MessageHeader_t *header = (const MessageHeader_t*)(messagebuffer);
    return header->dataLen+3;
}



bool QUCCBms::processMessage(const uint8_t *message, size_t length, BmsBasicInfo_t *basicInfo, BmsCellInfo_t *cellInfo)  {

    bool result = false;
    MessageHeader_t *header = (MessageHeader_t*)message;

    if(header->start != STARTBYTE || header->status != 0 || length != header->dataLen+7) {
        
        DEBUG_W("BMS: Received invalid message\n");
        
        return false;
    }

    switch(header->command) {
        case 0x03: result = parseBasicInfo(header,length,basicInfo);//Its basic info
        break;
        case 0x04: result = parseCellInfo(header,length,cellInfo); // Its Cell info
        break;
        default:
            debugW("BMS: Got unknown answer message with type %d\n",(int)header->command);            
            break;
    }

    return result;
}

bool QUCCBms::parseCellInfo(const MessageHeader_t *data, size_t length, BmsCellInfo_t *cellInfo) const {
    const MessageHeader_t *header = (const MessageHeader_t*)data;
    const QUCCCellInfo_t *cellData = (const QUCCCellInfo_t*)&header->dataLen;

    size_t numCells = cellData->getNumOfCells();
   
    cellInfo->setNumCells(numCells);    

    for(size_t i = 0; i<numCells;++i) {
        cellInfo->setVoltage(i,cellData->getCellVolt(i));    
    }
    
    return true;
}



bool QUCCBms::parseBasicInfo(const MessageHeader_t *message, size_t dataSize,BmsBasicInfo_t *basicInfo) const {
    
    
    const QUCCBasicInfo_t *info = (const QUCCBasicInfo_t*)message->data;
    basicInfo->capacityRemain = info->getcapacityRemain();
    
    basicInfo->totalVoltage = info->getTotalVoltage();
	basicInfo->current = info->getcurrent();
    basicInfo->nominalCapacity = info->getnominalCapacity();
	basicInfo->cycleLife =info->getcycleLife(); 
    basicInfo->productDate = info->getproductDate();    
    basicInfo->version = info->getversion();
    basicInfo->stateOfCharge = info->getstateOfCharge(); // in percent
    basicInfo->cellsInSeries = info->getcellsInSeries();
    basicInfo->numTempSensors = info->getnumTempSensors();
    basicInfo->temps[0] = info->getTemp(0);
    basicInfo->temps[1] = info->getTemp(1);
    basicInfo->temps[2] = info->getTemp(2);
    basicInfo->alarmsStatus.rawValue = info->getprotectionStatus();
    basicInfo->balanceStatusLow = info->getbalanceStatusLow();
    basicInfo->balanceStatusHigh = info->getbalanceStatusHigh();
    basicInfo->batteryStatus.rawValue = 0;
    basicInfo->batteryStatus.status.DischargingEnabled = (info->getfetControlStatus() >> 1) & 1;
    basicInfo->batteryStatus.status.ChargingEnabled =  info->getfetControlStatus() & 1;
    
    return true;

}
    
}

#endif
