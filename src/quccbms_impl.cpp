
#include "common.h"

#if QUCCBMS

#include "debugManagement.h"
#include "quccbms_impl.h"

namespace BMS { 

namespace {

#define STARTBYTE 0xDD
#define STOPBYTE 0x77

#define MAXMESSAGESIZE 64
#define BAUDRATE 9600



#pragma pack(push,1)
struct QUCCBasicInfo_t
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

struct QUCCCellInfo_t
{
    public:
    uint8_t getNumOfCells() const {return numOfCells>>1;}
    uint16_t getCellVolt(unsigned int i) const { if (i<numOfCells) {return __builtin_bswap16(cellVolt[i]);} else { return 0;} }
    private:
	uint8_t numOfCells;
	uint16_t cellVolt[0]; // This in mV, hence devide it by 1000
} ;

#pragma pack(pop)




bool QUCCBms::setup() {
    mProtocolVersion = 1;
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
        case 0x03: result = parseBasicInfo(header,length,basicInfo,);//Its basic info
        break;
        case 0x04: result = parseCellInfo(header,length,cellInfo); // Its Cell info
        break;
        default:
            debugW("BMS: Got unknown answer message with type %d\n",(int)header->command);            
            break;
    }

    return result;
}

bool QUCCBms::parseCellInfo(const uint8_t *data, size_t length, BmsCellInfo_t *cellInfo) const {
    const MessageHeader_t *header = (const MessageHeader_t*)data;
    const QUCCCellInfo_t *cellData = (const QUCCCellInfo_t*)header->data;

    size_t numCells = cellData->getNumOfCells();
   
    cellInfo->setNumCells(numCells);    

    for(size_t i = 0; i<numCells;++i) {
        cellInfo->setVoltage(cellData->getCellVolt(i));    
    }
    
    return true;
}



bool QUCCBms::parseBasicInfo(const MessageHeader_t *message, size_t dataSize,BmsBasicInfo_t *basicInfo) {
    
    
    size_t currentIndex = 0;
    size_t dataLength = message->dataLength();
    const QUCCBasicInfo_t *info = (const QUCCBasicInfo_t*)message->data;
    
    basicInfo->


    while(currentIndex < dataLength) 
    {
        currentRegister = buffer[currentIndex++];
        DEBUG_I("Register: 0x%02x\n",currentRegister);
        switch(currentRegister) {            
            case 0x79:
                currentIndex += parseCellInfo(buffer+currentIndex,cellInfo);
                break;
            case 0x80:
            case 0x81:
            case 0x82:
                // Values above 100 indicate a negative value
                NEXT16Bit(val16)
                basicInfo->temps[currentRegister - 0x80] = MKTEMP(val16);                
                break;
            case 0x83:
                NEXT16Bit(basicInfo->totalVoltage)                
                break;
            case 0x84:
                // Set sign bit indicates a negative value
                // Why they don't encode it in 2-complement do only the chinese gods know.
                NEXT16Bit(val16)
                if(mProtocolVersion == 1) {
                    basicInfo->current = (val16 & 0x8000) ? -1*((val16 & 0x7FFF) ):(val16 & 0x7FFF) ;
                } else {
                    basicInfo->current = 10000-(int)val16;
                }
                break;
            case 0x85:
                basicInfo->stateOfCharge = buffer[currentIndex++];
                break;
            case 0x86:
                basicInfo->numTempSensors = buffer[currentIndex++];
                break;
            case 0x87: 
                NEXT16Bit(basicInfo->cycleLife);
                break;
            case 0x89:
                // No idea what "Total cycle capacity" is.
                currentIndex += 4;
                break;
            case 0x8a:
                NEXT16Bit(basicInfo->cellsInSeries);
                break;
            case 0x8b:    
                NEXT16Bit(basicInfo->alarmsStatus.rawValue);
                break;
            case 0x8c:
                NEXT16Bit(basicInfo->batteryStatus.rawValue);
                break;
            case 0xc0:
                mProtocolVersion = buffer[currentIndex++];
                break;
             // From here on we have the holding registers I don't care about
             case 0x9d: 
             case 0xa9:
             case 0xab:
             case 0xac:   
             case 0xae:
             case 0xaf:
             case 0xb1: 
             case 0xb3:
             case 0xb8:
             case 0xbb:
             case 0xbc:
             case 0xbd:
                // 8-Bit registers
                ++currentIndex;
                break;             
             case 0x8e:
             case 0x8f:
             case 0x90:
             case 0x91:
             case 0x92:
             case 0x93:
             case 0x94:
             case 0x95:
             case 0x96:
             case 0x97:
             case 0x98:
             case 0x99:
             case 0x9a:
             case 0x9b:
             case 0x9c:             
             case 0x9e:
             case 0x9f:
             case 0xa0:
             case 0xa1:
             case 0xa2:
             case 0xa3:
             case 0xa4:
             case 0xa5:
             case 0xa6:
             case 0xa7:
             case 0xa8:
             case 0xad:
             case 0xb0:
             case 0xbe:
             case 0xbf:
                // 16-Bit registers
                currentIndex+=2;
                break;
            case 0xaa:
            case 0xb5:
            case 0xb6:
            case 0xb9:
                //32-bit registers
                currentIndex+=4;
                break;
            case 0xb2:
                currentIndex+=10;
                break;
            case 0xb4:
                currentIndex+=8;
                break;
            case 0xb7:
                currentIndex+=15;
                break;
            case 0xba:
                currentIndex+=24;
                break;            
            default: 
                DEBUG_W("Unknown Opcode %02x\n",currentRegister );
                currentIndex+=1;
                break;
        }
    }
    DEBUG_I("Done\n\n");
    return true;

}
    
}

#endif
