
#include "common.h"

#include "debugManagement.h"
#include "JKBms_impl.h"

namespace BMS { 

namespace {

static const int STARTCODE = 0x574E;

enum FRAMETYPE
{
    REQUEST = 0,
    REPLY = 1,
    REPORT=2,
    NUMFRAMETYPES
};

enum COMMAND {
    ACTIVATE = 0x01,
    WRTITEREGS = 0x02,
    READBASICS = 0x03,
    PAIRCODE = 0x05,
    READALL = 0x06,
    NUMCOMMANDS
};

enum SOURCE 
{
    SRC_BMS = 0,
    SRC_BLUETOOTH = 1,
    SRC_GPS = 2,
    SRC_PC=3,
    NUMSOURCES
};

}

bool JKBms::setup() {
    mProtocolVersion = 1;
    return true;
}

 bool JKBms::getHighFrequRequest(const uint8_t **request,size_t *length) const {
    static const uint8_t message[] = {0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0xAA}; 
    *request = message;
    *length = sizeof(message);
    return true;
 }

 bool JKBms::getLowFrequRequest(const uint8_t **request,size_t *length) const {
    static const uint8_t message[]  = {0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x29}; // read all
    *request = message;
    *length = sizeof(message);
    return true;
 }

 bool JKBms::getSetupRequest(uint16_t id, const uint8_t **request,size_t *length) const {
     // Ask for nominal capacity to be able to calculate remaining capacity
    static const uint8_t message[] = {0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0xD0}; 

    *request = message;
    *length = sizeof(message);        

    return id == 0;
 }

 size_t JKBms::messageHeaderSize() const {
     return sizeof(MessageHeader_t);
 }

size_t JKBms::getBodySize(const uint8_t *messagebuffer,size_t length) const {
    const MessageHeader_t *header = (const MessageHeader_t*)(messagebuffer);
    return header->length()-sizeof(*header);
}

bool JKBms::processMessage(const uint8_t *message, size_t length, BmsBasicInfo_t *basicInfo, BmsCellInfo_t *cellInfo)  {

    bool result = false;
    const MessageHeader_t *header = (const MessageHeader_t*)message;

    if(header->start != STARTCODE ||  length != header->length()) {
        
        DEBUG_W("BMS: Received invalid message with startcode 0x%04x and size %d (%d)\n",header->start,header->length(),length);
        
        return false;
    }
    if(header->frameType == REPLY) {
    
        result = parseMessage(header,length,basicInfo,cellInfo);//Its basic info
    } else {
        debugW("BMS: Got unknown answer message with type %d\n",(int)header->frameType);            
    }

    return result;
}

uint16_t JKBms::parseCellInfo(const uint8_t *data,BmsCellInfo_t *cellInfo) const {
    size_t numCells = data[0] / 3;
    const Cell_t *cell = (const Cell_t*)&data[1];

    cellInfo->setNumCells(numCells);    
    DEBUG_I("Num Cells: %d\n",numCells);

    for(size_t i = 0; i<numCells;++i,++cell) {        
        cellInfo->setVoltage(cell->index-1,__builtin_bswap16(cell->voltage));          
    }
    
    return data[0]+1;
}


#define MKTEMP(TMP) ((TMP)<=100 ?  (int16_t)(TMP):100-((int16_t)(TMP)))
#define MK16(HIGH,LOW) (((uint16_t)(HIGH))<<8 | (LOW))
#define MK32(BH,B2,B3,BL) ((((uint32_t)BH)<<24)|(((uint32_t)B2)<<16) | (((uint32_t)B3)<<8) | BL)
#define NEXT16Bit(RESULT) { RESULT = MK16(buffer[currentIndex],buffer[currentIndex+1]); currentIndex+=2; }
#define NEXT32Bit(RESULT) { RESULT = MK32(buffer[currentIndex],buffer[currentIndex+1], buffer[currentIndex+2],buffer[currentIndex+3]); currentIndex+=4; }

bool JKBms::parseMessage(const MessageHeader_t *message, size_t dataSize,BmsBasicInfo_t *basicInfo, BmsCellInfo_t *cellInfo) {
    if(dataSize != message->length() ) {
        DEBUG_W("BMS Cellinfo: Message size too small %d or not equal message size %d\n",(int)dataSize, message->length());
        return false;
    }

    
    size_t currentIndex = 0;
    size_t dataLength = message->dataLength();
    const uint8_t *buffer = message->data;
    uint16_t val16=0;
    uint8_t currentRegister;
    DEBUG_I("\nParsing Message\n");
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
                basicInfo->temps[currentRegister - 0x80] = MKTEMP(val16) * 10;                
                break;
            case 0x83:
                NEXT16Bit(basicInfo->totalVoltage)                
                break;
            case 0x84:
                // Set sign bit indicates a positive value
                // Why they don't encode it in 2-complement do only the chinese gods know.
                NEXT16Bit(val16)
                if(mProtocolVersion == 1) {
                    basicInfo->current = (val16 & 0x8000) ? ((val16 & 0x7FFF) ):-1*(val16 & 0x7FFF) ;
                } else {
                    basicInfo->current = 10000-(int)val16;
                }
                break;
            case 0x85:
                basicInfo->stateOfCharge = buffer[currentIndex++];
                basicInfo->capacityRemain = basicInfo->stateOfCharge * basicInfo->nominalCapacity;
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
            case 0xaa: 
                NEXT32Bit(basicInfo->nominalCapacity);   
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

