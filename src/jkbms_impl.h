
#pragma once

#include "bmstypes.h"

namespace BMS {


class JKBms {

    protected:
#pragma pack(push,1)

    struct MessageFooter_t
{
    uint8_t recordNumber[4];
    uint8_t endByte; // Always 0x68
    uint8_t unused[2];
    uint8_t checksum[2];
};

struct MessageHeader_t
{
	uint16_t start;  // Must be 0x8778 (If interpreted as string it is "NW")
    uint16_t dataLen;
    uint8_t terminalNr[4];
	uint8_t command;
	uint8_t source;
    uint8_t frameType;
    uint8_t data[0];
    uint16_t dataLength() const { return length()-sizeof(MessageHeader_t)-sizeof(MessageFooter_t);}
    uint16_t length() const { return __builtin_bswap16(dataLen)+2;}
    MessageFooter_t *footer() { return (MessageFooter_t*)(data+dataLength());}
} ;

struct Cell_t
{
    uint8_t index;
    uint16_t voltage;
};

#pragma pack(pop)

public:
    static const int MESSAGESIZE = 512;
    static const int BAUDRATE = 115200;
public:
    bool setup();
    bool getHighFrequRequest( const uint8_t **request,size_t *length) const;
    bool getLowFrequRequest(const uint8_t **request,size_t *length) const ;
    size_t messageHeaderSize() const;

    size_t getBodySize(const uint8_t *messagebuffer,size_t length) const;
    bool processMessage(const uint8_t *answer, size_t messageSize, BmsBasicInfo_t *basicInfo, BmsCellInfo_t *cellInfo) ;
protected:

    uint16_t parseCellInfo(const uint8_t *data, BmsCellInfo_t *cellInfo) const ;
    bool parseMessage(const MessageHeader_t *message, size_t dataSize,BmsBasicInfo_t *basicInfo, BmsCellInfo_t *cellInfo) ;


    unsigned int mProtocolVersion;

};

}