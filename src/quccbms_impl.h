
#pragma once

#include "common.h"
#include "bmstypes.h"

namespace BMS {


class QUCCBms {

    protected:
    
#pragma pack(push,1)

struct MessageHeader_t
{
	uint8_t start;
	uint8_t command;
	uint8_t status;
	uint8_t dataLen;
    uint8_t data[0];
} ;

#pragma pack(pop)

public:
    static const int MESSAGESIZE = 64;
    static const int BAUDRATE = 9600;
public:
    bool setup();
    bool getHighFrequRequest( const uint8_t **request,size_t *length) const;
    bool getLowFrequRequest(const uint8_t **request,size_t *length) const ;
    bool getSetupRequest(uint16_t id, const uint8_t **request,size_t *length) const;
    size_t messageHeaderSize() const;

    size_t getBodySize(const uint8_t *messagebuffer,size_t length) const;
    bool processMessage(const uint8_t *answer, size_t messageSize, BmsBasicInfo_t *basicInfo, BmsCellInfo_t *cellInfo) ;
protected:

    bool parseCellInfo(const MessageHeader_t *message, size_t dataSize, BmsCellInfo_t *cellInfo) const ;
    bool parseBasicInfo(const MessageHeader_t *message, size_t dataSize,BmsBasicInfo_t *basicInfo) const ;


};

}