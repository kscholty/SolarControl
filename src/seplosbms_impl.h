
#pragma once

#include "common.h"
#include "bmstypes.h"

namespace BMS {


class SeplosBms {

protected:
     static constexpr const char*  headerMask = "~20%2hhx46%2hhx%1hhx%3hhx"; // "~20 ADR 46 CID2 LCHECKSUM LENID"
     static constexpr const char*  footerMask = "%4hx\r"; // "CHKSUM CR"
public:
    static const int MESSAGESIZE = 175;
    static const int BAUDRATE = 19200;
public:
    bool setup();
    bool getHighFrequRequest( const uint8_t **request,size_t *length) const;
    bool getLowFrequRequest(const uint8_t **request,size_t *length) const ;
    bool getSetupRequest(uint16_t id, const uint8_t **request,size_t *length) const;
    size_t messageHeaderSize() const;

    size_t getBodySize(const uint8_t *messagebuffer,size_t length) const;
    bool processMessage(const uint8_t *answer, size_t messageSize, BmsBasicInfo_t *basicInfo, BmsCellInfo_t *cellInfo) ;
protected:

    //bool parseCellInfo(const MessageHeader_t *message, size_t dataSize, BmsCellInfo_t *cellInfo) const ;
    //bool parseBasicInfo(const MessageHeader_t *message, size_t dataSize,BmsBasicInfo_t *basicInfo) const ;

    unsigned char calcLChecksum(unsigned short length) const;
    unsigned short calcChecksum(const uint8_t* message, unsigned short length) const;


};

}