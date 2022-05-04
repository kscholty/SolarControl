

#include "seplosbms_impl.h"


namespace BMS {


constexpr const char*  SeplosBms::headerMask;
constexpr const char*  SeplosBms::footerMask; 

bool SeplosBms::setup() 
{
    return false;
}

bool SeplosBms::getHighFrequRequest( const uint8_t **request,size_t *length) const
 {
     // Request warnings and stuff....
    static const uint8_t message[] = {0x7E,0x32,0x30,0x30,0x30,0x34,0x36,0x34,0x34,0x45,0x30,0x30,0x32,0x30,0x31,0x46,0x44,0x32,0x45,0x0D};
    *request = message;
    *length = sizeof(message);
    return true;
 }

bool SeplosBms::getLowFrequRequest(const uint8_t **request,size_t *length) const 
{
    // Telemetry CID2 = 0x42 request for address 0;
    // The resault will give us cell voltages
    static const uint8_t message[] = {0x7E,0x32,0x30,0x30,0x30,0x34,0x36,0x34,0x32,0x45,0x30,0x30,0x32,0x30,0x31,0x46,0x44,0x33,0x35,0x0D};
    *request = message;
    *length = sizeof(message);
    return true;
}

size_t SeplosBms::messageHeaderSize() const
{
    // This is the size including the LENGTH field.
    return 13;
}

size_t SeplosBms::getBodySize(const uint8_t *messagebuffer,size_t length) const
{
    uint8_t addr,CID2,lChecksum;
    uint16_t lenid;

    //"~20%2hhx46%2hhx%1hhx%3hhx"; // "~20 ADR 46 CID2 LCHECKSUM LENID"
    if((sscanf((const char*)messagebuffer,headerMask,&addr,&CID2,&lChecksum,&lenid) == 4)&& (calcLChecksum(lenid) == lChecksum)) {
        return lenid; // I guess that's wrong. Let's check the manual once more.
    }
    return 0;
}

bool SeplosBms::processMessage(const uint8_t *answer, size_t messageSize, BmsBasicInfo_t *basicInfo, BmsCellInfo_t *cellInfo) 
{
    return false;
}


unsigned char SeplosBms::calcLChecksum(unsigned short length) const
{
    unsigned short sum;
    
    sum = (length & 0x0F) +((length>>4) & 0x0F) + ((length >>8)&0x0F);    
    sum = ((~sum)+1) & 0x0F;
    return sum;
}

unsigned short SeplosBms::calcChecksum(const uint8_t* message, unsigned short length) const
{
    unsigned int sum = 0;
    // Start with 1 to skip SOI
    for(unsigned short i = 1; i<length;++i) {
        sum += message[i];
    }
    sum = ((~sum)+1) & 0xFFFF;
    return 0;
}


}