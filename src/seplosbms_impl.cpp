

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
     //"~ 2 0 0 0 4 6 4 4 E 0 0 2 0 0 F D 3 5 $"
    static const uint8_t message[] = {'~',0x32,0x30,0x30,0x30,0x34,0x36,0x34,0x34,0x45,0x30,0x30,0x32,0x30,0x30,0x46,0x44,0x33,0x35,'$'};
    *request = message;
    *length = sizeof(message);
    return true;
 }

bool SeplosBms::getLowFrequRequest(const uint8_t **request,size_t *length) const 
{
    // Telemetry CID2 = 0x42 request for address 0;
    // The resault will give us cell voltages
    // "~  2  0  0  0  4  6  4  2  E  0  0  2  0  0  F  D  3  7  $ "
    static const uint8_t message[] = {'~',0x32,0x30,0x30,0x30,0x34,0x36,0x34,0x32,0x45,0x30,0x30,0x32,0x30,0x31,0x46,0x44,0x33,0x37,'$'};
    *request = message;
    *length = sizeof(message);
    return true;
}

bool SeplosBms::getSetupRequest(uint16_t id, const uint8_t **request,size_t *length) const {
    *request = 0;
    *length = 0;
    return false;
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

    //"~20%2hhx46%2hhx%1hhx%3hhx"; "~20 ADR 46 CID2 LCHECKSUM LENID"
    if((sscanf((const char*)messagebuffer,headerMask,&addr,&CID2,&lChecksum,&lenid) == 4)&& (calcLChecksum(lenid) == lChecksum)) {
        return lenid+5; // It's DATA + CHECKSUM+ + EOI
    }
    return 0;
}

bool SeplosBms::processMessage(const uint8_t *answer, size_t messageSize, BmsBasicInfo_t *basicInfo, BmsCellInfo_t *cellInfo) 
{
    static const char *AnswerHigh = "~20004600806200011000000000010101010101010101010101060000010100000001141248800000100000001000000000000000000010EB0E$";
    static const char *AnwerLow = "~2000460010960001100D450D0C0CBB0C570B1107A30018000000000000000000000000000000000000060BA90BA908B708B70BB30BAE000007041D4B0A3A9801F33A98000103E800000000000000000000DEE5$";
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