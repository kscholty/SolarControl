

#include "seplosbms_impl.h"


namespace BMS {

// Test messages could be
//  static const char *AnswerLow = "~20004600806200011000000000010101010101010101010101060000010100000001141248800000100000001000000000000000000010EB0E$";
//  static const char *AnswerHigh ="~2000460010960001100D450D0C0CBB0C570B1107A30018000000000000000000000000000000000000060BA90BA908B708B70BB30BAE000007041D4B0A3A9801F33A98000103E800000000000000000000DEE5$";


constexpr const char*  SeplosBms::headerMask;
constexpr const char*  SeplosBms::footerMask; 

bool SeplosBms::setup() 
{
    Serial.println("Setting up Seplos BMS");
    return false;
}

bool SeplosBms::getLowFrequRequest( const uint8_t **request,size_t *length) const
 {
     // Request warnings and stuff....
     //"~ 2 0 0 0 4 6 4 4 E 0 0 2 0 0 F D 3 5 $"
    static const uint8_t message[] = {'~',0x32,0x30,0x30,0x30,0x34,0x36,0x34,0x34,0x45,0x30,0x30,0x32,0x30,0x30,0x46,0x44,0x33,0x35,'$'};
    *request = message;
    *length = sizeof(message);
    return true;
 }

bool SeplosBms::getHighFrequRequest(const uint8_t **request,size_t *length) const 
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

bool  SeplosBms::processHighFrequMessage(const uint8_t *answer, size_t messageSize, BmsBasicInfo_t *basicInfo, BmsCellInfo_t *cellInfo) {
    
    char const *scanPos = (const char*)answer+messageHeaderSize()+4;
    uint8_t count;
    uint16_t value;
    
    sscanf(scanPos,"%2hhx",&count);
    scanPos += 2;
    cellInfo->setNumCells(count);
    
    for(int i=0;i<count;++i) {
        uint16_t voltage;
        sscanf(scanPos,"%4hx",&voltage);
        scanPos += 4;
    }
    sscanf(scanPos,"%2hhx",&count);
    scanPos += 2;
    basicInfo->numTempSensors = max((int)count,3);    
    for(int i=0;i<count;++i) {
        uint16_t temp;
        sscanf(scanPos,"%4hx",&temp);
        scanPos += 4;
        temp -= 2731;
        if(i<3) basicInfo->temps[i] = temp;
        Serial.printf("Temp %d: %d\n",i,temp);
    }

    // current
    sscanf(scanPos,"%4hx",&value);
    scanPos += 4;
    basicInfo->current = value;
        
    // Pack Voltage
    sscanf(scanPos,"%4hx",&value);
    scanPos += 4;    
    basicInfo->totalVoltage = value;

    // Remaining capacity
    sscanf(scanPos,"%4hx",&value);
    scanPos += 4;
    basicInfo->capacityRemain = value;
    
    //sscanf(scanPos,"%2hhx",&count);
    //Serial.printf("Should be 10: %d\n",count);
    scanPos += 2;
    
    // Battery Capacity
    sscanf(scanPos,"%4hx",&value);
    scanPos += 4;
    basicInfo->nominalCapacity = value;
    
    // SOC
    sscanf(scanPos,"%4hx",&value);
    scanPos += 4;
    basicInfo->stateOfCharge = value;    

    // Rated capacity Skip
    //sscanf(scanPos,"%4hx",&value);
    scanPos += 4;
    
    // Cycles
    sscanf(scanPos,"%4hx",&value);
    scanPos += 4;
    basicInfo->cycleLife = value;


    return true;

    // SOH Skip
    //sscanf(scanPos,"%4hx",&value);
    scanPos += 4;    

    // Port Voltage Skip
    //sscanf(scanPos,"%4hx",&value);
    scanPos += 4;
    
    

}

bool  SeplosBms::processLowFrequMessage(const uint8_t *answer, size_t messageSize, BmsBasicInfo_t *basicInfo, BmsCellInfo_t *cellInfo) {

    char const *scanPos = (const char*)answer+messageHeaderSize()+4;
    uint8_t count,value8;
    uint16_t value16;
    Alarms_t &alarms = basicInfo->alarmsStatus.alarms;
    
    // #Cells
    sscanf(scanPos,"%2hhx",&count);
    scanPos += 2;

    // Cell warnings    
    scanPos += 2*count; // Skip it
    /*
    alarms.CellOvervoltage = alarms.CellUndervoltage = 0;
    for(int i=0;i<count;++i) {
        sscanf(scanPos,"%2hhx",&value8);
        scanPos += 2;
        Serial.printf("Warning %d: %hhx\n",i,value8);
        
        switch(value8) {
            case 1: alarms.CellUndervoltage = 1; break;
            case 2: alarms.CellOvervoltage = 1; break;
            default: break;
        }
    }
*/
    // #Sensors
    sscanf(scanPos,"%2hhx",&count);
    scanPos += 2;        

    // Temps
    scanPos += 2*count; // Skip it
    /*
    alarms.BatteryOverTemperature = alarms.BatteryLowTemperature = 0;
    // Cell Temp warnings
    for(int i=0;i<count;++i) {        
        sscanf(scanPos,"%2hhx",&value8);
        scanPos += 2;            
        Serial.printf("Temp Warning%d: %hhx\n",i,value8);
        switch(i) {
            case 0:
            case 1:
            case 2:
            case 3: if(value8==0x1) { alarms.BatteryLowTemperature = 1;} else if(value8 == 0x2) {alarms.BatteryOverTemperature = 1;} break; 
            case 4: if(value8 != 0) { alarms.OvertemperatureAlarmBatteryBox =1;} else { alarms.OvertemperatureAlarmBatteryBox =0;} break;
            case 5: if(value8 != 0) { alarms.BmsOverTemperature =1;} else {alarms.BmsOverTemperature = 0;} break;
            default: break;
        }
    }
*/
    // Current warning
    //sscanf(scanPos,"%2hhx",&value8); Skip
    scanPos += 2;
    /*
    Serial.printf("Current Warning %hhx\n",value8);
    switch(value8) {
        case 0x1: alarms.DischargingOvercurrent = 1; alarms.ChargingOvercurrent = 0; break;
        case 0x2: alarms.ChargingOvercurrent = 1; alarms.DischargingOvercurrent = 0; break;
        default: alarms.ChargingOvercurrent = alarms.DischargingOvercurrent = 0; break;
    }
*/
    // Pack Voltage
    //sscanf(scanPos,"%2hhx",&value8); Skip
    scanPos += 2;
    /*
    Serial.printf("Pack Voltage Warning %hhx\n",value8);
    switch(value8) {
        case 0x1: alarms.DischargingUndervoltage = 1; alarms.ChargingOvervoltage = 0; break;
        case 0x2: alarms.ChargingOvervoltage = 1; alarms.DischargingUndervoltage = 0; break;
        default:  alarms.ChargingOvervoltage = alarms.DischargingUndervoltage = 0; break;
    }
    */

    //Counter
    //sscanf(scanPos,"%2hhx",&value8);
    scanPos += 2;
    
     // Warning 1
     // Skip
     scanPos += 2;

     // Warning 2
     sscanf(scanPos,"%2hhx",&value8);
    scanPos += 2;
    alarms.CellOvervoltage = (value8 >> 1) & 0x01;
    alarms.CellUndervoltage= (value8 >> 3) & 0x01;

    alarms.ChargingOvervoltage = (value8 >> 5) & 0x01;
    alarms.DischargingUndervoltage = (value8 >> 7) & 0x01;
    

    // Warning 3
    sscanf(scanPos,"%2hhx",&value8);
    scanPos += 2;
    alarms.BatteryOverTemperature = ((value8 >> 1) & 0x01) | ((value8 >> 5) & 0x01);
    alarms.BatteryLowTemperature = ((value8 >> 3) & 0x01) | ((value8 >> 7) & 0x01);

    // Warning 4
    // sscanf(scanPos,"%2hhx",&value8); Skip
    scanPos += 2;
    
    
    // Warning 5
    sscanf(scanPos,"%2hhx",&value8);
    scanPos += 2;
    alarms.ChargingOvercurrent = ((value8 >> 1) & 0x01) ;
    alarms.DischargingOvercurrent = ((value8 >> 3) & 0x01) ;
    
    // Warning 6
    //sscanf(scanPos,"%2hhx",&value8); Skip
    scanPos += 2;

    // Power status
    sscanf(scanPos,"%2hhx",&value8);
    scanPos += 2;
    basicInfo->batteryStatus.status.DischargingEnabled = (value8 & 0x01) ;
    basicInfo->batteryStatus.status.ChargingEnabled = ((value8 >> 1) & 0x01) ;


    sscanf(scanPos,"%2hhx",&value8);
    scanPos += 2;
    basicInfo->balanceStatusLow = value8;
    
    sscanf(scanPos,"%2hhx",&value8);
    scanPos += 2;
    basicInfo->balanceStatusLow |= ((uint16_t)(value8))<<8;

    return true;
}

bool SeplosBms::processMessage(const uint8_t *answer, size_t messageSize, BmsBasicInfo_t *basicInfo, BmsCellInfo_t *cellInfo) 
{
    Serial.printf("Seplos processing message with size %d\n", messageSize);
    if(messageSize == 168) {
        return processHighFrequMessage(answer,messageSize,basicInfo,cellInfo);
    }
    return processLowFrequMessage(answer,messageSize,basicInfo,cellInfo);    
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