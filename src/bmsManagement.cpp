
#include <Arduino.h>

#include "common.h"
#include "debugManagement.h"
#include "bmsManagement.h"
#include "modbusManagement.h"

/*
// A
#define STARTBYTE 65

// Z
#define STOPBYTE 90
*/

/* JDB */

#define STARTCODE 0x8778
#define STOPBYTE 0x68

#define MAXMESSAGESIZE 512
#define BAUDRATE 115200

char gBmsUpdateIntervalValue[NUMBER_LEN] = "3000";
char gBmsDummyValue[STRING_LEN];

BmsBasicInfo_t *gBmsBasicInfo = 0;
BmsCellInfo_t *gBmsCellInfo = 0;

bool gBmsDisconnect = true;
bool gBmsUpdated = false;
static uint8_t protocolVersion = 0;


static uint8_t messagebuffer[MAXMESSAGESIZE];

static TaskHandle_t taskId;

static unsigned long bmsUpdateIntervalMilis = 3000;

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

#pragma pack(push,1)
struct MessageHeader_t
{
	uint16_t start;  // Must be 0x8778 (If interpreted as string it is "NW")
    uint16_t dataLen;
    uint8_t terminalNr[4];
	uint8_t command;
	uint8_t source;
    uint8_t frameType;
    uint8_t data[0];

    uint16_t length() { return __builtin_bswap16(dataLen);}
} ;

struct MessageFooter_t
{
    uint8_t recordNumber[4];
    uint8_t endByte; // Always 0x68
    uint8_t unused[2];
    uint8_t checksum[2];
};

struct Cell_t
{
    uint8_t index;
    uint16_t voltage;
};
#pragma pack(pop)

// We use Serial 2 for communication.
// It's connected to an RS485 bus. 
// It is shared with the Charge controllers therefore we have to 
// get a mutex that protects the serial line from parallel access.
size_t readAnswerMessage()
{
    #define TIMEOUT 1000
    size_t toRead,read;
    unsigned long starttime = millis();

    size_t messagePointer = 0;
    MessageHeader_t *header =(MessageHeader_t*)messagebuffer;
    header->start = 0;
    toRead = sizeof(MessageHeader_t);
    do
    {
        read = Serial2.readBytes(messagebuffer + messagePointer, toRead);
        toRead -= read;
        messagePointer += read;        
    
    } while (toRead && (millis() - starttime) < TIMEOUT);

    if(toRead != 0 || header->start != STARTCODE) {
        // We didn't read anything or the answer has the wrong format
        // That is assumed to be an error
        debugE("BMS: Still to read %d bytes. Startvalue is %X\n",toRead, (int)header->start);
        return 0;
    }

    // Now we have the header.
    // Let's check the length of the message.
    // dataLen + checksum + stopbyte
    toRead = header->length()+2-sizeof(MessageHeader_t); // Length is the whole message without the two start bytes
    if(toRead+sizeof(MessageHeader_t) > MAXMESSAGESIZE) {
        debugW("BMS: Message too large for buffer %d\n", toRead );
        return 0;
    }
    
    messagePointer = 0;
     do
    {       
        read = Serial2.readBytes(header->data + messagePointer, toRead);
        toRead -= read;
        messagePointer += read;
    } while (toRead && read);

    return (messagePointer + sizeof(*header));
}

static bool bmsSetupBms() {
   
   bmsUpdateIntervalMilis = atoi(gBmsUpdateIntervalValue);
   if(bmsUpdateIntervalMilis < 500) {
       bmsUpdateIntervalMilis = 3000;
   }
   return true;
}


static uint16_t parseCellInfo(uint8_t *data)
{
    
    size_t numCells = data[0] / 3;
    Cell_t *cell = (Cell_t*)&data[1];


    if(!gBmsCellInfo) {
        gBmsCellInfo = (BmsCellInfo_t*)malloc(numCells*sizeof(uint16_t)+sizeof(uint8_t));
        gBmsCellInfo->setNumCells(numCells);
    } 
    DBG_SECT ( else {if(gBmsCellInfo->getNumOfCells() != numCells) {DEBUG_W("BMS: Numer of cells changed!");}})

    for(size_t i = 0; i<numCells;++i) {
        gBmsCellInfo->setVoltage(cell->index,__builtin_bswap16(cell->voltage));    
    }
    
    return data[0]+1;
}


#define MKTEMP(TMP) (TMP)<=100 ?  (int16_t)(TMP):100-((int16_t)(TMP))
#define MK16(HIGH,LOW) (((uint16_t)(HIGH))<<8 | (LOW))
#define NEXT16Bit(RESULT) { RESULT = MK16(buffer[currentIndex],buffer[currentIndex+1]); currentIndex+=2; }

static bool parseMessage(MessageHeader_t *message, size_t dataSize) {
    if(dataSize != message->length() + 2) {
        DEBUG_W("BMS Cellinfo: Message size too small %d or not equal message size %d\n",(int)dataSize, message->length());
        return false;
    }

    if(!gBmsBasicInfo) {
        gBmsBasicInfo = (BmsBasicInfo_t*)malloc(sizeof(BmsBasicInfo_t));
    }

    size_t currentIndex = 0;
    uint8_t *buffer = message->data;
    uint16_t val16=0;
    uint8_t currentRegister = buffer[currentIndex++];
    while(buffer[currentIndex] != STOPBYTE) {
        switch(buffer[currentIndex]) {
            case 0x79:
                currentIndex += parseCellInfo(buffer+currentIndex+1)+1;
                break;
            case 0x80:
            case 0x81:
            case 0x82:
                // Values above 100 indicate a negative value
                NEXT16Bit(val16)
                gBmsBasicInfo->temps[currentRegister - 0x80] = MKTEMP(val16);                
                break;
            case 0x83:
                NEXT16Bit(gBmsBasicInfo->totalVoltage)                
                break;
            case 0x84:
                // Set sign bit indicates a negative value
                // Why they don't encode it in 2-complement do only the chinese gods know.
                NEXT16Bit(val16)
                if(protocolVersion == 1) {
                    gBmsBasicInfo->current = (val16 & 0x8000) ? -1*((val16 & 0x7FFF) ):(val16 & 0x7FFF) ;
                } else {
                    gBmsBasicInfo->current = 10000-(int)val16;
                }
                break;
            case 0x85:
                gBmsBasicInfo->stateOfCharge = buffer[currentIndex++];
                break;
            case 0x86:
                gBmsBasicInfo->numTempSensors = buffer[currentIndex++];
                break;
            case 0x87: 
                NEXT16Bit(gBmsBasicInfo->cycleLife);
                break;
            case 0x89:
                // No idea what "Total cycle capacity" is.
                currentIndex += 4;
                break;
            case 0x8a:
                NEXT16Bit(gBmsBasicInfo->cellsInSeries);
                break;
            case 0x8b:    
                NEXT16Bit(gBmsBasicInfo->alarmsStatus.rawValue);
                break;
            case 0x8c:
                NEXT16Bit(gBmsBasicInfo->batteryStatus.rawValue);
                break;
            case 0xc0:
                protocolVersion = buffer[currentIndex++];
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
                currentIndex+=1;
                break;
        }
    }
    gBmsUpdated = true;
    return true;

}

static bool processMessage(uint8_t *message, size_t length) {

    bool result = false;
    MessageHeader_t *header = (MessageHeader_t*)message;

    if(header->start != STARTCODE ||  length != header->dataLen+7) {
        
        DEBUG_W("BMS: Received invalid message\n");
        
        return false;
    }
    if(header->frameType == REPLY) {
    
        result = parseMessage(header,length);//Its basic info
    } else {
        debugW("BMS: Got unknown answer message with type %d\n",(int)header->command);            
    }

    return result;
}

static bool handleRequest(uint8_t *request, size_t length)
{
    bool result = false;
    size_t messageSize = 0;
    if (xSemaphoreTake(gSerial2Mutex, pdMS_TO_TICKS(bmsUpdateIntervalMilis - 400)) == pdTRUE)
    {
        while (Serial2.read() != -1);
        if(Serial2.baudRate()!= BAUDRATE) {
            Serial2.updateBaudRate(BAUDRATE);
        }

        Serial2.write(request, length);
        Serial2.flush();     
        vTaskDelay(300);  
        messageSize = readAnswerMessage();
        xSemaphoreGive(gSerial2Mutex);
        if (messageSize)
        {            
            // We got an answer
            result = processMessage(messagebuffer, messageSize);
        } else {
             debugW("BMS: Could not get result message\n");
        }
    }
    return result;
}


DBG_SECT(
void printCellInfo() //DBUG all data to uart
{

    Debug.printf("Number of cells: %u\n", gBmsCellInfo->getNumOfCells());
    for (byte i = 0; i < gBmsCellInfo->getNumOfCells(); ++i)
    {
        Debug.printf("Cell no. %u", i);
        Debug.printf("   %f\n", (float)gBmsCellInfo->getCellVolt(i) / 1000);
    }    
}

void printBasicInfo() //DBUG_ON all data to uart
{
    
    Debug.printf("Total voltage: %f\n", (float)gBmsBasicInfo->getTotalVoltage() / 100);
    Debug.printf("Amps: %f\n", (float)gBmsBasicInfo->getcurrent() / 100);
    Debug.printf("CapacityRemainAh: %f\n", (float)gBmsBasicInfo->getcapacityRemain() / 100);
    Debug.printf("CapacityRemainPercent: %d\n", gBmsBasicInfo->getstateOfCharge());        
    Debug.printf("Mosfet Status: 0x%x\n", gBmsBasicInfo->getStatus());
    Debug.printf("Cycle life: %d\n", gBmsBasicInfo->getcycleLife());
    Debug.printf("Cells in Series: %d\n", gBmsBasicInfo->getcellsInSeries());
    Debug.printf("NominalCapacity: %f\n", (float)gBmsBasicInfo->getnominalCapacity()/ 100.0);
    Debug.printf("ProtectionStatus: 0x%x\n", gBmsBasicInfo->getprotectionStatus());
    Debug.printf("Version: %d\n", gBmsBasicInfo->getversion());

    Debug.println("Temps");
    for(int i = 0;i<gBmsBasicInfo->getnumTempSensors();++i) {
        Debug.printf("Temp %d: %f \n", i, (float)gBmsBasicInfo->getTemp(i) / 10.0);
    }
}

)

#define MSGSIZE 21
static bool readSingleData(uint8_t readRegister = 0x83)
{
    bool result = false;
    
    static uint8_t request[MSGSIZE] = {0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x83, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0xA9}; 

    request[11] = readRegister;
#if 0
    // This is only for DBUG_ON purposes
    static uint8_t result[] = {0xDD, 0x03, 0x00, 0x1B, 0x17, 0x00, 0x00, 0x00, 0x02, 0xD0, 0x03, 0xE8, 0x00, 0x00, 0x20, 0x78, 0x01, 0x02,
                               0x00, 0x00, 0x02, 0x04, 0x10, 0x48, 0x03, 0x0F, 0x02, 0x0B, 0x76, 0x0B, 0x82, 0xFB, 0xFF, 0x77};
    memcpy(messagebuffer, result, sizeof(result));
    processMessage(result, sizeof(result));
#endif
    
    // Uncomment this in real software
    result = handleRequest(request, MSGSIZE);

/*
DBG_SECT(
    if (Debug.isActive(Debug.DEBUG)) {
        printBasicInfo();
    }
)
*/
    return result;
}

static bool readAllValues()
{
    static uint8_t request[MSGSIZE]  = {0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x29}; // read all
    bool result = 0;
#if 0
    static uint8_t result[] = {0xDD, 0x04, 0x00, 0x1E, 0x0F, 0x66, 0x0F, 0x63, 0x0F, 0x63, 0x0F, 0x64, 0x0F, 0x3E, 0x0F, 0x63, 0x0F, 0x37,
                               0x0F, 0x5B, 0x0F, 0x65, 0x0F, 0x3B, 0x0F, 0x63, 0x0F, 0x63, 0x0F, 0x3C, 0x0F, 0x66, 0x0F, 0x3D, 0xF9, 0xF9, 0x77};
    memcpy(messagebuffer, result, sizeof(result));
    processMessage(result, sizeof(result));
#endif
    result = handleRequest(request, MSGSIZE);

/*
DBG_SECT(
    if (Debug.isActive(Debug.DEBUG)) {
    if (gBmsCellInfo)
    {
        printCellInfo();
    }
    }
)
*/
    return result;
}

void bmsEnable(bool on) {
    gBmsDisconnect = !on;
}

void bmsLoop(void *)
{
    #define FORCEINTERVAL  150000
    TickType_t previousTime = xTaskGetTickCount();
    unsigned long lastRead = 0;
    readAllValues();
    while (true)
    {
        vTaskDelayUntil(&previousTime, pdMS_TO_TICKS(bmsUpdateIntervalMilis));

        if (!gBmsDisconnect)
        {
            // Query the BMS
            readAllValues();
        } else {
            // Once in a while we update the basic data nevertheless.            
            unsigned long now = previousTime * portTICK_PERIOD_MS; 
            if(now - lastRead > FORCEINTERVAL) {
                DEBUG_I("Forcing BMS basic data update\n");
                if(readAllValues()) {
                    lastRead = now;
                }
            }
        }
    }
}


void bmsSetup()
{
    if(bmsSetupBms()) {
        BaseType_t result = xTaskCreate(bmsLoop, "bms", 2048, 0, 1, &taskId);

        if (result != pdPASS)
        {
            rprintE(" BMS taskCreation failed with error ");
            rprintEln(result);
        }
    }
}
