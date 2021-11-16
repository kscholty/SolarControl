
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

#define STARTBYTE 0xDD
#define STOPBYTE 0x77

#define MAXMESSAGESIZE 64
#define BAUDRATE 9600

char gBmsUpdateIntervalValue[NUMBER_LEN] = "3000";
char gBmsDummyValue[STRING_LEN];

BmsBasicInfo_t *gBmsBasicInfo = 0;
BmsCellInfo_t *gBmsCellInfo = 0;

bool gBmsDisconnect = true;

static uint8_t messagebuffer[MAXMESSAGESIZE];

static TaskHandle_t taskId;

static unsigned long bmsUpdateIntervalMilis = 3000;


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

// We use Serial 2 for communication.
// It's connected to an RS485 bus. 
// It is shared with the Charge controllers therefore we have to 
// get a mutex that protects the serial line from parallel access.


size_t readAnswerMessage()
{
    size_t toRead,read;
    int repeat = 2;

    uint8_t messagePointer = 0;
    MessageHeader_t *header =(MessageHeader_t*)messagebuffer;
    header->start = 0;
    toRead = sizeof(MessageHeader_t);
    do
    {
        read = Serial2.readBytes(messagebuffer + messagePointer, toRead);
        toRead -= read;
        messagePointer += read;        
    } while (toRead && read );

    if(toRead != 0 || header->start != STARTBYTE) {
        // We didn't read anything or the answer has the wrong format
        // That is assumed to be an error
        debugE("BMS: Still to read %d bytes. Startvalue is %X\n",toRead, (int)header->start);
        return 0;
    }

    // Now we have the header.
    // Let's check the length of the message.
    // dataLen + checksum + stopbyte
    toRead = header->dataLen+3; // 2bytes checksum+stopbyte
    if(toRead+sizeof(MessageHeader_t) > MAXMESSAGESIZE) {
        debugW("BMS: Message too large %d\n", toRead );
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


static bool parseBasicInfo(BmsBasicInfo_t* data, uint8_t datasize) {
    
    if(datasize<sizeof(BmsBasicInfo_t) || data->size() != datasize) {
        DEBUG_W("BMS Cellinfo: Message size too small %d or not equal message size %d\n",(int)datasize, data->size());
        return false;
    }

    if(!gBmsBasicInfo) {
        gBmsBasicInfo = (BmsBasicInfo_t*)malloc(datasize);
    }
    memcpy(gBmsBasicInfo,data,datasize);
    return true;
}

static bool parseCellInfo(BmsCellInfo_t *data, uint8_t datasize)
{

    if (data->getNumOfCells() * 2 + sizeof(uint8_t) != datasize)
    {
        DEBUG_W("BMS Cellinfo: Message size wrong %d\n",(int)datasize);
        return false;
    }
    
    if(!gBmsCellInfo) {
        gBmsCellInfo = (BmsCellInfo_t*)malloc(datasize);
    }

    memcpy(gBmsCellInfo, data, datasize);

    return true;
}

static bool processMessage(uint8_t *message, size_t length) {

    bool result = false;
    MessageHeader_t *header = (MessageHeader_t*)message;

    if(header->start != STARTBYTE || header->status != 0 || length != header->dataLen+7) {
        
        DEBUG_W("BMS: Received invalid message\n");
        
        return false;
    }

    switch(header->command) {
        case 0x03: result = parseBasicInfo((BmsBasicInfo_t*)header->data,header->dataLen);//Its basic info
        break;
        case 0x04: result = parseCellInfo((BmsCellInfo_t*)&header->dataLen,header->dataLen+sizeof(header->dataLen)); // Its Cell info
        break;
        default:
            debugW("BMS: Got unknown answer message with type %d\n",(int)header->command);            
            break;
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
    Debug.printf("Balance Code Low: 0x%x\n", gBmsBasicInfo->getbalanceStatusLow());
    Debug.printf("Balance Code High: 0x%x\n", gBmsBasicInfo->getbalanceStatusHigh());
    Debug.printf("Mosfet Status: 0x%x\n", gBmsBasicInfo->getfetControlStatus());
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

static bool readBasicData()
{
    static uint8_t request[7] = {0xdd, 0xa5, 0x03, 0x00, 0xff, 0xfd, 0x77};

#if 0
    // This is only for DBUG_ON purposes
    static uint8_t result[] = {0xDD, 0x03, 0x00, 0x1B, 0x17, 0x00, 0x00, 0x00, 0x02, 0xD0, 0x03, 0xE8, 0x00, 0x00, 0x20, 0x78, 0x01, 0x02,
                               0x00, 0x00, 0x02, 0x04, 0x10, 0x48, 0x03, 0x0F, 0x02, 0x0B, 0x76, 0x0B, 0x82, 0xFB, 0xFF, 0x77};
    memcpy(messagebuffer, result, sizeof(result));
    processMessage(result, sizeof(result));
#endif
    // Uncomment this in real software
    handleRequest(request, 7);

DBG_SECT(
    if (Debug.isActive(Debug.DEBUG)) {
        printBasicInfo();
    }
)
    return false;
}

static bool readCellValues()
{
    static uint8_t request[7] = {0xdd, 0xa5, 0x4, 0x0, 0xff, 0xfc, 0x77};

#if 0
    static uint8_t result[] = {0xDD, 0x04, 0x00, 0x1E, 0x0F, 0x66, 0x0F, 0x63, 0x0F, 0x63, 0x0F, 0x64, 0x0F, 0x3E, 0x0F, 0x63, 0x0F, 0x37,
                               0x0F, 0x5B, 0x0F, 0x65, 0x0F, 0x3B, 0x0F, 0x63, 0x0F, 0x63, 0x0F, 0x3C, 0x0F, 0x66, 0x0F, 0x3D, 0xF9, 0xF9, 0x77};
    memcpy(messagebuffer, result, sizeof(result));
    processMessage(result, sizeof(result));
#endif
    handleRequest(request, 7);

DBG_SECT(
    if (Debug.isActive(Debug.DEBUG)) {
    if (gBmsCellInfo)
    {
        printCellInfo();
    }
    }
)
    return false;
}

void bmsEnable(bool on) {
    gBmsDisconnect = !on;
}

void bmsLoop(void *)
{
    TickType_t previousTime = xTaskGetTickCount();
    while (true)
    {
        vTaskDelayUntil(&previousTime, pdMS_TO_TICKS(bmsUpdateIntervalMilis));

        if (!gBmsDisconnect)
        {
            // Query the BMS
            readBasicData();
            readCellValues();
            modbusUpdateBMSValues();            
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
