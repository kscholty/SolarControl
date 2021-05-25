
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEClient.h>

#include "bmsManagement.h"


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

static char serviceUUIDValue[STRING_LEN] = "0000ff00-0000-1000-8000-00805f9b34fb";
static char readCharUUIDValue[STRING_LEN]= "0000ff01-0000-1000-8000-00805f9b34fb";
static char writeCharUUIDValue[STRING_LEN]= "0000ff02-0000-1000-8000-00805f9b34fb";


char gBmsServerNameValue[STRING_LEN] = "A4:C1:38:F1:DC:55";
char gBmsUpdateIntervalValue[NUMBER_LEN] = "3000";



BmsBasicInfo_t *gBmsBasicInfo = 0;
BmsCellInfo_t *gBmsCellInfo = 0;

bool gBmsConnected = false;
bool gBmsDisconnect = false;


static BLEAddress *pServerAddress;

static BLEClient *pClient = 0;
static BLERemoteCharacteristic* pReadCharacteristic = 0;
static BLERemoteCharacteristic* pWriteCharacteristic = 0;
static unsigned long lastReconnectAttempt = 0;
static uint8_t messagebuffer[MAXMESSAGESIZE];
static uint8_t messagePointer = 0;
static bool messageValid = false;
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

class BLECallbacks: public BLEClientCallbacks {
public:
	virtual ~BLECallbacks() {};
	virtual void onConnect(BLEClient *pClient) {
        gBmsConnected = true;
    };
	virtual void onDisconnect(BLEClient *pClient) {
        gBmsConnected = false;
        delete pClient;
        pClient = 0;
        pReadCharacteristic = 0;
        pWriteCharacteristic = 0;        
    };
} bleClientCallbacks;

void bleNotifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    if (!length)
    {
        return;
    }

    messageValid = false;
    if (pData[0] == STARTBYTE)
    {
        messagePointer = 0;
    }

    if (messagePointer + length > MAXMESSAGESIZE)
    {
#if DEBUG
        Serial.println("Message size exceeds limit, discarding it.");
#endif
        messagePointer = 0;
        return;
    }

    memcpy(messagebuffer + messagePointer, pData, length);
    messagePointer += length;

    if (pData[length - 1] == STOPBYTE)
    {
        messageValid = true;
        xTaskNotify(taskId, messagePointer, eSetValueWithOverwrite);
    }
}

static bool bmsSetupBLE() {
    String address(gBmsServerNameValue);

    return false;
    
    address.trim();
    if(address.length() == 0 || address.length() != 17) {
        // Don't go on. This is useless...
        return false;
    }
   BLEDevice::init(thingName);
   pServerAddress = new BLEAddress(address.c_str());
   bmsUpdateIntervalMilis = atoi(gBmsUpdateIntervalValue);
   if(bmsUpdateIntervalMilis < 500) {
       bmsUpdateIntervalMilis = 3000;
   }
   return true;
}

bool bleConnectToServer()
{
#if DEBUG
    Serial.print("Forming a BLE connection to ");
    Serial.println(pServerAddress->toString().c_str());
#endif

    if(gBmsDisconnect) {
        return false;
    }

    if(gBmsConnected) {
        return true;
    }

    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(&bleClientCallbacks);

    // Connect to the remove BLE Server.
    if (!pClient->connect(*pServerAddress))
    {
        Serial.println("Connection to BLE server failed");
        delete pClient;
        return false;
    }

    #if DEBUG
    Serial.println(" - Connected to server");
    #endif

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService *pRemoteService = pClient->getService(serviceUUIDValue);

#if DEBUG
    Serial.println(pRemoteService->toString().c_str());
#endif

    if (pRemoteService == nullptr)
    {
#if DEBUG
        Serial.print("Failed to find our service UUID: ");
        Serial.println(serviceUUIDValue);
#endif
        pClient->disconnect();
        return false;
    }
#if DEBUG
    Serial.println(" - Found our service");
#endif

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pReadCharacteristic = pRemoteService->getCharacteristic(readCharUUIDValue);
    if (pReadCharacteristic == nullptr)
    {
        Serial.print("Failed to find read characteristic UUID: ");
        Serial.println(readCharUUIDValue);

        pClient->disconnect();  
        return false;
    }

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pWriteCharacteristic = pRemoteService->getCharacteristic(writeCharUUIDValue);
    if (pWriteCharacteristic == nullptr)
    {
        Serial.print("Failed to find read characteristic UUID: ");
        Serial.println(writeCharUUIDValue);

        pClient->disconnect();    
        return false;
    }
    pReadCharacteristic->registerForNotify(bleNotifyCallback);
    return true;
}


static bool parseBasicInfo(BmsBasicInfo_t* data, uint8_t datasize) {
    
    if(datasize<sizeof(BmsBasicInfo_t) || data->size() != datasize) {
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
        return false;
    }
    
    if(!gBmsCellInfo) {
        gBmsCellInfo = (BmsCellInfo_t*)malloc(datasize);
    }

    memcpy(gBmsCellInfo, data, datasize);

    return true;
}

static bool processMessage(uint8_t *message, uint32_t length) {

    bool result = false;
    MessageHeader_t *header = (MessageHeader_t*)message;

    if(header->start != STARTBYTE || header->status != 0 || length != header->dataLen+7) {
        #if DEBUG
        Serial.println("Received invalid message");
        #endif

        return false;
    }

    switch(header->command) {
        case 0x03: result = parseBasicInfo((BmsBasicInfo_t*)header->data,header->dataLen);//It's basic info
        break;
        case 0x04: result = parseCellInfo((BmsCellInfo_t*)&header->dataLen,header->dataLen+sizeof(header->dataLen)); // It's Cell info
        break;
        default:
            Serial.print("BMS: Got unknown answer message with type ");
            Serial.println(header->command);
            break;
    }

    return result;
}

static bool handleRequest(uint8_t *request, size_t length)
{
    bool result = false;
    uint32_t messageSize = 0;
    if (pWriteCharacteristic)
    {
        pWriteCharacteristic->writeValue(request, length, false);
        if (xTaskNotifyWait(0, 0, &messageSize, pdMS_TO_TICKS(bmsUpdateIntervalMilis / 3)) != pdFAIL)
        {            
            // We got a notification
            result = processMessage(messagebuffer, messageSize);
        } else {
            Serial.println("COuld not get result message");
        }
    }
    return result;
}


#if DEBUG
void printCellInfo() //debug all data to uart
{

    Serial.printf("Number of cells: %u\n", gBmsCellInfo->getNumOfCells());
    for (byte i = 0; i < gBmsCellInfo->getNumOfCells(); ++i)
    {
        Serial.printf("Cell no. %u", i);
        Serial.printf("   %f\n", (float)gBmsCellInfo->getCellVolt(i) / 1000);
    }    
}

void printBasicInfo() //debug all data to uart
{
    
    Serial.printf("Total voltage: %f\n", (float)gBmsBasicInfo->getTotalVoltage() / 100);
    Serial.printf("Amps: %f\n", (float)gBmsBasicInfo->getcurrent() / 100);
    Serial.printf("CapacityRemainAh: %f\n", (float)gBmsBasicInfo->getcapacityRemain() / 100);
    Serial.printf("CapacityRemainPercent: %d\n", gBmsBasicInfo->getstateOfCharge());    
    Serial.printf("Balance Code Low: 0x%x\n", gBmsBasicInfo->getbalanceStatusLow());
    Serial.printf("Balance Code High: 0x%x\n", gBmsBasicInfo->getbalanceStatusHigh());
    Serial.printf("Mosfet Status: 0x%x\n", gBmsBasicInfo->getfetControlStatus());
    Serial.printf("Cycle life: %d\n", gBmsBasicInfo->getcycleLife());
    Serial.printf("Cells in Series: %d\n", gBmsBasicInfo->getcellsInSeries());
    Serial.printf("NominalCapacity: %f\n", (float)gBmsBasicInfo->getnominalCapacity()/ 100.0);
    Serial.printf("ProtectionStatus: 0x%x\n", gBmsBasicInfo->getprotectionStatus());
    Serial.printf("Version: %d\n", gBmsBasicInfo->getversion());

    Serial.println("Temps");
    for(int i = 0;i<gBmsBasicInfo->getnumTempSensors();++i) {
        Serial.printf("Temp %d: %f \n", i, (float)gBmsBasicInfo->getTemp(i) / 10.0);
    }
}

#endif

static bool readBasicData()
{
    static uint8_t request[7] = {0xdd, 0xa5, 0x03, 0x00, 0xff, 0xfd, 0x77};

    // This is only for debug purposes
    static uint8_t result[] = {0xDD, 0x03, 0x00, 0x1B, 0x17, 0x00, 0x00, 0x00, 0x02, 0xD0, 0x03, 0xE8, 0x00, 0x00, 0x20, 0x78, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x10, 0x48, 0x03, 0x0F, 0x02, 0x0B, 0x76, 0x0B, 0x82, 0xFB, 0xFF, 0x77};
    memcpy(messagebuffer, result, sizeof(result));
    processMessage(result, sizeof(result));

    // Uncomment this in real software
    //handleRequest(request, 7);

#if DEBUG
    printBasicInfo();
#endif
    return false;
}

static bool readCellValues()
{
    static uint8_t request[7] = {0xdd, 0xa5, 0x4, 0x0, 0xff, 0xfc, 0x77};

    static uint8_t result[] = {0xDD, 0x04, 0x00, 0x1E, 0x0F, 0x66, 0x0F, 0x63, 0x0F, 0x63, 0x0F, 0x64, 0x0F, 0x3E, 0x0F, 0x63, 0x0F, 0x37,
                               0x0F, 0x5B, 0x0F, 0x65, 0x0F, 0x3B, 0x0F, 0x63, 0x0F, 0x63, 0x0F, 0x3C, 0x0F, 0x66, 0x0F, 0x3D, 0xF9, 0xF9, 0x77};
    memcpy(messagebuffer, result, sizeof(result));
    processMessage(result, sizeof(result));

    //handleRequest(request, 7);

#if DEBUG
    if (gBmsCellInfo)
    {
        printCellInfo();
    }
#endif
    return false;
}

void bmsEnable(bool on) {
    gBmsDisconnect = on;
    lastReconnectAttempt = 0;    
}

void bmsLoop(void *)
{
    TickType_t previousTime = xTaskGetTickCount();
    while (true)
    {
        vTaskDelayUntil(&previousTime, pdMS_TO_TICKS(bmsUpdateIntervalMilis));

        if (!gBmsConnected)
        {
            if (!gBmsDisconnect)
            {
                unsigned long now = millis();
                if (now - lastReconnectAttempt > 5000)
                {
                    lastReconnectAttempt = now;
                    // Attempt to reconnect
                    if (bleConnectToServer())
                    {
                        lastReconnectAttempt = 0;
                    }
                }
            }
        }
        else
        {
            if (!gBmsDisconnect)
            {
                // Query the BMS
                if (pWriteCharacteristic)
                {
                    readBasicData();
                    readCellValues();
                }
            }
            else
            {  // Disconnect from BMS
                if (pClient)
                {
                    pClient->disconnect();
                }
            }
        }
    }
}

void bmsSetup()
{
    if(bmsSetupBLE()) {
        BaseType_t result = xTaskCreate(bmsLoop, "bms", 3072, 0, 1, &taskId);

        if (result != pdPASS)
        {
            Serial.print(" BMS taskCreation failed with error ");
            Serial.println(result);
        }
    }
}
