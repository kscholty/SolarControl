
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEClient.h>

#include "bmsManagement.h"

char gServerNameValue[STRING_LEN] = "A4:C1:38:F1:DC:55";
char gServiceUUIDValue[STRING_LEN] = "0000ff00-0000-1000-8000-00805f9b34fb";
char gReadCharUUIDValue[STRING_LEN]= "0000ff01-0000-1000-8000-00805f9b34fb";
char gWriteCharUUIDValue[STRING_LEN]= "0000ff02-0000-1000-8000-00805f9b34fb";

static unsigned long bmsUpdateIntervalMilis = 15000;
static BLEAddress *pServerAddress;
static bool bleConnected = false;
static BLERemoteCharacteristic* pReadCharacteristic;
static BLERemoteCharacteristic* pWriteCharacteristic;
static unsigned long lastReconnectAttempt = 0;

class BLECallbacks: public BLEClientCallbacks {
public:
	virtual ~BLECallbacks() {};
	virtual void onConnect(BLEClient *pClient) {
        bleConnected = true;
    };
	virtual void onDisconnect(BLEClient *pClient) {
        bleConnected = false;
    };
} bleClientCallbacks;

void bleNotifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    if (isNotify)
        Serial.print("Got Notify with ");
    else
        Serial.print("Got No Notify with ");
    Serial.print(length);
    Serial.println(" bytes of data.");
        Serial.print(">");
    for (int i = 0; i < length; ++i)
    {
        Serial.print((char)pData[i]);
    }
    Serial.println("<");
#if 0    
    if (pReadCharacteristic)
    {
        std::string value = pReadCharacteristic->readValue();
        const char *data = value.data();
        unsigned int len = value.length();
        Serial.print("Value has ");
        Serial.print(length);
        Serial.println(" bytes of data.");
            Serial.print(">");
        for (int i = 0; i < len; ++i)
        {
            Serial.print(data[i]);
        }
        Serial.println("<");
    }
#endif
    if(pWriteCharacteristic) {
       // pWriteCharacteristic->writeValue(pData,length,false);
    }
}

void bmsSetupBLE() {
   BLEDevice::init(thingName);
   pServerAddress = new BLEAddress(std::string(gServerNameValue));
   
}


bool bleConnectToServer() 
{
    #if DEBUG
    Serial.print("Forming a BLE connection to ");
    Serial.println(pServerAddress->toString().c_str());
    #endif
    
    BLEClient* pClient  = BLEDevice::createClient();
    pClient->setClientCallbacks(&bleClientCallbacks);

    // Connect to the remove BLE Server.
    if(!pClient->connect(*pServerAddress) ) {
        Serial.println("Connection to BLE server failed");
        delete pClient;
        return false;
    }

    delay(10);
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(gServiceUUIDValue);
    #if DEBUG
    Serial.println(pRemoteService->toString().c_str());
    #endif
    if (pRemoteService == nullptr) {
        #if DEBUG
      Serial.print("Failed to find our service UUID: ");
      Serial.println(gServiceUUIDValue);
      #endif
      pClient->disconnect();
      return false;
    }
    #if DEBUG
    Serial.println(" - Found our service");
#endif

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pReadCharacteristic = pRemoteService->getCharacteristic(gReadCharUUIDValue);
    if (pReadCharacteristic == nullptr) {
      Serial.print("Failed to find read characteristic UUID: ");
      Serial.println(gReadCharUUIDValue);

      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found read characteristic");

// Obtain a reference to the characteristic in the service of the remote BLE server.
    pWriteCharacteristic = pRemoteService->getCharacteristic(gWriteCharUUIDValue);
    if (pWriteCharacteristic == nullptr) {
      Serial.print("Failed to find read characteristic UUID: ");
      Serial.println(gWriteCharUUIDValue);

      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found write characteristic");
    pReadCharacteristic->registerForNotify(bleNotifyCallback);
    return true;
}



void bmsLoop(void *)
{
    TickType_t previousTime = xTaskGetTickCount();
    while (true)
    {
        vTaskDelayUntil(&previousTime, pdMS_TO_TICKS(bmsUpdateIntervalMilis));
        
        if (!bleConnected)
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
        else
        {
            // Query the BMS
            if (pWriteCharacteristic)
            {
                pWriteCharacteristic->writeValue((uint8_t*)"Hallo\r\n", 7, false);
            }
        }
    }
}

void bmsSetup()
{
    TaskHandle_t taskId;

    bmsSetupBLE();

    BaseType_t result = xTaskCreate(bmsLoop, "bms", 3072, 0, 1, &taskId);
    if (result != pdPASS)
    {
        Serial.print(" Charge Controller taskCreation failed with error ");
        Serial.println(result);
    }
}