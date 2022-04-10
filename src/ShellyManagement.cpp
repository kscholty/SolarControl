#include <WiFi.h>
#include <netdb.h>
#include "ShellyManagement.h"
#include "mqttmanagement.h"
#include "inverterManagement.h"

char ginverterShellyValue[STRING_LEN] = "";
ShellyDevice shelly3EM;
static bool isInit = false;
static const int delayMs = 500;

bool resolveHostname(char hostname[STRING_LEN]) {
  struct hostent *host;

  if (WiFi.isConnected()) {   
    host = gethostbyname(hostname);
    if (host) {      
      char IP[16];
      sprintf(IP,"%d.%d.%d.%d",host->h_addr_list[0][0], 
                               host->h_addr_list[0][1], 
                               host->h_addr_list[0][2],
                               host->h_addr_list[0][3] );
      DEBUG_I("Shelly 3EM Resolved IP is %s\n",IP);
      shelly3EM.setIp(IP);
      return true;
    }
  }
  DEBUG_E("Could not resolve 3EM hostname %s\n",hostname);
  return false;
}

void shellyInit() { 

    if(!isInit) {
        isInit = resolveHostname(mqttEM3Name); 
        //shelly3EM.setDelay(500);
        shelly3EM.setDelay(0);
    }
}

void shellyLoop(void *) {
  static unsigned long counter = 0;

  static const unsigned long RESOLVEDELAY = (1000 / delayMs * 60 * 15);
  TickType_t previousTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&previousTime, pdMS_TO_TICKS(delayMs));
    if (!isInit) {
      shellyInit();
    } else {
      if (shelly3EM.updateNow()) {
        bool powerUpdated = false;
        for (int valueType = 0; valueType < ValueNumValues; ++valueType) {
          for (unsigned int aLeg = 0; aLeg < 3; ++aLeg) {
            switch (valueType) {
              case ValuePower: {
                float aValue = shelly3EM.getPower(aLeg) / 1000.0f;

                if (gGridLegValues[valueType][aLeg] != aValue) {
                  gGridLegValues[valueType][aLeg] = aValue;
                  powerUpdated = true;
                  // DEBUG_I("Power (%d)= %.2f\n", aLeg, aValue);
                }
              } break;
              case ValueCurrent:
                gGridLegValues[valueType][aLeg] =
                    shelly3EM.getCurrent(aLeg) / 1000.0f;
                break;
              case ValueVoltage:
                gGridLegValues[valueType][aLeg] =
                    shelly3EM.getVoltage(aLeg) / 1000.0f;
                break;
              case ValuePowerFactor:
                gGridLegValues[valueType][aLeg] =
                    shelly3EM.getPowerFactor(aLeg) / 1000.0f;
                break;
              default:
                break;
            }
          }
          gGridSumValues[valueType] = gGridLegValues[valueType][0] +
                                      gGridLegValues[valueType][1] +
                                      gGridLegValues[valueType][2];
          /*
          if (i == ValuePowerFactor && gGridLegValues[i][aLeg] < 0) {
            gGridLegValues[i][aLeg] = fabs(gGridLegValues[i][aLeg]);
          }
          */
        }
        if (powerUpdated) {
          gInverterGridPowerUpdated();
        }
      } else {
        DEBUG_E("Last Shelly (%s) Error: %s", shelly3EM.getQueryUrl().c_str(),
                shelly3EM.getLastErrorString().c_str());
      }

      if (++counter > RESOLVEDELAY) {
        // Repeat name resolution from time to time if
        // ip changes.
        if (resolveHostname(mqttEM3Name)) {
          counter = 0;
        }
      }
    }
  }
}

void shellySetup()
{
    TaskHandle_t handle;
    BaseType_t result = xTaskCreate(shellyLoop, "shelly", 4096, 0, 2, &handle);
    if (result != pdPASS)
    {
        if (Debug.isActive(Debug.ERROR))
        {
            Debug.print(" Shelly taskCreation failed with error ");
            Debug.println(result);
        }
        gInverterTaskHandle = 0;
    }
}