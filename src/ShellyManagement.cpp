#include <WiFi.h>
#include <netdb.h>
#include "ShellyManagement.h"
#include "mqttmanagement.h"

char ginverterShellyValue[STRING_LEN] = "";
ShellyDevice shelly3EM;
static bool isInit = false;



bool resolveHostname(char hostname[STRING_LEN]) {
  struct hostent *host;

  if (WiFi.isConnected()) {
    host = gethostbyname(mqttEM3Name);
    if (host) {
      shelly3EM.setIp(host->h_addr_list[0]);
      return true;
    }
  }
  return false;
}

void shellyInit() { 

    if(!isInit) {
        isInit = resolveHostname(mqttEM3Name); 
        shelly3EM.setDelay(500);
    }
}

void shellyLoop(unsigned long) {
    if(!isInit) {
        shellyInit();
    } else {
        shelly3EM.doLoop();
    }
}

