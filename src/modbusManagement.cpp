
#include <WiFiServer.h>
#include "common.h"
#include "modbusManagement.h"
#include "ModbusTCPServer.h"

WiFiServer wifiServer(502);
ModbusTCPServer modbusServer;

void modbusUpdateChargerValues() {

}

void modbusUpdateInvcverterValues() {

}

void modbusUpdateBMSValues() {

}

void modbusSetup() {

    wifiServer.begin();
    modbusServer.begin();
    modbusServer.configureHoldingRegisters(0,5);
}

void modbusLoop(unsigned long) {

    // listen for incoming clients
  WiFiClient client = wifiServer.available();
  
  if (client) {
    
    // let the Modbus TCP accept the connection 
    modbusTCPServer.accept(client);

    while (client.connected()) {
      // poll for Modbus TCP requests, while client connected
      modbusTCPServer.poll();     
    }
}
