#include <Arduino.h>

#include "common.h"
#include "debugManagement.h"


#include <DNSServer.h>
#include <ESPmDNS.h>

static unsigned long lastConnectionAttempt = 0;

DBG_SECT (
RemoteDebug Debug;
extern bool doCalibration;

void processCmdRemoteDebug() {

	String lastCmd = Debug.getLastCommand();

	if (lastCmd == "calibrate") {
		
		debugA("Clibrating Current sensor");
		doCalibration = true;
	} 
}

bool debugStart() {
	
	return Debug.begin(thingName, TELNET_PORT,Debug.ERROR); // Initialize the WiFi server	
}

void debugSetup() {
    MDNS.addService("telnet", "tcp", 23);

    Debug.setResetCmdEnabled(true); // Enable the reset command
	//Debug.showProfiler(true); // Profiler (Good to measure times, to optimize codes)
	Debug.showColors(true); // Colors

    String helpCmd = "calibrate - calculate calkibration values\r\n";


	Debug.setHelpProjectsCmds(helpCmd);
	Debug.setCallBackProjectCmds(&processCmdRemoteDebug);
	lastConnectionAttempt = millis();
}

void debugLoop(unsigned long now) {

	static bool connected =  false;

	if(!connected && now-lastConnectionAttempt > 10000) {
		lastConnectionAttempt = now;
		if (WiFi.isConnected()) {
			connected = debugStart();
		}
	} else {
    	Debug.handle();
	}
}

)