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

	String options = "";
	uint8_t pos = lastCmd.indexOf(" ");
	if (pos > 0) {
		options = lastCmd.substring(pos + 1);
	}

	if (lastCmd == "calibrate") {		
		debugA("Calibrating Current and voltage sensor");
		doCalibration = true;
	} else 	if(lastCmd.startsWith("dummy ")) {		
		debugA("Dummy command with option %s\n",options.c_str());		
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

    String helpCmd = "calibrate - calculate calibration values\r\n";

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