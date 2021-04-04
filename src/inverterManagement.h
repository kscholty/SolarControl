#pragma once

#include "common.h"

#define INVERTERPIN 4

// The values for the three grid legs
extern float gGridLegsPower[3];

// The sum of used power
extern float gGridSumPower;

// Stores the time the last update from the
// Gris Power Sensor was received
extern long gInverterLastUpdateReceived;

// The maximum time of ms that er allowe to pass between two power readings
// before the inverter's is set to 0;
extern long gInverterTimeout;

// Setting this to true forces the iverter's output to 0
extern bool gInverterShutdown;

// Initializes the inverter. It starts wit 0 output
extern void inverterSetup();

// This sets the morst urgent pins
extern void inverterPreInit();

// Parameters for the web config to store results in
extern char gInverterTargetValue[STRING_LEN];
extern char gInverterTimeoutValue[STRING_LEN];

extern TaskHandle_t gInverterTaskHandle;

// Force the inverter output to 0
#define inverterLock() gInverterShutdown = true

// Allow dynamic change of inverter output
#define inverterUnlock() gInverterShutdown = false

// Check in Inverer is forced to 0
#define inverterLocked() (gInverterShutdown == true)
