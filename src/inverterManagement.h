#pragma once

#include "common.h"

// Thepin the inverter is connected to
#define INVERTERPIN 4

enum EGridValueTypes {
    ValuePower = 0,
    ValueCurrent = 1,
    ValueVoltage = 2,
    ValuePowerFactor = 3,
    ValueNumValues
};


extern double Kp;
extern double Ki;
extern double Kd;

extern void inverterActivatePidValues();

// This contains all lage values of the different 
// Types
extern float gGridLegValues[ValueNumValues][3];
extern float gGridSumValues[ValueNumValues];

// The current power output of the inverter
extern double gInverterPower;

//The current curent output of the inverter
extern float gInverterCurrent;

// The current voltage at the inverter output
extern float gInverterVoltage;

// The current power factor at the inverter output
extern float gInverterPowerFactor;

// The target wattage we want the inverter to generate
extern double gInverterTarget;

// The maximum time of ms that er allowe to pass between two power readings
// before the inverter's is set to 0;
extern long gInverterTimeout;

// Setting this to true forces the iverter's output to 0
extern bool gInverterShutdown;

// Is true, the charge controllers should stay active even if battery is full
extern bool gInverterExcessToGrid;

// Initializes the inverter. It starts wit 0 output
extern void inverterSetup();

// This sets the morst urgent pins
extern void inverterPreInit();

// Parameters for the web config to store results in
extern char gInverterOffsetValue[NUMBER_LEN];
extern char gInverterTimeoutValue[NUMBER_LEN];
extern char gInverterEmergencyTargetValue[NUMBER_LEN];
extern char gSendExcessToGrid[NUMBER_LEN];
extern char gInverterUpdateIntervalValue[NUMBER_LEN];
extern char gInverterLegValue[NUMBER_LEN] ;


extern TaskHandle_t gInverterTaskHandle;

// This has to be called by MQTT when a new
// Consumption value has been received.
extern void gInverterGridPowerUpdated();

extern void inverterSetRealTarget();

// Force the inverter output to 0
#define inverterLock() gInverterShutdown = true

// Allow dynamic change of inverter output
#define inverterUnlock() gInverterShutdown = false

// Check in Inverer is forced to 0
#define inverterLocked() (gInverterShutdown == true)
