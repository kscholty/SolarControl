#pragma once

volatile extern int32_t gVoltageFactor;
extern int32_t gMaxVoltageFactor;
extern int32_t gMinVoltageFactor;
volatile extern int32_t gVoltageOffset;
extern char gVoltageFactorCorrectionOffset[NUMBER_LEN];

extern bool adcInit();
extern double adcGetCurrent();
extern double adcGetVoltage();
extern double adcGetPower();

extern void adcStop();
extern void adcStart();
