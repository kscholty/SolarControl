#pragma once

extern bool adcInit();
extern double adcGetCurrent();
extern double adcGetVoltage();
extern double adcGetPower();

extern void adcStop();
extern void adcStart();
