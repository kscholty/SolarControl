
#pragma once

#if NEVER
#include <stdint.h>

extern uint16_t adcCalibrateValue(uint16_t val);

extern void adcStartCalibration();

extern size_t adcReadCalibrationData();

#endif