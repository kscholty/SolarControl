#pragma once 

#include "common.h"

extern void wifiSetup();
extern void wifiReconnect();
extern void wifiLoop(unsigned long);
extern bool gNeedReset;
