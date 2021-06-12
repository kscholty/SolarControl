
#pragma once


extern void debugSetup();
extern void debugLoop(unsigned long);

// #define DEBUG_DISABLED
#ifdef DEBUG_DISABLED
#define DBG_SECT(code) {}
#else
#define DBG_SECT(code) code

#include <RemoteDebug.h>
extern RemoteDebug Debug;
#endif

