

#pragma once

#include "common.h"

extern char mqttServerValue[STRING_LEN];
extern char mqttPortValue[NUMBER_LEN];
extern char mqttUserNameValue[STRING_LEN];
extern char mqttUserPasswordValue[STRING_LEN];

extern void mqttSetup();
extern void mqttLoop(unsigned long);
extern bool mqttReconnect();

extern char mqttEM3Name[STRING_LEN];
extern char mqttEM3Topic[STRING_LEN];

#if DEBUG
#define BLYNKPORT V50
extern void mqttDisable();
extern void mqttEnable();
extern bool mqttEnabled();
#endif

