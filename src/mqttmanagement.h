

#pragma once

#include "common.h"

extern char mqttServerValue[STRING_LEN];
extern char mqttPortValue[STRING_LEN];
extern char mqttUserNameValue[STRING_LEN];
extern char mqttUserPasswordValue[STRING_LEN];

extern void mqttSetup();
extern void mqttLoop(long);
extern void mqttReconnect();

extern char mqttEM3Name[STRING_LEN];
extern char mqttEM3Topic[STRING_LEN];