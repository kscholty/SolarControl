

#pragma once

#define STRING_LEN 128

extern char mqttServerValue[STRING_LEN];
extern char mqttPortValue[STRING_LEN];
extern char mqttUserNameValue[STRING_LEN];
extern char mqttUserPasswordValue[STRING_LEN];

extern void mqttSetup();
extern void mqttLoop();
