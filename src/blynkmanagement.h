
#pragma once

#define BLYNK_STRLEN 64
extern char blynkTokenValue[BLYNK_STRLEN];
extern char blynkServerValue[BLYNK_STRLEN];
extern char blynkPortValue[BLYNK_STRLEN];

extern void blynkSetup();
extern void blynkLoop();