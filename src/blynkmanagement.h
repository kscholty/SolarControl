
#pragma once

#include "common.h"
#define BLYNK_STRLEN 64

#if DEBUG
#define BLYNK_VPIN_MQTT_ENABLE V50
#endif

#define BLYNK_VPIN_LEG_0 V0
#define BLYNK_VPIN_LEG_1 V1
#define BLYNK_VPIN_LEG_2 V2
#define BLYNK_VPIN_ALL_LEGS V3

#define FIRST_CHARGER_VPIN V4

// Charger ID can be either 0 or 1
// VPIN is the index of the pin of the controller (0...n) to address.
#define BLYNK_CHARGER_PIN(charger_id, VPIN) (((charger_id)*NUM_CHARGER_VALUES) + (VPIN) + FIRST_CHARGER_VPIN)

extern char blynkTokenValue[BLYNK_STRLEN];
extern char blynkServerValue[BLYNK_STRLEN];
extern char blynkPortValue[NUMBER_LEN];

extern void blynkSetup();
extern void blynkReconnect();
extern void blynkLoop(unsigned long);