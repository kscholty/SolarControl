

#pragma once

#include <Arduino.h>

#define STRING_LEN 40
#define NUMBER_LEN 32

#define JKBMS 0
#define QUCCBMS 1
#define SEPLOSBMS 2

// -- Initial name of the Thing. Used e.g. as SSID of the own Access Point.
static const char thingName[] = "SolarGridController";

extern SemaphoreHandle_t gSerial2Mutex;

