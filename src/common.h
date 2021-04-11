

#pragma once

#define DEBUG 1

#define STRING_LEN 64
#define NUMBER_LEN 32

// -- Initial name of the Thing. Used e.g. as SSID of the own Access Point.
static const char thingName[] = "SolarGridController";

extern SemaphoreHandle_t gSerial2Mutex;
