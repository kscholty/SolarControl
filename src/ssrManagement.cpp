

#include <Dusk2Dawn.h>

#include "common.h"
#include "ssrManagement.h"
#include "inverterManagement.h"
#include "chargeControllerManagement.h"

bool gBatteryFull = false;
bool gBatteryEmpty = false;

char gNtpServerValue[STRING_LEN]="pool.ntp.org";
char gTimezoneValue[NUMBER_LEN] = "1";
char gLatitudeValue[NUMBER_LEN] = "50.937531"; // Köln...
char gLongitudeValue[NUMBER_LEN] = "6.960279";
char ginverterShellyValue[STRING_LEN] = "";


bool chargerEnabled[NUM_CHARGERS] = {false,false};
int chargerPin[NUM_CHARGERS] = {SSR_PIN_1,SSR_PIN_2};

enum DayNightEventType {
  EvSunrise = 0,
  EvSunset = 1,
  EvMidnight = 2,  
  EvUnknown
};


unsigned long DailyEvents[4];

static DayNightEventType nextEvent = EvUnknown;
static double latitude = 50.937531;
static double longitude = 6.960279;
static int timezone = 1;
 
 static DayNightEventType dayNightCalcEvents();

bool chargerStatus(CHARGERS charger) {
    return chargerEnabled[charger];
}

bool ssrSwitch(CHARGERS charger, bool on)
{
    bool ret = chargerEnabled[charger];
    chargerEnabled[charger] = on;
    digitalWrite(chargerPin[charger], on?HIGH:LOW);  
    #if DEBUG
    Serial.print("Switching ");
    Serial.println(charger);
    #endif
    return ret;
}


void ssrSetup() {

    for(int i=0;i<NUM_CHARGERS;++i) {
        pinMode(chargerPin[i],OUTPUT);
        if(chargerIsValid(i)) {
            chargerEnabled[i] = true;
            digitalWrite(chargerPin[i], HIGH);  
        } else {
            chargerEnabled[i] = false;
            digitalWrite(chargerPin[i], LOW);  
        }
    }  
    latitude = atof(gLatitudeValue);
    longitude = atof(gLongitudeValue);
    timezone = atoi(gTimezoneValue);
    nextEvent = dayNightCalcEvents();
    #if DEBUG
    Serial.print("Next event is ");
    Serial.println(nextEvent);
    #endif
}

void handleBatteryFull()
{
    if (!gInverterExcessToGrid)
    {
        for(int i=0;i<NUM_CHARGERS;++i) {       
            ssrSwitch((CHARGERS)i,false);
        }
    }
}

void handleBatteryEmpty() {
    
}

static DayNightEventType dayNightCalcEvents()
 {
   struct tm now;
   time_t sinceMidnight;
   int sunrise, sunset;
   DayNightEventType ret = EvUnknown;

   if (getLocalTime(&now))
   {      
     Dusk2Dawn localArea(latitude, longitude, timezone);
     unsigned long millisNow = millis();
     sinceMidnight = ((now.tm_hour * 60)+now.tm_min)*60+now.tm_sec;

     sunrise = localArea.sunrise(now.tm_year, now.tm_mon+1, now.tm_mday, false)*60;
     sunset = localArea.sunset(now.tm_year, now.tm_mon+1, now.tm_mday, false)*60;
     DailyEvents[EvSunrise] = millisNow + (sunrise - sinceMidnight) * 1000;
     DailyEvents[EvSunset] = millisNow + (sunset - sinceMidnight) * 1000;
     DailyEvents[EvMidnight] = millisNow + (24 * 60 * 60 - sinceMidnight) * 1000;

#if DEBUG
    char tod[128];
    strftime(tod,128,"%A, %B %d %Y %H:%M:%S",&now) ;
    Serial.println(tod);
     Serial.print("Lat/Lon ");
     Serial.print(latitude);
     Serial.print(",");
     Serial.println(longitude);
     Serial.print("Timezone ");
     Serial.println(timezone);
     Serial.print("Sunset: ");
     Serial.println(sunset);
     Serial.println(sunset/60/60);
     Serial.print("Sunrise: ");
     Serial.println(sunrise);
     Serial.print("sinceMidnight: ");
     Serial.println(sinceMidnight);
#endif
     if (sinceMidnight < sunrise)
     {
       ret = EvSunrise;
     }
     else
     {
       
       if (sinceMidnight < sunset)
       {
         ret = EvSunset;
       }
       else
       {
         ret = EvMidnight;
       }
     }
   } else {
       DailyEvents[EvUnknown] = millis()+10000;
   }

   return ret;
 }

static void onSunset()
{
    for(int i=CHARGER_1;i<NUM_CHARGERS;++i) {
        if(chargerIsValid(i)) {
            ssrSwitch((CHARGERS)i,false);
        }
    }
}

static void onSunrise()
{
    for(int i=CHARGER_1;i<NUM_CHARGERS;++i) {
        if(chargerIsValid(i)) {
            ssrSwitch((CHARGERS)i,true);
        }
    }
}


 DayNightEventType dayNightProceed()
 {
   switch (nextEvent)
   {
   case EvSunrise:
     onSunrise();
     nextEvent = EvSunset;
     break;
   case EvSunset:
     onSunset();
     nextEvent = EvMidnight;
     break;
   case EvMidnight:
   case EvUnknown:
     nextEvent = dayNightCalcEvents();
     break;
   }
    #if DEBUG
    Serial.print("Next event is ");
    Serial.println(nextEvent);
    #endif
   return nextEvent;
 }

void ssrLoop(unsigned long now)  {
    if(gBatteryFull) {
        handleBatteryFull();
    } else if(gBatteryEmpty) {
        handleBatteryEmpty();
    }
    if(DailyEvents[nextEvent] <= now) {
        dayNightProceed();
    }
}

