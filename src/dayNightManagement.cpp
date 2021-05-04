#include <Arduino.h>
#include <Dusk2Dawn.h>

#include "common.h"
#include "ssrManagement.h"

enum DayNightEventType {
  EvSunrise = 0,
  EvSunset = 1,
  EvMidnight = 2,  
  EvUnknown
};


long DailyEvents[3];

 static DayNightEventType nextEvent = EvUnknown;
 static Dusk2Dawn localArea(34.0522, -118.2437, -8);

 DayNightEventType calcEvents()
 {
   struct tm now, midnight;
   time_t sinceMidnight;
   int sunrise, sunset;
   DayNightEventType ret = EvUnknown;

   if (getLocalTime(&now))
   {     
     sinceMidnight = ((now.tm_hour * 60)+now.tm_min)*60+now.tm_sec;

     sunrise = localArea.sunrise(now.tm_year, now.tm_mon, now.tm_mday, false);
     sunset = localArea.sunset(now.tm_year, now.tm_mon, now.tm_mday, false);
     DailyEvents[EvSunrise] = (sunrise-sinceMidnight) * 1000;
     DailyEvents[EvSunset] = (sunset-sinceMidnight) * 1000;
     DailyEvents[EvMidnight] = (24*60*60-sinceMidnight) * 1000;

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
   }

   return ret;
 }



bool day() {
    struct tm now,midnight;
    double lat, lon;
    int sunrise,sunset;
    time_t sinceMidnight;


    if (getLocalTime(&now))
    {
      midnight = now;
      midnight.tm_hour = 0;
      midnight.tm_min = 0;
      midnight.tm_sec = 0;
      sinceMidnight = difftime(mktime(&now), mktime(&midnight));

       sunrise = localArea.sunrise(now.tm_year, now.tm_mon, now.tm_mday, false);
       sunset = localArea.sunset(now.tm_year, now.tm_mon, now.tm_mday, false);
    }
      return (sinceMidnight > sunrise && sinceMidnight < sunset);
}


