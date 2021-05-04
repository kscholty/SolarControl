
#if NEVER

#include <arduino.h>
#include <driver/dac.h>
#include <driver/adc.h>
#include <SPIFFS.h>

#include "common.h"
#include "calibration.h"
#include "adc.h"
 
// Uncomment to get Text for including in programm:
//#define GRAPH
#define ADC_PIN 34

float *Results;

static uint16_t *adcCalibrationData = 0;

size_t adcReadCalibrationData()
{
    size_t readTotal = 0;

    Serial.println("Entering adcReadCalibrationData");    
    if (!SPIFFS.begin(false))
    {
        Serial.println("An Error has occurred while mounting SPIFFS for reading calibration data");
        return 0;
    }

    Serial.println("SPIFFS OK");

    File file = SPIFFS.open("/calibration.bin", FILE_READ);    

    if (file)
    {
        size_t read = 0;
        size_t toRead = 1024 * sizeof(uint16_t);

        if (!adcCalibrationData)
        {
            adcCalibrationData = (uint16_t *)malloc(1024 * sizeof(uint16_t));
        }

        uint8_t *readBuffer = (uint8_t *)adcCalibrationData;

        Serial.println("Reading");
        do
        {
            read = file.read(readBuffer, toRead);
            toRead -= read;
            readBuffer += read;
            readTotal += read;
        } while (read && toRead);
        if(readTotal  != 1024 * sizeof(uint16_t)) {
            Serial.print("Read only "); Serial.print(readTotal); Serial.println(" bytes");
        } else {
            Serial.print("Read all "); Serial.print(readTotal); Serial.println(" bytes");
        }
    }
    else
    {
        Serial.println("Failed to open calibration data");
    }

    file.close();
    SPIFFS.end();
    return readTotal;
}

bool saveToFile() {

    bool res = true;

    Serial.println("Entering adcWriteCalibrationData");    

    if(!adcCalibrationData) {
        return false;
    }

    if(!SPIFFS.begin(true)){
      Serial.println("An Error has occurred while mounting SPIFFS for writing calibration data");
      return false;
    }
    File file = SPIFFS.open("/calibration.bin", FILE_WRITE);

    size_t written = 0;
    size_t toWrite = 1024*sizeof(uint16_t);
    uint8_t *writeBuffer = (uint8_t*)adcCalibrationData;
    
    do {
        written = file.write(writeBuffer,toWrite);
        toWrite -=written;
        writeBuffer += written;
    } while(toWrite && written);

    if(toWrite) {
        Serial.println("Could not write calibration data");
        res = false;
    } else {
        Serial.println("Wrote successfully");
    }
    file.flush();
    file.close();
    SPIFFS.end();

    return res;
}

void Dotest(int i) {
    uint16_t a1;
  //Serial.print("DoTest "); Serial.println(j);
  dac_output_voltage(DAC_CHANNEL_1, (i) & 0xff);
  delay(100);
  for (int j=0;j<500;j++) {                    
      a1 = analogRead(ADC_PIN);      
      Results[i*16]=0.9*Results[i*16] + 0.1*a1;
      delayMicroseconds(100);
      //Results[i*16]=a1;
  }
}

void setupCalibration()
{

    adcStop();

    dac_output_enable(DAC_CHANNEL_1); // pin 25
    dac_output_voltage(DAC_CHANNEL_1, 0);
    analogSetWidth(12);
    analogReadResolution(12);
    adcAttachPin(ADC_PIN);

    Results = (float *)calloc(4096, sizeof(float));        
    if(!adcCalibrationData) {
        adcCalibrationData = (uint16_t *)calloc(1024, sizeof(uint16_t));
    } else {
        memset(adcCalibrationData,0,1024*sizeof(uint16_t));
    }
}

void finalizeCalibration() {
    
    saveToFile();

    dac_output_voltage(DAC_CHANNEL_1, 0);
    dac_output_disable(DAC_CHANNEL_1); // pin 25
    adcAttachPin(35);
    

    adcStart();   
}

void calibrate(void *) {

setupCalibration();

#if DEBUG
    Serial.println("Test Liniarity ...");
#endif

    for (int i=0; i<256; i++) {
        if ((i%10)==0) Serial.println(i);
        Dotest(i);
    }

#if DEBUG
    for (int i=0; i<256; i++) {       
       Serial.print(i*16); Serial.print(" "); Serial.println(Results[i*16]); 
    }     
    Serial.println("Calculate interpolated values ..");
#endif

    for (int i=0; i<256; i++) {
        float step;
        if(i<255)         
            step = (Results[(i+1)*16] - Results[i*16]) / 16.0f;
        else 
            step = (4095 - Results[i*16]) / 16.0f;
       for (int j=1; j<16; j++) {
          Results[i*16+j] = Results[i*16+j-1]+step;
          
       }
    }

#if 0
    for (int i=0; i<4096; i++) {
        Results[i]=0.5 + Results[i];
        Serial.print(i); Serial.print(" "); Serial.println(Results[i]); 
    } 
#endif
   
    for (int i=0; i<4096; i+=4) {
        float diff, minDiff;
        uint16_t index=i;
        int lower = i-100, upper = i+100;
        index = i;
        //if(lower <0) 
        lower = 0; 
        //if(upper>4096) 
        upper = 4096;
        minDiff=99999;
        for (int j=lower; j<upper; j++) {
            diff=abs((float)(i) - Results[j]);
            if(diff<minDiff) {
                minDiff=diff;
                index=j;
            }
        }

        adcCalibrationData[i/4]=index;
    }

    free(Results);
    Results = 0;

    for (int i=0; i<10; i++) Serial.println();
    Serial.println(" uint16_t ADC_LUT[1024] = { 0,");
    for (int i=1; i<1023; i++) {
      Serial.print((int)adcCalibrationData[i]); Serial.print(",");
      if ((i%16)==0) Serial.println();
    }
    Serial.println((int)adcCalibrationData[1023]); Serial.println("} ;");


   finalizeCalibration();
   vTaskDelete(xTaskGetCurrentTaskHandle());

}

uint16_t adcCalibrateValue(uint16_t val) {

    if(adcCalibrationData) {
        uint index = val>>2; // val /4
        uint dist = val & 0x3; // val % 4

        return (adcCalibrationData[index] + ( (float)adcCalibrationData[index+1] - (float)adcCalibrationData[index] ) * (float)dist / 4.0f + 0.5) ;


    } else {
        return val;
    }

}

void adcStartCalibration()
{
    Serial.print(" Starting calibration task");
    BaseType_t result = xTaskCreate(calibrate, "calibrate", 4096, 0, 1, NULL);
    if (result != pdPASS)
    {
        Serial.print(" Calibration taskCreation failed with error ");
        Serial.println(result);
    }
}



#endif