#include <Arduino.h>

#include <driver/adc.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include <driver/i2s.h>
#include <soc/syscon_reg.h>
#include <esp_event_loop.h>
#include <esp_adc_cal.h>

#include "common.h"
#include "debugManagement.h"
#include "adc.h"

//i2s number
#define I2S_NUM           (I2S_NUM_0)
//i2s sample rate
#define I2S_SAMPLE_RATE   (64000)
//i2s data bits
#define I2S_SAMPLE_BITS   (I2S_BITS_PER_SAMPLE_16BIT)
//I2S read buffer length
#define READ_LEN      (sizeof(int16_t) * I2S_SAMPLE_RATE / 200)

//I2S data format
#define I2S_FORMAT        (I2S_CHANNEL_FMT_ONLY_RIGHT)
//I2S channel number
#define I2S_CHANNEL_NUM   ((I2S_FORMAT < I2S_CHANNEL_FMT_ONLY_RIGHT) ? (2) : (1))
//I2S built-in ADC unit
#define I2S_ADC_UNIT       ADC_UNIT_1
//I2S built-in ADC channel
#define I2S_ADC_CURRENT_CHANNEL     ADC1_CHANNEL_0
#define I2S_ADC_VOLTAGE_CHANNEL     ADC1_CHANNEL_5

// how many buffers do we want to use for DMA
#define ADC_BUFFER_COUNT   2

// How many old values do we want to use
// to get a more stable reading
#define ADC_AVERAGE_COUNT 16

#define SAMPLESIZE 2


typedef uint16_t buffer_t[SAMPLESIZE * READ_LEN / sizeof(int16_t)];
struct bufStruct {
    bufStruct() { count = 0;}
    size_t count;
    buffer_t buffer;
};

IRAM_ATTR void readAdc(void *);
void calculate(bufStruct *currentBuf, bufStruct *voltageBuf);
void printBuffer(bufStruct *);
void splitBuffer(const bufStruct *input, bufStruct outputChannels[2]);
double adcGetVoltage();
void denoiseBuffer(bufStruct *buff);
void calibration(size_t);


static TaskHandle_t readTask = NULL;
static xQueueHandle mutex;
static xQueueHandle i2s_event_queue;

enum ValueType {
    CURRENT = 0,
    VOLTAGE = 1,
    POWER = 2,
    MaxValueType
};


// The sum of suared values. Basis for 
// calculation of root...
ICACHE_RAM_ATTR static double  avgValueSquareSum[MaxValueType] = {0.0,0.0,0.0};
ICACHE_RAM_ATTR static double oldVals[ADC_AVERAGE_COUNT][MaxValueType];

static const int32_t avgCalibration[2]={1632,1670};
static const int32_t maxCalibration[2]={1513,427};
static const double maxVals[2] = {6.66,325.0};

int oldValsPos[MaxValueType] = {0,0,0};

bool doCalibration = true;

static volatile bool stopAdcUsage=false;
esp_adc_cal_characteristics_t *adc_chars;


void characterize() {

//Characterize ADC at particular atten
    adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(I2S_ADC_UNIT, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, adc_chars);
    //Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        Serial.println("eFuse Vref");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        Serial.println("Two Point");
    } else {
        Serial.println("Default");
    }
}

bool adcInit()
{
     i2s_config_t i2s_config = {
        (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_RX |  I2S_MODE_ADC_BUILT_IN),
        I2S_SAMPLE_RATE,
        I2S_SAMPLE_BITS,
        I2S_FORMAT,
        I2S_COMM_FORMAT_I2S,
        ESP_INTR_FLAG_LEVEL1,
        ADC_BUFFER_COUNT,
        READ_LEN,
        2,
        true,
        false
     };

    
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(I2S_ADC_CURRENT_CHANNEL, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(I2S_ADC_VOLTAGE_CHANNEL, ADC_ATTEN_DB_11);

     
     //install and start i2s driver
     i2s_driver_install(I2S_NUM, &i2s_config, ADC_BUFFER_COUNT, &i2s_event_queue);
     
     //init ADC pad
     //i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CURRENT_CHANNEL);

     // This is the trick that makes it work. 
     // You have to call it with a 0 here. Not calling it won't work, but calling it with a correct
     // adc_unit will cause the unit to use only the set channel
     i2s_set_adc_mode((adc_unit_t)0, I2S_ADC_CURRENT_CHANNEL);
    
    // Length of the pattern table  (-1 obviously)
    SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR1_PATT_LEN, 1, SYSCON_SARADC_SAR1_PATT_LEN_S);
    // Channels 0 and 5 with 12db Attenuation and length of 12 bit
    WRITE_PERI_REG(SYSCON_SARADC_SAR1_PATT_TAB1_REG,0x0F5F0000);

 // The raw ADC data is written in DMA in inverted form. This add an inversion:
    SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV);

    mutex = xSemaphoreCreateMutex();

    //adcReadCalibrationData();

    memset(oldVals,0,ADC_AVERAGE_COUNT*MaxValueType*sizeof(oldVals[0][0]));
    
     xTaskCreate(readAdc, "ADC read", 4096, 0, 5, &readTask);
     return true;
}

void readValues(bufStruct *recBuffer, bufStruct *results)
{
    system_event_t evt;
    uint16_t *buffer = recBuffer->buffer;
    size_t read = 0;
    recBuffer->count = 0;
    for (int i = 0; i < SAMPLESIZE; ++i)
    {
        if (xQueueReceive(i2s_event_queue, &evt, portMAX_DELAY) == pdPASS)
        {
            if (evt.event_id == (system_event_id_t)I2S_EVENT_RX_DONE)
            {
                {
                    //read data from I2S bus, in this case, from ADC.
                    i2s_read(I2S_NUM, buffer, READ_LEN, &read, 0);                    
                    recBuffer->count = read / sizeof(uint16_t);   
                    splitBuffer(recBuffer,  results);                         
                }
            }            
        }
    }
   // Serial.printf("Read %d values\n",recBuffer->count);
}

void readAdc(void *buffer)
{

    bufStruct *i2s_read_buff = (bufStruct *)calloc(1, sizeof(bufStruct));
    bufStruct *results = (bufStruct *)calloc(2, sizeof(bufStruct));

    characterize();

    // ADC-I2S Scanning does not normal start (more badly) after the reset of ESP32.
    // It is necessary to add a delay of at least 5 seconds before switching on the ADC for a reliable start, this is issue.
    vTaskDelay(7000 / portTICK_RATE_MS);

    i2s_adc_enable(I2S_NUM);
    i2s_start(I2S_NUM);
    stopAdcUsage = false;
    while (!stopAdcUsage)
    {
        if (doCalibration)
        {
            calibration(3000);
            doCalibration = false;
        }
        else
        {

            results[CURRENT].count = 0;
            results[VOLTAGE].count = 0;
            readValues(i2s_read_buff, results);  
            //printBuffer(i2s_read_buff);
            calculate(&results[CURRENT], &results[VOLTAGE]);
        }
    }

    DEBUG_I(" Adc task exiting... ");
    // Clean up and exit
    i2s_adc_disable(I2S_NUM);
    i2s_stop(I2S_NUM);
    i2s_driver_uninstall(I2S_NUM);
    readTask = 0;
    vTaskDelete(xTaskGetCurrentTaskHandle());
}

bool calculateCalibrationValues(bufStruct *values, double &minV, double &maxV, double &average, int32_t & offset) {

    denoiseBuffer(values);
    maxV = 0; 
    minV = 4096;
    average = 0;

    #define FILTERSHIFT 13     // for low pass filters to determine ADC offsets
    #define FILTERROUNDING (1<<12)

    unsigned long fOffset = (unsigned long)offset << FILTERSHIFT;

    for(size_t i = 0;i<values->count;++i) {
        uint16_t value = values->buffer[i];
        double val = esp_adc_cal_raw_to_voltage(value, adc_chars);
        //double val = value;

        maxV = max (maxV,val);
        minV = min(minV,val);       
        average += val;
        val = val - offset;
        fOffset += val;
        offset = (uint32_t) ((fOffset+FILTERROUNDING)>>FILTERSHIFT);
    }
    average /= values->count;    
    return true;
}

void calibration(size_t iterations = 100) {
    
    double avgMax[2] = {0,0};
    double avgMin[2] = {0,0};
    double avgAvg[2] = {0,0};
    int32_t offsets[2] = {avgCalibration[0],avgCalibration[1]};

    Serial1.println("Starting clibration");
    Serial1.flush();
    bufStruct *i2s_read_buff = (bufStruct *)calloc(1, sizeof(bufStruct));
    bufStruct *results = (bufStruct *)calloc(2, sizeof(bufStruct));
    for(int i = 0; i<iterations;++i) {
        double min,max,avg;
       
        readValues(i2s_read_buff, results);        
        calculateCalibrationValues(&results[CURRENT],min,max,avg,offsets[CURRENT]);
        avgMax[CURRENT] += max;
        avgMin[CURRENT] += min;
        avgAvg[CURRENT] += avg;

        calculateCalibrationValues(&results[VOLTAGE],min,max,avg,offsets[VOLTAGE]);      
        avgMax[VOLTAGE] += max;
        avgMin[VOLTAGE] += min;
        avgAvg[VOLTAGE] += avg;
    }

    free(i2s_read_buff);
    free(results);
    for(int i = 0;i<2;++i) {
        avgMax[i] /= iterations;
        avgMin[i] /= iterations;
        avgAvg[i] /= iterations;
        //avgCalibration[i] = avgAvg[i];
        //maxCalibration[i] = (avgMax[i]-avgMin[i])/2.0;
        DBG_SECT(
            Debug.printf("Min: %.3f, Max: %.3f, avg: %.3f, range %.3f, median: %.3f offset %d\n\r", avgMin[i], avgMax[i], avgAvg[i], avgMax[i]-avgMin[i], (avgMax[i]-avgMin[i])/2,offsets[i]);
      )
        Serial.printf("Min: %.3f, Max: %.3f, avg: %.3f, range %.3f, median: %.3f offset %d\n\r", avgMin[i], avgMax[i], avgAvg[i], avgMax[i]-avgMin[i], (avgMax[i]-avgMin[i])/2,offsets[i]);
    }
    Serial.println("----------");
}

void printBuffer(bufStruct *i2s_read_buff) {
    size_t count = i2s_read_buff->count ;
   
    Serial.printf("Read: %d values\n",count);
    for(int i=0;i<count;i+=2) {
        uint16_t value = i2s_read_buff->buffer[i];
       double val = esp_adc_cal_raw_to_voltage(value, adc_chars);
        //double val = value;
        Serial.printf("%.3f\n\r",val);               
    }
    /*
    Serial.println("1500.0");
Serial.println("1500.0");
Serial.println("1500.0");
Serial.println("1500.0");
*/
    Serial.println("------------------");
}


void splitBuffer(const bufStruct *input, bufStruct outputChannels[2]) {
    
    for(size_t i=0;i<input->count;++i) {
        uint16_t value = input->buffer[i];
        //Serial.printf("%d: %x : %x\r\n", i,value>>12,value & 0xFFF);
        int index = (value>>14)&0xf; // 12 bytes are the value, then shift 2 more to see if its 5 or 0
        outputChannels[index].buffer[outputChannels[index].count++] = value & 0xFFF;        
    }

    //Serial.printf("Counts: 0:%d, 4:%d\n",outputChannels[0].count, outputChannels[1].count);
}

void denoiseBuffer(bufStruct *buff) {
    
    size_t count = buff->count;
     count = count >> 2; // Calculate average of 4 measurements
    for (size_t i = 0; i < count; ++i)
    {
        // Reduce noise
        size_t index = i << 2;
        buff->buffer[i] = ((uint32_t)buff->buffer[index] + (uint32_t)buff->buffer[index + 1] + 
                           (uint32_t)buff->buffer[index + 2] + (uint32_t)buff->buffer[index + 3]) >> 2 ;
    }
    buff->count = count;
}


double calculateSquareMean(bufStruct *aBuf, ValueType calibrationValueIndex)
{
    double sum = 0.0;
    
    uint16_t *buffer = aBuf->buffer;
    const int32_t zeroValue = avgCalibration[calibrationValueIndex];
    const int32_t maxValue = maxCalibration[calibrationValueIndex];
    const double maxTargetValue = maxVals[calibrationValueIndex];
   // double avgVal=0.0;
    for (size_t i = 0; i < aBuf->count; ++i, ++buffer)
    {
        // Get voltage
        //Serial.printf("%d\r\n",(int)*buffer);
        int32_t val = esp_adc_cal_raw_to_voltage(*buffer, adc_chars);
        //int32_t val = *buffer;
        

        //Project measuerd value into taregt value
        val = (val - zeroValue)  * maxTargetValue / maxValue;   
       // avgVal += val;
        sum += (val * val);
        //Serial.printf("%f\r\n",val);
    }
    if (aBuf->count)
    {
        sum = sum / (aBuf->count * ADC_AVERAGE_COUNT);
    }
    #if 0
    static size_t count = 0;
    if(count % 101 == 0) {
        Serial.printf("AvgSumVal(%d) = %.3f\r\n",calibrationValueIndex,avgVal / aBuf->count);
    }
    count++;
    #endif
    avgValueSquareSum[calibrationValueIndex] = avgValueSquareSum[calibrationValueIndex] - oldVals[oldValsPos[calibrationValueIndex]][calibrationValueIndex] + sum;
    oldVals[oldValsPos[calibrationValueIndex]][calibrationValueIndex] = sum;
    oldValsPos[calibrationValueIndex] = (oldValsPos[calibrationValueIndex] + 1) % ADC_AVERAGE_COUNT;

    return avgValueSquareSum[calibrationValueIndex];
}

void calculateCurrent(bufStruct *currentBuf) 
{
        calculateSquareMean(currentBuf,CURRENT);       
}      

void calculateVoltage(bufStruct *voltageBuf) 
{
     calculateSquareMean(voltageBuf,VOLTAGE);        

}

// We are calculating the constant portion of the sum we want to calculate
// The factor 16 is to enhance precision when we do the divion. This will be removed at the end.
static const double CStar = maxVals[CURRENT]*maxVals[VOLTAGE] / (maxCalibration[CURRENT] * maxCalibration[VOLTAGE]);
static const double CStarDash =  maxVals[CURRENT]*maxVals[VOLTAGE] / (maxCalibration[CURRENT] * maxCalibration[VOLTAGE]) * avgCalibration[CURRENT] * avgCalibration[VOLTAGE] ;

void calculatePower(bufStruct *currentBuf, bufStruct *voltageBuf) {

    size_t count = min(currentBuf->count,voltageBuf->count);
    int32_t sumPower=0;

    for (size_t i = 0; i < count; ++i)
    {
        // Get voltage
        int32_t valCurrent = esp_adc_cal_raw_to_voltage(currentBuf->buffer[i], adc_chars);
        int32_t valVoltage = esp_adc_cal_raw_to_voltage(voltageBuf->buffer[i], adc_chars);

        //Project measuerd value into taregt value
        //valCurrent = (valCurrent - avgCalibration[CURRENT]) * maxVals[CURRENT] / maxCalibration[CURRENT];
        //valVoltage = (valVoltage - avgCalibration[VOLTAGE]) * maxVals[VOLTAGE] / maxCalibration[VOLTAGE] ;
        //sumPower += valCurrent*valVoltage;

        sumPower += valCurrent*valVoltage-valCurrent*avgCalibration[VOLTAGE]-valVoltage*avgCalibration[CURRENT];         
    }

     if (count)
    {        
        //sumPower = sumPower / ((double)count*(double)ADC_AVERAGE_COUNT);
        sumPower = ((CStar*sumPower / count + CStarDash) / ADC_AVERAGE_COUNT);
        //sumPower = sumPower / ADC_AVERAGE_COUNT;
        //sumPower = sumPower / 16; // Remove factor 16 we used above for precision.

        avgValueSquareSum[POWER] = avgValueSquareSum[POWER] - oldVals[oldValsPos[POWER]][POWER] + sumPower;
        
        oldVals[oldValsPos[POWER]][POWER] = sumPower;
        oldValsPos[POWER] = (oldValsPos[POWER]+1) % ADC_AVERAGE_COUNT;
    } 
}

void calculate(bufStruct *currentBuf, bufStruct *voltageBuf)
{

    denoiseBuffer(currentBuf);
    //calculateCurrent(currentBuf);
    denoiseBuffer(voltageBuf);
    //calculateVoltage(voltageBuf);
    calculatePower(currentBuf,voltageBuf);
#if 0
    DBG_SECT(
    static int count=0;
    if(count % 1000 == 0) {
        //Debug.printf("Current is: %.3f\r\n",adcGetCurrent());
        //Debug.printf("Voltage is: %.3f\r\n",adcGetVoltage());
        //Debug.printf("Power is: %.3f (%.3f)\r\n",adcGetPower(),adcGetCurrent()*adcGetVoltage());
        //Serial.printf("Voltage is: %.3f\r\n",adcGetVoltage());
        //Serial.printf("Serial is: %.3f\r\n",adcGetCurrent());

    }
    count++;
    )
#endif
    return;
}


double adcGetVoltage()
{
    //xSemaphoreTake(mutex, portMAX_DELAY);
    double val = avgValueSquareSum[VOLTAGE];
    //xSemaphoreGive(mutex);   
    return sqrt(val);
}

double adcGetCurrent()
{
    //xSemaphoreTake(mutex, portMAX_DELAY);
    double val = avgValueSquareSum[CURRENT];
    //xSemaphoreGive(mutex);   
    return sqrt(val);
}

double adcGetPower() {
     return avgValueSquareSum[POWER];
}

void adcStop() {
    Serial.print(" Stopping adc task ");
   stopAdcUsage = true;   
   do {
       vTaskDelay( 50 / portTICK_PERIOD_MS);
   } while(readTask)   ;

   Serial.println(" ADC task gone");
}

void adcStart() {
    adcInit();
}
