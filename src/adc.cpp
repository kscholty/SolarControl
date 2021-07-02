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


//i2s number
#define I2S_NUM           (I2S_NUM_0)
//i2s sample rate
#define I2S_SAMPLE_RATE   (40000)
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
#define ADC_AVERAGE_COUNT 8

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
void splitBuffer(const bufStruct *input, size_t startInputIndex, bufStruct outputChannels[2]);
double adcGetVoltage();
void denoiseBuffer(bufStruct *buff);
void calibration(size_t);

// The sum of suared values. Basis for 
// calculation of root...
ICACHE_RAM_ATTR static double  avgCurrentSquareSum = 0;
ICACHE_RAM_ATTR static double  avgVoltageSquareSum = 0;

static TaskHandle_t readTask = NULL;
static xQueueHandle mutex;
static xQueueHandle i2s_event_queue;
ICACHE_RAM_ATTR static double oldCurrentVals[ADC_AVERAGE_COUNT];
ICACHE_RAM_ATTR static double oldVoltageVals[ADC_AVERAGE_COUNT];
int oldCurrentValsPos = 0;
int oldVoltageValsPos = 0;

DBG_SECT(
bool doCalibration = false;
)

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
    WRITE_PERI_REG(SYSCON_SARADC_SAR1_PATT_TAB1_REG,0x0F4F0000);

 // The raw ADC data is written in DMA in inverted form. This add an inversion:
    SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV);

    mutex = xSemaphoreCreateMutex();

    //adcReadCalibrationData();

    memset(oldCurrentVals,0,ADC_AVERAGE_COUNT*sizeof(*oldCurrentVals));
    memset(oldVoltageVals,0,ADC_AVERAGE_COUNT*sizeof(*oldVoltageVals));
     xTaskCreate(readAdc, "ADC read", 4096, 0, 5, &readTask);
     return true;
}

void readValues(bufStruct *recBuffer, bufStruct *results)
{
    system_event_t evt;
    uint16_t *buffer = recBuffer->buffer;
    size_t read = 0;
    size_t oldCount;
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
                    oldCount = recBuffer->count;
                    recBuffer->count += read / sizeof(uint16_t);   
                    splitBuffer(recBuffer, oldCount,  results);     
                    buffer = recBuffer->buffer + recBuffer->count;            
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
    vTaskDelay(5000 / portTICK_RATE_MS);

    i2s_adc_enable(I2S_NUM);
    i2s_start(I2S_NUM);
    stopAdcUsage = false;
    while (!stopAdcUsage)
    {
        DBG_SECT(
            if (doCalibration)
            {
                calibration(100);
            } else
        )
        {
            results[0].count = 0;
            results[1].count = 0;
            readValues(i2s_read_buff, results);
            //splitBuffer(i2s_read_buff, 0,  results);
            //printBuffer(i2s_read_buff);
            calculate(&results[0], &results[1]);
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

#ifndef DEBUG_DISABLED
bool calculateCalibrationValues(bufStruct *values, double &minV, double &maxV, double &average) {

    denoiseBuffer(values);
    maxV = 0; 
    minV = 4096;
    average = 0;
    
    for(size_t i = 0;i<values->count;++i) {
        uint16_t value = values->buffer[i];
        double val = esp_adc_cal_raw_to_voltage(value, adc_chars);
        maxV = max (maxV,val);
        minV = min(minV,val);       
        average += val;
    }
    average /= values->count;    
    return true;
}

void calibration(size_t iterations = 100) {
    
    double avgMax[2] = {0,0};
    double avgMin[2] = {0,0};
    double avgAvg[2] = {0,0};

    bufStruct *i2s_read_buff = (bufStruct *)calloc(1, sizeof(bufStruct));
    bufStruct *results = (bufStruct *)calloc(2, sizeof(bufStruct));
    for(int i = 0; i<iterations;++i) {
        double min,max,avg;
        readValues(i2s_read_buff, results);        
        calculateCalibrationValues(&results[0],min,max,avg);
        avgMax[0] += max;
        avgMin[0] += min;
        avgAvg[0] += avg;

        calculateCalibrationValues(&results[1],min,max,avg);
        avgMax[1] += max;
        avgMin[1] += min;
        avgAvg[1] += avg;
    }

    free(i2s_read_buff);
    free(results);
    for(int i = 0;i<2;++i) {
        avgMax[i] /= iterations;
        avgMin[i] /= iterations;
        avgAvg[i] /= iterations;

        DBG_SECT(
            Debug.printf("Min: %.3f, Max: %.3f, avg: %.3f, range %.3f, median: %.3f", avgMin[i], avgMax[i], avgAvg[i], avgMax[i]-avgMin[i], (avgMax[i]-avgMin[i])/2);
      )
    }
}

void printBuffer(bufStruct *i2s_read_buff) {
    size_t count = i2s_read_buff->count ;
   
    Serial.printf("Read: %d values\n",count);
    for(int i=0;i<count;++i) {
        uint16_t value = i2s_read_buff->buffer[i];
        double val = esp_adc_cal_raw_to_voltage(value, adc_chars);
        Serial.printf("%d : %x : %.3f\n",value, value, val);       
    }
    Serial.println("------------------");
}
#endif

void splitBuffer(const bufStruct *input, size_t startInputIndex, bufStruct outputChannels[2]) {
    
    for(size_t i=startInputIndex;i<input->count;++i) {
        uint16_t value = input->buffer[i];
        int index = value>>14; // 12 bytes are the value, then shift 2 more to see if its 4 or 0
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
        buff->buffer[i] = ((float)buff->buffer[index] + (float)buff->buffer[index + 1] + 
                           (float)buff->buffer[index + 2] + (float)buff->buffer[index + 3]) / 4.0f + 0.5f;
    }
    buff->count = count;
}

void denoiseBuffer_2(bufStruct *buff) {
    
    size_t count = buff->count;
     count = count >> 1; // Calculate average of 4 measurements
    for (size_t i = 0; i < count; ++i)
    {
        // Reduce noise
        size_t index = i << 1;
        buff->buffer[i] = ((float)buff->buffer[index] + (float)buff->buffer[index + 1] ) / 2.0f + 0.5f;
    }
    buff->count = count;
}

void calculateCurrent(bufStruct *currentBuf) 
{
    double sum = 0.0;
DBG_SECT(
        double avgRaw = 0;
        double avgAdj = 0;
        static unsigned int counter = 0;
        size_t start = millis();
)
    uint16_t *buffer  = currentBuf->buffer;
    for (size_t i = 0; i <currentBuf->count; ++i, ++buffer)
    {
        //uint32_t value = adcCalibrateValue(*buffer);
        uint32_t value = *buffer;
        // Get voltage
        double val = esp_adc_cal_raw_to_voltage(value, adc_chars);
        //double val = value;
        if(val<0) val = 0;
DBG_SECT (
        avgRaw += val;
)

        // We know that the voltage is between 0 and 3333mV
        // So 1666mV should be 0A In Fact 1.64mV gives us a reading of 1631
        // We can assume 1631 == 0A. So
        // (val-1631)/1631*10
        #define ZERO_V 1631
        #define MAX_A 6.66
        val = val * (MAX_A / ZERO_V) - MAX_A;
        #undef ZERO_V
        #undef MAX_A
DBG_SECT (
        avgAdj += val;
)     
        sum += (val * val);
    }
DBG_SECT (
    if(Debug.isActive(Debug.DEBUG)) {
    if(!counter) {
        Debug.printf("AvgRaw: %lf AvgAdj %lf\n",avgRaw/currentBuf->count,avgAdj/currentBuf->count); 
        Debug.println("---------------");
    }
    }
)
   

    if ( currentBuf->count)
    {
                
        sum = sum / (currentBuf->count*ADC_AVERAGE_COUNT);

        xSemaphoreTake(mutex, portMAX_DELAY);
        avgCurrentSquareSum = avgCurrentSquareSum - oldCurrentVals[oldCurrentValsPos] + sum;
        xSemaphoreGive(mutex);
        oldCurrentVals[oldCurrentValsPos] = sum;
        oldCurrentValsPos = (oldCurrentValsPos+1) % ADC_AVERAGE_COUNT;
    }

    DBG_SECT(
        if (Debug.isActive(Debug.DEBUG))
        {
            if (!counter)
            {
                Debug.printf("calc took %ld\n", millis() - start);
            }
        }
        #define STEPWIDTH 100
   	    counter = (counter +1 ) % STEPWIDTH;
        )
}

void calculateVoltage(bufStruct *voltageBuf) 
{
    double sum = 0.0;
    static double avgRaw = 0;
    static double avgAdj = 0;
    static double maxRaw = 0;
    static double minRaw = 4096.0;
    static int counter = 0;
DBG_SECT(
    unsigned long start = millis();
)
    uint16_t *buffer  = voltageBuf->buffer;
    for (size_t i = 0; i <voltageBuf->count; ++i, ++buffer)
    {
        //uint32_t value = adcCalibrateValue(*buffer);        
        // Get voltage
        double val = esp_adc_cal_raw_to_voltage(*buffer, adc_chars);
        //double val = *buffer;
        //if(val<0) val = 0;
DBG_SECT (
        avgRaw += val;
        maxRaw = max(maxRaw,val);    
        minRaw = min(minRaw,val);    
)

        // We know that the voltage is between 0 and 3333mV
        // So 1666mV should be 0V In Fact 1.64mV gives us a reading of 1631
        // We can assume 1631 == 0V. So
        // (val-1631)/1631*MAXV
        #define ZERO_V 1641.25
        #define MAX_VAL 406.0
        #define MAX_V 325.0
        
        val = (val-ZERO_V)/MAX_VAL * MAX_V;

        //val = val * (MAX_V / ZERO_V) - MAX_V;
        #undef ZERO_V
        #undef MAX_V
#if 1
        avgAdj += val;
#endif        
        sum += (val * val);
    }
#if 1
    #define STEPWIDTH 100
   	counter = (counter +1 ) % STEPWIDTH;
    if(!counter) {
        Serial.print("AvgRaw: "); Serial.print(avgRaw/(STEPWIDTH*voltageBuf->count)); Serial.print(" AvgAdj: "); Serial.println(avgAdj/(STEPWIDTH*voltageBuf->count));
        Serial.print("MinRaw: "); Serial.print(minRaw); Serial.print(" MaxRaw: "); Serial.println(maxRaw);
        minRaw = maxRaw; maxRaw = 0;
        avgRaw = 0; avgAdj = 0;
        Serial.println("---------------");
    }
#endif
   

    if ( voltageBuf->count)
    {
                
        sum = sum / (voltageBuf->count*ADC_AVERAGE_COUNT);

        xSemaphoreTake(mutex, portMAX_DELAY);
        avgVoltageSquareSum = avgVoltageSquareSum - oldVoltageVals[oldVoltageValsPos] + sum;
        xSemaphoreGive(mutex);
        oldVoltageVals[oldVoltageValsPos] = sum;
        oldVoltageValsPos = (oldVoltageValsPos+1) % ADC_AVERAGE_COUNT;
    }
    static double avgVoltage =0.0;
    if(!counter) {
        Serial.print("Voltage "); Serial.println(avgVoltage/STEPWIDTH);
        avgVoltage = adcGetVoltage() ;
    } else {
        avgVoltage += adcGetVoltage();
    }

DBG_SECT(
    if(!counter) {        
        DEBUG_D("calc took %ld", millis()-start);
    }
)
}

void calculate(bufStruct *currentBuf, bufStruct *voltageBuf)
{


    denoiseBuffer(currentBuf);
    denoiseBuffer(voltageBuf);

    //printBuffer(voltageBuf);
    //printBuffer(currentBuf);
    //delay(3000);
    calculateVoltage(voltageBuf);
    return;
}


double adcGetVoltage()
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    double val = avgVoltageSquareSum;
    xSemaphoreGive(mutex);   
    return sqrt(val);
}

double adcGetCurrent()
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    double val = avgCurrentSquareSum;
    xSemaphoreGive(mutex);   
    return sqrt(val);
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
