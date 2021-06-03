#include <Arduino.h>

#include <driver/adc.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include <driver/i2s.h>
#include <soc/syscon_reg.h>
#include <esp_event_loop.h>
#include <esp_adc_cal.h>

#include "common.h"


//i2s number
#define I2S_NUM           (I2S_NUM_0)
//i2s sample rate
#define I2S_SAMPLE_RATE   (32000)
//i2s data bits
#define I2S_SAMPLE_BITS   (I2S_BITS_PER_SAMPLE_16BIT)
//I2S read buffer length
#define READ_LEN      (sizeof(int16_t) * I2S_SAMPLE_RATE / 100)

//I2S data format
#define I2S_FORMAT        (I2S_CHANNEL_FMT_ONLY_RIGHT)
//I2S channel number
#define I2S_CHANNEL_NUM   ((I2S_FORMAT < I2S_CHANNEL_FMT_ONLY_RIGHT) ? (2) : (1))
//I2S built-in ADC unit
#define I2S_ADC_UNIT       ADC_UNIT_1
//I2S built-in ADC channel
#define I2S_ADC_CURRENT_CHANNEL     ADC1_CHANNEL_4
#define I2S_ADC_VOLTAGE_CHANNEL     ADC1_CHANNEL_5

// how many buffers do we want to use for DMA
#define ADC_BUFFER_COUNT   2

// How many old values do we want to use
// to get a more stable reading
#define ADC_AVERAGE_COUNT 5



typedef uint16_t buffer_t[READ_LEN / sizeof(int16_t)];
struct bufStruct {
    bufStruct() { count = 0;}
    size_t count;
    buffer_t buffer;
};

IRAM_ATTR void readAdc(void *);
void calculate(bufStruct *);
void printBuffer(bufStruct *);
void splitBuffer(const bufStruct *input, bufStruct outputChannels[2]);

// The sum of suared values. Basis for 
// calculation of root...
ICACHE_RAM_ATTR static double  avgSquareSum = 0;

static TaskHandle_t readTask = NULL;
static xQueueHandle mutex;
static xQueueHandle i2s_event_queue;
ICACHE_RAM_ATTR static double oldVals[ADC_AVERAGE_COUNT];
int oldValsPos = 0;

static volatile bool stopAdcUsage=false;
esp_adc_cal_characteristics_t *adc_chars;

#if 0
hw_timer_t * timer = NULL;

template <class T>
class RingBuffer
{
public:
    RingBuffer(size_t size=1000);
    ~RingBuffer();
    T &head();
    T get();
    void push(const T& value);
    size_t count();
    bool empty() { return (begin == end);};
private:
T *buffer;
T *begin,*end, *wrap;
};

template <class T>
RingBuffer<T>::RingBuffer(size_t size) 
{
    buffer = new T[size+1];
    begin = end = buffer;
    wrap = buffer+size+1;
}

template <class T>
IRAM_ATTR T& RingBuffer<T>::head() {
    return *begin;
}

template <class T>
IRAM_ATTR T RingBuffer<T>::get() {
    T result = *begin;
    if(!empty()) {
        ++begin;
        if(begin >= wrap) {
            begin = buffer;
        }
    }
    return result;
}

template <class T>
RingBuffer<T>::~RingBuffer() 
{ 
    delete [] buffer;
}
    
template <class T>
IRAM_ATTR void RingBuffer<T>::push(const T& value) {
    *end = value;
    if(++end >= wrap) {
        end = buffer;
    }
     if(begin == end) {
        ++begin;
        if(begin >= wrap) {
            begin = buffer;
        }
    }
}

template <class T>
size_t RingBuffer<T>::count()
{
    size_t result;
    if (end >= begin)
    {
        result = end - begin;
    }
    else
    {
        result = begin - end + wrap - begin;
    }
    return result;
}

#define BUFSIZE 200
DRAM_ATTR  RingBuffer<double> buffer(BUFSIZE);
DRAM_ATTR double sum= 0.0;
unsigned int count=0;
portMUX_TYPE DRAM_ATTR timerMux = portMUX_INITIALIZER_UNLOCKED; 

int IRAM_ATTR local_adc1_read(int channel) {
    uint16_t adc_value;
    SENS.sar_meas_start1.sar1_en_pad = (1 << channel); // only one channel is selected
    while (SENS.sar_slave_addr1.meas_status != 0);
    SENS.sar_meas_start1.meas1_start_sar = 0;
    SENS.sar_meas_start1.meas1_start_sar = 1;
    while (SENS.sar_meas_start1.meas1_done_sar == 0);
    adc_value = SENS.sar_meas_start1.meas1_data_sar;
    return adc_value;
}

double IRAM_ATTR calcI(int raw) {
    // Raw is between 0-4095 This is a value between -10A and 10A
    // So 2048 is 0. So we calculate (raw -2048)/2048*10 
    double result = (double) raw / 204.8d - 10;
    Serial.print("RESULT "); Serial.println(result);
    return result * result;
}

void IRAM_ATTR onTimer() {
    //portENTER_CRITICAL_ISR(&timerMux);
    //int value = local_adc1_read(0);
    int value = adc1_get_raw(ADC1_CHANNEL_0);
    //double newVal = calcI(value);

    //double newSum = sum-buffer.head()+newVal;
    //sum = newSum;
    //buffer.push(newVal);
    //portEXIT_CRITICAL_ISR(&timerMux);
}



void adcInit()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 20000, true);

    for (int i = 0; i < BUFSIZE; ++i)
    {
        int value = adc1_get_raw(ADC1_CHANNEL_0);
        buffer.push(calcI(value));
        delayMicroseconds(4000);
    }
    timerAlarmEnable(timer);
    count = BUFSIZE;
}
#endif



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
     //i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CURRENT_CHANNEL);#

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

    memset(oldVals,0,ADC_AVERAGE_COUNT*sizeof(*oldVals));
     xTaskCreate(readAdc, "ADC read", 4096, 0, 5, &readTask);
     return true;
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
        system_event_t evt;
        if (xQueueReceive(i2s_event_queue, &evt, portMAX_DELAY) == pdPASS)
        {
            if (evt.event_id == (system_event_id_t)I2S_EVENT_RX_DONE)
            {
                {
                    //read data from I2S bus, in this case, from ADC.
                    i2s_read(I2S_NUM, i2s_read_buff->buffer, READ_LEN, &i2s_read_buff->count, 0);
                    i2s_read_buff->count /= sizeof(uint16_t);
                    splitBuffer(i2s_read_buff, results);
                    //printBuffer(i2s_read_buff);
                    //calculate(i2s_read_buff);
                }
            }
            else
            {
                Serial.println("Could not get buffer");
            }
        }
    }

    Serial.println(" Adc task exiting... ");
    // Clean up and exit
    i2s_adc_disable(I2S_NUM);
    i2s_stop(I2S_NUM);
    i2s_driver_uninstall(I2S_NUM);
    readTask = 0;
    vTaskDelete(xTaskGetCurrentTaskHandle());
}

void printBuffer(bufStruct *i2s_read_buff) {
    size_t count = i2s_read_buff->count ;
   
    Serial.printf("Read: %d values\n",count);
    for(int i=0;i<10;++i) {
        Serial.printf("%d: %x\n",i2s_read_buff->buffer[i]>>12,i2s_read_buff->buffer[i] & 0xFFF);       
    }
    Serial.println("------------------");
}

void splitBuffer(const bufStruct *input, bufStruct outputChannels[2]) {
    outputChannels[0].count = 0;
    outputChannels[1].count = 0;
    for(size_t i=0;i<input->count;++i) {
        uint16_t value = input->buffer[i];
        int index = value>>14; // 12 bytes are the value, then shift 2 more to see if its 4 or 0
        outputChannels[index].buffer[outputChannels[index].count++] = value & 0xFFF;
    }

    Serial.printf("Counts: 0:%d, 4:%d\n",outputChannels[0].count, outputChannels[1].count);
}

void calculate(bufStruct *i2s_read_buff)
{

    double sum = 0.0;

#if DEBUG
    unsigned long start = millis();
#endif 

    size_t count = i2s_read_buff->count / sizeof(int16_t);
    uint16_t *buffer = (uint16_t *)i2s_read_buff->buffer;
    count = count >> 2; // Calculate average of 4 measurements
    for (size_t i = 0; i < count; ++i)
    {
        // Reduce noise
        size_t index = i << 2;
        buffer[i] = ((float)buffer[index] + (float)buffer[index + 1] + (float)buffer[index + 2] + (float)buffer[index + 3]) / 4.0f + 0.5f;
    }
#if DEBUG
    static size_t counter = 0;
    if (++counter == 200)
        counter = 0;
    
    double avgRaw = 0;
    double avgAdj = 0;
#endif
    
    for (size_t i = 0; i < count; ++i, ++buffer)
    {
        //uint32_t value = adcCalibrateValue(*buffer);
        uint32_t value = *buffer;
        // Get voltage
        double val = esp_adc_cal_raw_to_voltage(value, adc_chars);
        //double val = value;
        if(val<0) val = 0;
#if DEBUG
        avgRaw += val;    
#endif

        // We know that the voltage is between 0 and 3333mV
        // So 1666mV should be 0A In Fact 1.64mV gives us a reading of 1631
        // We can assume 1631 == 0A. So
        // (val-1631)/1631*10
        #define ZERO_V 1631
        #define MAX_A 6.66
        val = val * (MAX_A / ZERO_V) - MAX_A;
#if DEBUG
        avgAdj += val;
#endif        
        sum += (val * val);
    }
#if DEBUG
    if(!counter) {
        Serial.print("AvgRaw: "); Serial.print(avgRaw/count); Serial.print(" AvgAdj: "); Serial.println(avgAdj/count);
        Serial.println("---------------");
    }
#endif
    i2s_read_buff->count = 0;

    if (count)
    {
                
        sum = sum / (count*ADC_AVERAGE_COUNT);

        xSemaphoreTake(mutex, portMAX_DELAY);
        avgSquareSum = avgSquareSum - oldVals[oldValsPos] + sum;
        xSemaphoreGive(mutex);
        oldVals[oldValsPos] = sum;
        oldValsPos = (oldValsPos+1) % ADC_AVERAGE_COUNT;
    }
    #if DEBUG
    if(!counter) {
        unsigned long duration = millis()-start;
        Serial.print("calc took "); Serial.println(duration);
    }
    
    #endif
}

double adcGetCurrent()
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    double val = avgSquareSum;
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
