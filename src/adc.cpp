#include "adc.h"

#include <Arduino.h>
#include <driver/adc.h>
#include <driver/i2s.h>
#include <esp_adc_cal.h>
#include <esp_event.h>
#include <freertos/ringbuf.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include <soc/syscon_reg.h>

#include "ButterworthLPF.h"
#include "common.h"
#include "debugManagement.h"
#include "wifimanagement.h"

// i2s number
#define I2S_NUM (I2S_NUM_0)
// i2s sample rate
#define I2S_SAMPLE_RATE (16000)
// i2s data bits
#define I2S_SAMPLE_BITS (I2S_BITS_PER_SAMPLE_16BIT)

#define NUM_AVG_SAMPLES 1

#define READ_BLOCK (sizeof(int16_t) * I2S_SAMPLE_RATE / 50)
// I2S read buffer length
#define READ_LEN ((NUM_AVG_SAMPLES) * (READ_BLOCK))

// I2S data format
#define I2S_FORMAT (I2S_CHANNEL_FMT_ONLY_RIGHT)
// I2S channel number
#define I2S_CHANNEL_NUM ((I2S_FORMAT < I2S_CHANNEL_FMT_ONLY_RIGHT) ? (2) : (1))
// I2S built-in ADC unit
#define I2S_ADC_UNIT ADC_UNIT_1
// I2S built-in ADC channel
#define I2S_ADC_CURRENT_CHANNEL ADC1_CHANNEL_0
#define I2S_ADC_VOLTAGE_CHANNEL ADC1_CHANNEL_5

// how many buffers do we want to use for DMA
#define ADC_BUFFER_COUNT 4

// How many old values do we want to use
// to get a more stable reading
#define ADC_AVERAGE_COUNT 5

#define SAMPLESIZE 1

// Number of sampleas Voltage is behind Current
// In my case it's 1ms
#define VOLTAGE_OFFSET (I2S_SAMPLE_RATE / 2000)

typedef uint16_t buffer_t[(SAMPLESIZE * READ_LEN + VOLTAGE_OFFSET) / sizeof(int16_t)];
struct bufStruct {
  bufStruct() { count = 0; }
  size_t count;
  buffer_t buffer;
};

static ButterworthLPF lpf1(2, 60, 8000);
static ButterworthLPF lpf2(2, 60, 8000);
static ButterworthLPF *filters[2] = {&lpf1, &lpf2};

static void readAdc(void *);

static void calculate(bufStruct *currentBuf, bufStruct *voltageBuf);

#ifdef PRINT_BUFFER
static void printBuffer(bufStruct *);
static void printBuffer(uint16_t *, size_t);
#endif

static void splitBuffer(const bufStruct *input, bufStruct outputChannels[2]);
double adcGetVoltage();
void calibration(size_t, bufStruct *, bufStruct *);

static TaskHandle_t readTask = NULL;

enum ValueType { CURRENT = 0, VOLTAGE = 1, POWER = 2, MaxValueType };

// The sum of suared values. Basis for
// calculation of root...
ICACHE_RAM_ATTR static double avgValueSquareSum[MaxValueType] = {0.0, 0.0, 0.0};
ICACHE_RAM_ATTR static double oldVals[ADC_AVERAGE_COUNT][MaxValueType];

// Real value = (value-avg)/maxCal*maxVal
static const int32_t avgCalibration[2] = {1524, 1630};
static const int32_t maxCalibration[2] = {1270, 367};
static const double maxVals[2] = {6.66, 337};

static int oldValsPos[MaxValueType] = {0, 0, 0};

bool doCalibration = false;

static volatile bool stopAdcUsage = false;
static esp_adc_cal_characteristics_t *adc_chars;

void characterize() {
  // Characterize ADC at particular atten
  adc_chars = (esp_adc_cal_characteristics_t *)calloc(
      1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
      I2S_ADC_UNIT, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, adc_chars);
  // Check type of calibration value used to characterize ADC
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    Serial.println("eFuse Vref");
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    Serial.println("Two Point");
  } else {
    Serial.println("Default");
  }
}

bool adcInit() {
  i2s_config_t i2s_config = {
      (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
      I2S_SAMPLE_RATE,
      I2S_SAMPLE_BITS,
      I2S_FORMAT,
      I2S_COMM_FORMAT_STAND_I2S,
      ESP_INTR_FLAG_LEVEL1,
      ADC_BUFFER_COUNT,
      READ_LEN / 2 /*READ_LEN / ((I2S_SAMPLE_BITS + 15 ) / 16 * 2 )*/,
      true,
      true,
      0};

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(I2S_ADC_CURRENT_CHANNEL, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(I2S_ADC_VOLTAGE_CHANNEL, ADC_ATTEN_DB_11);

  // install and start i2s driver
  i2s_driver_install(I2S_NUM, &i2s_config, ADC_BUFFER_COUNT,
                     0 /* &i2s_event_queue */);

  // init ADC pad
  // i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CURRENT_CHANNEL);

  // This is the trick that makes it work.
  // You have to call it with a 0 here. Not calling it won't work, but calling
  // it with a correct adc_unit will cause the unit to use only the set channel
  i2s_set_adc_mode((adc_unit_t)0, I2S_ADC_CURRENT_CHANNEL);

  // Length of the pattern table  (-1 obviously)
  SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR1_PATT_LEN, 1,
                    SYSCON_SARADC_SAR1_PATT_LEN_S);
  // Channels 0 and 5 with 12db Attenuation and length of 12 bit
  WRITE_PERI_REG(SYSCON_SARADC_SAR1_PATT_TAB1_REG, 0x0F5F0000);

  // The raw ADC data is written in DMA in inverted form. This add an inversion:
  SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV);

  memset(oldVals, 0, ADC_AVERAGE_COUNT * MaxValueType * sizeof(oldVals[0][0]));

  xTaskCreate(readAdc, "ADC read", 4096, 0, 6, &readTask);
 

  return true;
}

void readValues(bufStruct *recBuffer, bufStruct *results) {
  uint16_t *buffer = recBuffer->buffer;
  size_t read = 0;
  {
    // read data from I2S bus, in this case, from ADC.
    i2s_read(I2S_NUM, buffer, READ_LEN, &read, portMAX_DELAY);
    recBuffer->count = read / sizeof(uint16_t);
    splitBuffer(recBuffer, results);
  }
}

void readAdc(void *buffer) {
  bufStruct *i2s_read_buff = (bufStruct *)calloc(1, sizeof(bufStruct));
  bufStruct *results = (bufStruct *)calloc(2, sizeof(bufStruct));

  characterize();

  // ADC-I2S Scanning does not start right after the reset of
  // ESP32. It is necessary to add a delay of at least 5 seconds before
  // switching on the ADC for a reliable start.
  vTaskDelay(5000 / portTICK_RATE_MS);

  i2s_adc_enable(I2S_NUM);
  i2s_start(I2S_NUM);
  stopAdcUsage = false;
  size_t duration = 0, count = 0, res = 0, readt = 0;
  while (!stopAdcUsage) {
    if (doCalibration) {
      calibration(1000, i2s_read_buff, results);
      doCalibration = false;
    } else {
      results[CURRENT].count = 0;
      results[VOLTAGE].count = VOLTAGE_OFFSET;  // Shift the values according to the ofdfsets
      DBG_SECT(int64_t start = esp_timer_get_time();)
      readValues(i2s_read_buff, results);
#ifdef PRINT_BUFFER      
       printBuffer(i2s_read_buff);
#endif
      DBG_SECT(int64_t startC = esp_timer_get_time();)
      calculate(&results[CURRENT], &results[VOLTAGE]);
      // Put the last values into the front of the buffer for the next
      // iteration.
      memmove(
          results[VOLTAGE].buffer,
          results[VOLTAGE].buffer + (results[VOLTAGE].count - VOLTAGE_OFFSET),
          VOLTAGE_OFFSET * sizeof(uint16_t));

#ifndef DEBUG_DISABLED
      if (Debug.isActive(Debug.INFO)) {
        int64_t now = esp_timer_get_time();
        res += now - startC;
        duration += now - start;
        readt += startC - start;
        if (++count >= 500) { 
          double current, voltage, power;
          current = adcGetCurrent();
          voltage = adcGetVoltage();
          power = current * voltage;
          DEBUG_I(
              "#Vals %d, Times: Read %d us, Calc %d us. Sum %d Res: Voltage: "
              "%.2f Current: %.3f, apower %.3f rpower %.3f\r\n",
              i2s_read_buff->count, readt / count, res / count,
              duration / count, voltage, current, power, adcGetPower());

          duration = count = res = readt = 0;
        }
      }
#endif
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

bool calculateCalibrationValues(bufStruct *values, int32_t &minV, int32_t &maxV,
                                int32_t &average, int32_t &offset) {
 
#define FILTERSHIFT 13  // for low pass filters to determine ADC offsets
#define FILTERROUNDING (1 << 12)

  unsigned long fOffset = (unsigned long)offset << FILTERSHIFT;

  for (size_t i = 0; i < values->count; ++i) {
    uint16_t value = values->buffer[i];

    int32_t val = esp_adc_cal_raw_to_voltage(value, adc_chars);

    maxV = max(maxV, val);
    minV = min(minV, val);
    average += val;
    val = val - offset;
    fOffset += val;
    offset = (uint32_t)((fOffset + FILTERROUNDING) >> FILTERSHIFT);
  }
  return true;
}

void calibration(size_t iterations, bufStruct *i2s_read_buff,
                 bufStruct *results) {
#define SUBLOOP 10
  double avgMax[2] = {0, 0};
  double avgMin[2] = {0, 0};
  double avgAvg[2] = {0, 0};
  int32_t offsets[2] = {avgCalibration[0], avgCalibration[1]};
  int32_t counts = 0;

  int32_t min[2] = {4096, 4096}, max[2] = {0, 0}, avg[2] = {0, 0};
  for (int i = 0; i < iterations; ++i) {
    results[CURRENT].count = 0;
    results[VOLTAGE].count = 0;
    readValues(i2s_read_buff, results);
    counts += results[CURRENT].count;
    calculateCalibrationValues(&results[CURRENT], min[CURRENT], max[CURRENT],
                               avg[CURRENT], offsets[CURRENT]);
    if (i % SUBLOOP == 0) {
      avgMax[CURRENT] += max[CURRENT];
      avgMin[CURRENT] += min[CURRENT];
      avgAvg[CURRENT] += avg[CURRENT] / (double)counts;
      max[CURRENT] = 0;
      min[CURRENT] = 4096;
      avg[CURRENT] = 0;
      counts = 0;
    }
    calculateCalibrationValues(&results[VOLTAGE], min[VOLTAGE], max[VOLTAGE],
                               avg[VOLTAGE], offsets[VOLTAGE]);
    if (i % SUBLOOP == 0) {
      avgMax[VOLTAGE] += max[VOLTAGE];
      avgMin[VOLTAGE] += min[VOLTAGE];
      avgAvg[VOLTAGE] += avg[VOLTAGE] / (double)counts;
      max[VOLTAGE] = 0;
      min[VOLTAGE] = 4096;
      avg[VOLTAGE] = 0;
      counts = 0;
    }
  }

  for (int i = 0; i < 2; ++i) {
    avgMax[i] /= iterations / SUBLOOP;
    avgMin[i] /= iterations / SUBLOOP;
    avgAvg[i] /= iterations / SUBLOOP;

    DBG_SECT(
        Debug.printf("Min: %.3f, Max: %.3f, avg: %.3f, range %.3f, median: "
                     "%.3f offset %d\n\r",
                     avgMin[i], avgMax[i], avgAvg[i], avgMax[i] - avgMin[i],
                     (avgMax[i] - avgMin[i]) / 2, offsets[i]);)
    Serial.printf(
        "Min: %.3f, Max: %.3f, avg: %.3f, range %.3f, median: %.3f offset "
        "%d\n\r",
        avgMin[i], avgMax[i], avgAvg[i], avgMax[i] - avgMin[i],
        (avgMax[i] - avgMin[i]) / 2, offsets[i]);
  }
  Serial.println("----------");
}

#ifdef PRINT_BUFFER
void printBuffer(bufStruct *i2s_read_buff) {
  printBuffer(i2s_read_buff->buffer, i2s_read_buff->count);
}

void printBuffer(uint16_t *buffer, size_t count) {
  Serial.printf("Read: %d values\n", count);
  for (int i = 0; i < count; i += 2) {
    uint16_t value = buffer[i];
    uint32_t val = esp_adc_cal_raw_to_voltage(value, adc_chars);
    // double val = value;
    Serial.printf("%d\n\r", val);
  }
}
#endif

void splitBuffer(const bufStruct *input, bufStruct outputChannels[2]) {
  // We denoise the buffer by using a low pass filter

#define handleValue(INDEX, VALUE)                                          \
  {                                                                        \
    actStruct = &outputChannels[(INDEX)];                                  \
    actStruct->buffer[actStruct->count] = filters[(INDEX)]->update(VALUE); \
    ++actStruct->count;                                                    \
  }

  int lastIndex = (~(input->buffer[0] >> 14)) & 0x1;
  size_t cnt = input->count / NUM_AVG_SAMPLES;
  for (size_t i = 0; i < cnt; ++i) {
    uint16_t value = input->buffer[i];
    int index = (value >> 14) & 0x1;  // 12 bytes are the value, then shift 2 more to see if its 5 or 0
    int otherIndex;
    bufStruct *actStruct;
    switch (lastIndex ^ index) {  // fast version of if...
      case 0:  // The two indices are identical, therefore, generate a dummy
               // value for the missing index

        otherIndex = (~index) & 0x1;
        actStruct = &outputChannels[otherIndex];  
        handleValue(otherIndex, actStruct->buffer[actStruct->count - 1]);

        // Fall through
      case 1:
        handleValue(index, value & 0x0FFF);
        break;
    }
    lastIndex = index;
  }
}

static const double CStarSquares[2] = {
    maxVals[CURRENT] * maxVals[CURRENT] / maxCalibration[CURRENT] /
        maxCalibration[CURRENT],
    maxVals[VOLTAGE] * maxVals[VOLTAGE] / maxCalibration[VOLTAGE] /
        maxCalibration[VOLTAGE]};
static const int32_t C1Squares[2] = {
    avgCalibration[CURRENT] * avgCalibration[CURRENT],
    avgCalibration[VOLTAGE] * avgCalibration[VOLTAGE]};
static const int32_t zeroValues2[2] = {2 * avgCalibration[CURRENT],
                                       2 * avgCalibration[VOLTAGE]};

double calculateSquareMean(bufStruct *aBuf, ValueType calibrationValueIndex) {
  int32_t sum = 0;
  uint16_t *buffer = aBuf->buffer;
  const int32_t zeroValue2 = zeroValues2[calibrationValueIndex];
  double res;

  int32_t upper = calibrationValueIndex == VOLTAGE
                      ? aBuf->count - VOLTAGE_OFFSET
                      : aBuf->count;
  for (size_t i = 0; i < upper; ++i, ++buffer) {
    
    int32_t val = esp_adc_cal_raw_to_voltage(*buffer, adc_chars);    
    sum += (val * val) - (val * zeroValue2);
  }
  if (upper) {
    res = ((CStarSquares[calibrationValueIndex] * sum) / upper +
           CStarSquares[calibrationValueIndex] *
               C1Squares[calibrationValueIndex]) /
          ADC_AVERAGE_COUNT;
  } else {
    res = 0;
  }

  avgValueSquareSum[calibrationValueIndex] =
      avgValueSquareSum[calibrationValueIndex] -
      oldVals[oldValsPos[calibrationValueIndex]][calibrationValueIndex] + res;
  oldVals[oldValsPos[calibrationValueIndex]][calibrationValueIndex] = res;
  oldValsPos[calibrationValueIndex] =
      (oldValsPos[calibrationValueIndex] + 1) % ADC_AVERAGE_COUNT;

  return avgValueSquareSum[calibrationValueIndex];
}

void calculateCurrent(bufStruct *currentBuf) {
  calculateSquareMean(currentBuf, CURRENT);
}

void calculateVoltage(bufStruct *voltageBuf) {
  calculateSquareMean(voltageBuf, VOLTAGE);
}

// We are calculating the constant portion of the sum we want to calculate
static const double CStar = maxVals[CURRENT] * maxVals[VOLTAGE] /
                            (maxCalibration[CURRENT] * maxCalibration[VOLTAGE]);
static const double CStarDash =
    maxVals[CURRENT] * maxVals[VOLTAGE] /
    (maxCalibration[CURRENT] * maxCalibration[VOLTAGE]) *
    avgCalibration[CURRENT] * avgCalibration[VOLTAGE];

void calculatePower(const uint16_t *currentBuf, size_t currentBufSize,
                    const uint16_t *voltageBuf, size_t voltageBufSize) {
  size_t count = min(currentBufSize, voltageBufSize);
  int32_t sumPower = 0;

  for (size_t i = 0; i < count; ++i) {
    // Get voltage
    int32_t valCurrent = esp_adc_cal_raw_to_voltage(currentBuf[i], adc_chars);
    int32_t valVoltage = esp_adc_cal_raw_to_voltage(voltageBuf[i], adc_chars);

    sumPower += valCurrent * valVoltage - valCurrent * avgCalibration[VOLTAGE] -
                valVoltage * avgCalibration[CURRENT];
  }

  if (count) {
    sumPower = ((CStar * sumPower / count + CStarDash) / ADC_AVERAGE_COUNT);
    avgValueSquareSum[POWER] =
        avgValueSquareSum[POWER] - oldVals[oldValsPos[POWER]][POWER] + sumPower;

    oldVals[oldValsPos[POWER]][POWER] = sumPower;
    oldValsPos[POWER] = (oldValsPos[POWER] + 1) % ADC_AVERAGE_COUNT;
  }
}

void calculate(bufStruct *currentBuf, bufStruct *voltageBuf) {
  
  calculateVoltage(voltageBuf);
  calculateCurrent(currentBuf);
  calculatePower(currentBuf->buffer, currentBuf->count, voltageBuf->buffer,
                 voltageBuf->count);
}

double adcGetVoltage() {
  double val = avgValueSquareSum[VOLTAGE];
  return roundf(sqrt(val));
}

double adcGetCurrent() {
  double val = avgValueSquareSum[CURRENT];
  return sqrt(val);
}

double adcGetPower() { return avgValueSquareSum[POWER]; }

void adcStop() {
  Serial.print(" Stopping adc task ");
  stopAdcUsage = true;
  do {
    vTaskDelay(50 / portTICK_PERIOD_MS);
  } while (readTask);

  Serial.println(" ADC task gone");
}

void adcStart() { adcInit(); }
