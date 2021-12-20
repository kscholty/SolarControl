
#pragma once

#include <HardwareSerial.h>
#include "common.h"
#include "bmstypes.h"

#if JKBMS
#include "jkbms_impl.h"
#elif QUCCBMS
#include "quccbms_impl.h"
#endif

namespace BMS {

template <class BMS_IMPL>
class CBms {
 public:
  CBms(HardwareSerial &aStream, SemaphoreHandle_t aMutex)
      : mCommStream(aStream),
        mStreamMutex(aMutex),
        mLowDelayMs(300000),
        mHighDelayMs(1000),
        mLastLowTrigger(0),
        mLastHighTrigger(0),
        mBasicInfo(0),
        mCellInfo(0) {}
  ~CBms(){};

  bool setup(){return mBmsImpl.setup();}
  bool doLoop();
  void setLowFrequencyDelay(unsigned int aDelay) { mLowDelayMs = aDelay; }
  void setHighFrequencyDelay(unsigned int aDelay) { mHighDelayMs = aDelay; }
  const BMS::BmsBasicInfo_t &basicInfo() const { return mBasicInfo; }
  const BMS::BmsCellInfo_t &cellInfo() const { return mCellInfo; }

 protected:
  bool readHighFrequentData();
  bool readLowFrequentData();

 private:
  bool handleRequest(const uint8_t *request, size_t length);
  size_t readAnswerMessage(uint8_t **answer);

 protected:
  HardwareSerial &mCommStream;
  SemaphoreHandle_t mStreamMutex;
  unsigned long mLowDelayMs;
  unsigned long mHighDelayMs;
  unsigned long mLastLowTrigger;
  unsigned long mLastHighTrigger;
  BmsBasicInfo_t mBasicInfo;
  BmsCellInfo_t mCellInfo;
  BMS_IMPL mBmsImpl;
  
};

template <class BMS_IMPL>
bool CBms<BMS_IMPL>::readHighFrequentData() {
  const uint8_t *request = 0;
  size_t length = 0;
  bool res = mBmsImpl.getHighFrequRequest(&request, &length);
  if (res && length && request) {
    return handleRequest(request, length);
  }

  return false;
}

template <class BMS_IMPL>
bool CBms<BMS_IMPL>::readLowFrequentData() {
  const uint8_t  *request = 0;
  size_t length = 0;
  bool res = mBmsImpl.getLowFrequRequest(&request, &length);
  if (res && length && request) {
    return handleRequest(request, length);
  }

  return false;
}

template <class BMS_IMPL>
bool CBms<BMS_IMPL>::handleRequest(const uint8_t *request, size_t length) {
  bool result = false;
  size_t messageSize = 0;
  uint8_t *answer = 0;
  if (!mStreamMutex ||
      xSemaphoreTake(mStreamMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    while (mCommStream.read() != -1)
      ;
    if (mCommStream.baudRate() != mBmsImpl.BAUDRATE) {
      mCommStream.updateBaudRate(mBmsImpl.BAUDRATE);
    }

    mCommStream.write(request, length);
    mCommStream.flush();
    vTaskDelay(300);
    messageSize = readAnswerMessage(&answer);
    if (mStreamMutex) xSemaphoreGive(mStreamMutex);
    if (messageSize) {
      // We got an answer
      result =
          mBmsImpl.processMessage(answer, messageSize, &mBasicInfo, &mCellInfo);
    } else {
      debugW("BMS: Could not get result message\n");
    }
  }
  return result;
}

template <class BMS_IMPL>
size_t CBms<BMS_IMPL>::readAnswerMessage(uint8_t **answer) {
#define TIMEOUT 1000
  static uint8_t messagebuffer[BMS_IMPL::MESSAGESIZE];
  size_t toRead, read;
  unsigned long starttime = millis();

  *answer = messagebuffer;
  size_t messagePointer = 0;
  toRead = mBmsImpl.messageHeaderSize();

  do {
    read = mCommStream.readBytes(messagebuffer + messagePointer, toRead);
    toRead -= read;
    messagePointer += read;

  } while (toRead && (millis() - starttime) < TIMEOUT);

  if (toRead != 0) {
    // We didn't read anything or the answer has the wrong format
    // That is assumed to be an error
    debugE("BMS: Still to read %d bytes.\n", toRead);
    return 0;
  }

  toRead = mBmsImpl.getBodySize(messagebuffer, messagePointer);

  if (toRead + messagePointer > sizeof(messagebuffer)) {
    debugW("BMS: Message too large for buffer %d\n", toRead);
    return 0;
  } else {
    debugI("Body size to read:%d \n", toRead);
  }

  do {
    read = mCommStream.readBytes(messagebuffer + messagePointer, toRead);
    toRead -= read;
    messagePointer += read;
  } while (toRead && read);

  if (toRead) {
    debugW("BMS: Still to read %d bytes in body.\n", toRead);
  }

  if (Debug.isActive(Debug.INFO)) {
    debugI("BMS: Message size is %d\n", messagePointer);
    for (int i = 0; i < messagePointer; ++i) {
      Debug.printf("0x%02x ", messagebuffer[i]);
    }
    Debug.printf("\n");
  }
  return (messagePointer);
}

template <class BMS_IMPL>
bool CBms<BMS_IMPL>::doLoop() {
  size_t now = millis();
  bool res = false;
  if (now - mLastLowTrigger > mLowDelayMs) {
    res = readLowFrequentData();
    mLastLowTrigger = now;
  } else {
    if (now - mLastHighTrigger > mHighDelayMs) {
      res |= readHighFrequentData();
      mLastHighTrigger = now;
    }
  }
  return res;
}

}  // namespace BMS

#if JKBMS
typedef BMS::CBms<BMS::JKBms> BMS_t;
#elif QUCCBMS
typedef BMS::CBms<BMS::QUCCBms> BMS_t;
#endif