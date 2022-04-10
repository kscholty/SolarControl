
#include <arduino.h>
#include <Wifi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include "ShellyDevice.h"


ShellyDevice::ShellyDevice() {
    setDelay(1000);
		mNumMeters = 0;
		mNumEmeters=0;
		mNumRelays = 0;
		mInit = false;
		mLastQuery = 0;
    mLastErrorString="Not initialized";
    mLastErrorCode = 0;
}


ShellyDevice::ShellyDevice(String ip, unsigned long delay) {
		setIp(ip);
    setDelay(delay);
		mNumMeters = 0;
		mNumEmeters=0;
		mNumRelays = 0;
		mInit = false;
		mLastQuery = 0;
    mLastErrorString="No Error";
    mLastErrorCode = 0;
}

bool ShellyDevice::updateNow() {
  bool result = true;
  if (mBaseUrl.isEmpty()) {
    return false;
  }
  if (!mInit) {
    result = queryShelly();
  }

  if (result) {
    result = queryStatus();    
  }
  return result;
}

bool ShellyDevice::doLoop() {
	bool result = false;
	unsigned long now = millis();

	if(now-mLastQuery>=mDelay && !mBaseUrl.isEmpty()) {
		updateNow();
		mLastQuery = now;    
    result = true;
	}	

	return result;
}


int ShellyDevice::getPower(unsigned int index) {
  if(index < max(mNumMeters,mNumEmeters)) {
    return mPowerValues[index];
  }
  return 0;
}

int ShellyDevice::getCurrent(unsigned int index) {
  if (index < mNumEmeters) {
    return mCurrentValues[index];
  }
  return 0;
}

int ShellyDevice::getVoltage(unsigned int index) {
  if (index < mNumEmeters) {
    return mVoltageValues[index];
  }
  return 0;
}

int ShellyDevice::getPowerFactor(unsigned int index) {
  if (index < mNumEmeters) {
    return mPowerFactorValues[index];
  }
  return 0;
}

bool ShellyDevice::queryStatus() {
  DynamicJsonDocument doc(2048);

  if (sendGetRequest(doc, "/status")) {
    for (int i = 0; i < mNumMeters; ++i) {
      JsonObject meter = doc["meters"][i];
      mPowerValues[i] = (float)meter["power"]*1000;
    }

    for (int i = 0; i < mNumEmeters; ++i) {
      JsonObject meter = doc["emeters"][i];
      mPowerValues[i] = (float)meter["power"]*1000;
      mCurrentValues[i] = (float)meter["current"]*1000;
      mVoltageValues[i] = (float)meter["voltage"]*1000;
      mPowerFactorValues[i] = (float)meter["pf"]*1000;
    }

    for (int i = 0; i < mNumRelays; ++i) {
      JsonObject relay = doc["relays"][i];
      mRelayValues[i] = relay["ison"];
    }
    return true;
  }
  return false;
}

bool ShellyDevice::queryShelly() {
  StaticJsonDocument<384> doc;

  if (sendGetRequest(doc, "/shelly")) {
    mDeviceType = (const char*)doc["type"];
    mNumMeters = doc["num_meters"];
    mNumEmeters = doc["num_emeters"];
    mNumRelays = doc["num_outputs"];
    mNumMeters = min(mNumMeters,numElements); 
    mNumEmeters = min(mNumEmeters,numElements); 
    mNumRelays = min(mNumRelays,numElements); 
	  mInit = true;
    return true;
  }
  return false;
}


bool ShellyDevice::queryRelay(int index) {
  bool result = false;
	StaticJsonDocument<256> doc;
	String endpoint = "/relay/";
	endpoint += String(index);
	if(index < mNumRelays) {
		if (sendGetRequest(doc, endpoint)) {
			mRelayValues[index] = doc["ison"];
      result = true;
		}
	}

	return result;
}

bool ShellyDevice::sendGetRequest(JsonDocument& doc, String endpoint) {
  WiFiClient client;
  HTTPClient http;
  bool result = true;

  String request = mBaseUrl + endpoint;
  
  http.begin(client, request);
  int httpResponseCode = http.GET();

  if (httpResponseCode == HTTP_CODE_OK ) {    
    DeserializationError error = deserializeJson(doc, http.getString());
    mLastErrorCode = error.code();
    mLastErrorString = error.f_str();
    result = error == DeserializationError::Ok;    
  } else {
    mLastErrorCode = httpResponseCode;
    if(mLastErrorCode < 0) {
      mLastErrorString = http.errorToString(httpResponseCode);
    } else {
      mLastErrorString =String("Http: ")+String(mLastErrorCode);
    }
	  result = false;
  }
  // Free resources
  http.end();

  return result;
}


