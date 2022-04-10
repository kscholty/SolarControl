
#pragma once
#define ARDUINOJSON_USE_DOUBLE 1
#include <ArduinoJson.h>

class ShellyDevice {

public:

/**
 * @brief Construct a new Shelly Device object
 * Use @c setDelay() and @c setIp() to define the parameters
 * 
 */
ShellyDevice();
/**
 * @brief Construct a new Shelly Device object
 * 
 * @param ip The IP address of the client
 * @param delay The delay between two queries. Ised in doLoop
 */
ShellyDevice(String ip, unsigned long delay);

/** 
 * @brief Returns the Type name of the device
 * @return The device ID string
 */
 String getDeviceType() { return mDeviceType;} 


/**
 * @brief Get the number of meters
 * 
 * @return Number of meters implemented in the device
 */
unsigned int getNumMeters() { return mNumMeters;}

/**
 * @brief Get the number of Emeters in the device
 * 
 * @return Number of emeters
 */
unsigned int getNumEmeters() { return mNumEmeters;}

/**
 * @brief Get the number of relays in the device
 * 
 * @return numer ov relays
 */
unsigned int getNumRelays() { return mNumRelays;}

/**
 * @brief Set the Delay 
 * 
 * @param aDelay The delay to set
 */
void setDelay(unsigned long aDelay) {mDelay=aDelay;}

/**
 * @brief Set the Ip of the client
 * 
 * Thias will be effective for the next updateNow() call.
 * 
 * @param anIp The new IP address
 */
void setIp(String anIp) {mBaseUrl = "http://" + anIp;}

/**
 * @brief Query the URL used to conbtact the client
 * 
 * 
 * @returns The query URL
 */
const String& getQueryUrl() {return mBaseUrl;}


/**
 * @brief Get the Last Error code
 * 
 * @return int tzhe last error code. 0 means No error
 */
int getLastError() { return mLastErrorCode;}

/**
 * @brief Get the Last Error String 
 * 
 * @return A string describing the last error
 */
String getLastErrorString() { return mLastErrorString;}

/**
	 * Gets the status of the device.
	 * 
	 * @param index the index of the meter to query
	 * @return the power in W of the respective meter or 0 if that does not exist
	 */
int getPower(unsigned int index);


/**
	 * Gets the status of the device.
	 * 
	 * 
	 * @param index the index of the emeter to query
	 * @return the current of the resapective emeter or 0 if that does not exist
	 */
int getCurrent(unsigned int index);

/**
	 * Gets the status of the device.
	 *
	 * 
	 * @param index the index of the emeter to query
	 * @return the voltage of the resapective emeter or 0 if that does not exist
	 */
int getVoltage(unsigned int index);

/**
	 * Gets the status of the device.
	 *
	 * 
	 * @param index the index of the emeter to query
	 * @return the power factor of the resapective emeter or 0 if that does not exist
	 */
int getPowerFactor(unsigned int index);


/**
	 * Gets the "ison" state of the relay with the given index.
	 * 
	 * @param index the index of the relay
	 * @return the boolean value
	 */
bool getRelayIson(unsigned int index) {if(index<mNumRelays) return mRelayValues[index]; return false;}

/**
 * @brief Update a relay independently from the power data
 * 
 * You have to call at least one @ref updateNow() before this can succeed
 * @param index The index of the relay
 * @return true if query was successful
 * @return false On failure
 */
bool updateRelay(unsigned int index) { return queryRelay(index);}


/**
 * @brief Forces an update of the device
 * 
 * @return true If update was succeessful
 * @return false If update failed
 */
bool updateNow();

/**
 * @brief Updates the device if the set delay has been passed
 * 
 * @return true If update took place
 * @return false Otherwise
 */
bool doLoop();


private:
	static const uint16_t numElements = 3;
	bool sendGetRequest(JsonDocument &doc, String endpoint);
	bool queryShelly();
	bool queryStatus();
	bool queryRelay(int index);

	bool mInit;
	size_t mLastQuery;
	size_t mDelay;
	String mBaseUrl;
	uint16_t mNumRelays;
	uint16_t mNumMeters;
	uint16_t mNumEmeters;
	String mDeviceType;
	String mLastErrorString;
	int mLastErrorCode;
	int mPowerValues[numElements];
	int mCurrentValues[numElements];
	int mVoltageValues[numElements];
	int mPowerFactorValues[numElements];
	bool mRelayValues[numElements];
	

};


