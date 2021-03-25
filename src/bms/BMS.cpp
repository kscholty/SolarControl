/*
 * \brief Source file for BMS class
 * \author rahmaevao
 * \version 0.1
 * \date November 2018
 */

#include "BMS.hpp"

uint8_t bmsInterruptedCountRxDMA = 0;
uint8_t bmsInterruptedCountTim6 = 0;
/**
 * \brief Constructor
 */
BMS::BMS() {
	m_bmsData.current = 0;
	m_bmsData.chargePercentage = 0;
	m_bmsData.protectionState = 0;
	m_bmsData.halStatus = HAL_OK;
	m_thisBms = jbdChina;
	m_typeRequstTiny = GLOBAL_MESSAGE;

	HAL_UART_Receive_DMA(&huartBMS, dmaBuffer, lenDmaBuffer);
}

/**
 * \brief Read logs from BMS. Cleaning logs in BMS.
 *
* This method also is checking the type of BMS
*/
BmsLogs BMS::readBmsLogs(void) {
	HAL_StatusTypeDef halStatus = HAL_ERROR;
	uint8_t jbdSearchCounter = 0;
	while ((jbdSearchCounter < 3) && (halStatus != HAL_OK))
	{
		jbdSearchCounter++;
		requstLogs();
		halStatus = parseDmaBuffer(LOGS_MESSAGE);
	}


	if (halStatus != HAL_OK) // Hypothesis: connected TinyBMS
	{
		m_thisBms = tinyBms;
		// Change global variables
		lenRequestMessage = 8;
		uint8_t requestMessageForTinyBms[] = { 0x01, 0x03, 0x00, 0x26, 0x00, 0x0A, 0x24, 0x06};
		memcpy(requestMessage, requestMessageForTinyBms, lenRequestMessage);

		// Change baudrate
		HAL_UART_DeInit(&huartBMS);
		huartBMS.Init.BaudRate = 115200;
		HAL_UART_Init(&huartBMS);

		lenDmaBuffer = 25;
		HAL_UART_Receive_DMA(&huartBMS, dmaBuffer, lenDmaBuffer);

		uint8_t tinySearchCounter = 0;
		while ((tinySearchCounter < 3) && (halStatus != HAL_OK))
		{
			tinySearchCounter++;
			requstLogs();
			halStatus = parseDmaBuffer(LOGS_MESSAGE);
		}
	}
	m_bmsLogs.halStatus = halStatus;
	eraseBmsLogs();
	return getBmsLogs();
}

/**
 * \brief Run autosender mode
 *
 * Autosender mode it is mode when STM send the request message if dma buffer
 * if filled.
 */
void BMS::autoSenderModeEnable(void) {

	// Reset transfer counter DMA1 Channel6
	DMA1_Channel6->CCR &= ~DMA_CCR_EN;
	DMA1_Channel6->CNDTR = lenDmaBuffer;
	DMA1_Channel6->CCR |= DMA_CCR_EN;

	HAL_TIM_Base_Start(&htim6);
	HAL_TIM_Base_Start_IT(&htim6);

	// Clear flag global interrupt
	DMA1->IFCR |= DMA_IFCR_CGIF6;


	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	vTaskDelay(200);
}

/**
 * \brief Read data from BMS: current, cupacity, protection state, hal status
 */
BmsData BMS::readBmsData(void) {
	if(bmsInterruptedCountRxDMA > 0)
	{
		HAL_UART_Transmit(&huartBMS, requestMessage, lenRequestMessage, 100);
		bmsInterruptedCountRxDMA--;
	}
	if(bmsInterruptedCountTim6 > 0)
	{
		HAL_UART_Transmit(&huartBMS, requestMessage, lenRequestMessage, 100);
		HAL_UART_Receive_DMA(&huartBMS, dmaBuffer, lenDmaBuffer);
		bmsInterruptedCountTim6--;
	}

	if (bufferHalStatus == HAL_TIMEOUT) {
		m_bmsData.halStatus = HAL_TIMEOUT;
	} else {
		m_bmsData.halStatus = parseDmaBuffer(GLOBAL_MESSAGE);
	}
	return getBmsData();
}

/**
 * \brief Finding in buffer message with specific commandCode and allocation
 * data in class fields.
 * \param[in] t_byte2 - type of message.
 * \return HAL status reading buffer.
 */
HAL_StatusTypeDef BMS::parseDmaBuffer(TypeMessageBms t_typeMessage) {
	HAL_StatusTypeDef halStatus = HAL_ERROR;
	uint8_t * message = new uint8_t[lenDmaBuffer];
	uint8_t entryCounter = 0;
	while ((halStatus != HAL_OK) && (entryCounter<=10))
	{
		halStatus = readDmaBuffer(message, t_typeMessage);
		entryCounter++;
		if (halStatus != HAL_OK)
		{
			vTaskDelay(30);
		}
	}
	parseTheMessage(message, t_typeMessage);
	delete[] message;

	return halStatus;
}

/**
 * \brief Finding in buffer message with specific commandCode.
 * \param[out] t_message found message starting with START_BYTE_JBD.
 * \param[in] t_byte2 tipe of message for finding.
 * \return HAL status reading buffer.
 */
HAL_StatusTypeDef BMS::readDmaBuffer(uint8_t * t_message, TypeMessageBms t_typeMessage) {
	uint16_t begineIndex = 0;
	uint8_t startByte;
	uint8_t secondByte;

	switch(m_thisBms)
	{
		case jbdChina:
		{
			startByte = START_BYTE_JBD;
			if (t_typeMessage == LOGS_MESSAGE) {
				secondByte = REQ_LOGS_JBD;
			}else{
				secondByte = GLOBAL;
			}
			break;
		}
		case tinyBms:
		{
			startByte = START_BYTE_TINY;
			secondByte = GLOBAL;
			break;
		}
	}


	for (uint16_t i = 0; i < lenDmaBuffer; i++) {
		if (dmaBuffer[i] != startByte)
			continue;

		if (i == lenDmaBuffer - 1)
				{
			if (dmaBuffer[0] != secondByte)
				continue;
		} else {
			if ((dmaBuffer[i + 1] != secondByte))
				continue;
		}

		begineIndex = i;

		// Copy first half message
		for (int j = 0; j < lenDmaBuffer - begineIndex; j++) {
			t_message[j] = dmaBuffer[begineIndex + j];
		}

		// Copy second half message
		for (int j = 0; j < begineIndex; j++) {
			t_message[j + lenDmaBuffer - begineIndex] = dmaBuffer[j];
		}

		if (checkCheckSumRecieve(t_message) == true) {
			return HAL_OK;
		} else {

			return HAL_ERROR;
		}
	}
	return HAL_BUSY;
}

/**
 * \brief Parse the message and  allocation data in class fields. Autodetection
 * type message.
 * \param [in] t_bmsMessage
 * \return None
 */
void BMS::parseTheMessage(uint8_t * t_message, TypeMessageBms t_typeMessageBms) {
	switch (m_thisBms) {
		case jbdChina:
		{
			switch (t_typeMessageBms) {
			case GLOBAL_MESSAGE:
				m_bmsData.current = ((t_message[6] << 8) | t_message[7]) * 10;
				m_bmsData.chargePercentage = t_message[23];
				m_bmsData.protectionState = (t_message[20] << 8) | t_message[21];
				break;
			case LOGS_MESSAGE:
				m_bmsLogs.ShortCurrentCounter = t_message[4] << 8 | t_message[5];
				m_bmsLogs.ChargeOverCurrentCounter = t_message[6] << 8 | t_message[7];
				m_bmsLogs.DischargeOverCurrentCounter = t_message[8] << 8| t_message[9];
				m_bmsLogs.CellOverVoltgaeCurrentCounter = t_message[10] << 8 | t_message[11];
				m_bmsLogs.CellUnderVoltageCounter = t_message[12] << 8 | t_message[13];
				m_bmsLogs.ChargeOverTemperatureCounter = t_message[14] << 8 | t_message[15];
				m_bmsLogs.ChargeUnderTemperatureCounter = t_message[16] << 8 | t_message[17];
				m_bmsLogs.DischargeOverTemperatureCounter = t_message[18] << 8 | t_message[19];
				m_bmsLogs.DischargeUnderTemperatureCounter = t_message[20] << 8 | t_message[21];
				m_bmsLogs.PackOverVoltageCounter = t_message[22] << 8 | t_message[23];
				m_bmsLogs.PackUnderVoltageCounter = t_message[24] << 8 | t_message[25];
				break;
			}
			break;
		}
		case tinyBms:
		{
			switch (t_typeMessageBms) {
			case GLOBAL_MESSAGE:
				m_bmsData.current = (int16_t) (converUint32ToFloat((t_message[5] << 24) | (t_message[6]<<16) | (t_message[3]<<8) | t_message[4])*1000);
				m_bmsData.chargePercentage = (uint8_t)(((t_message[21] << 24) | (t_message[22]<<16) | (t_message[19]<<8) | t_message[20])/1000000);
				break;
			case LOGS_MESSAGE:
				m_bmsLogs.PackUnderVoltageCounter = (t_message[3]<<8) | t_message[4];
				m_bmsLogs.PackOverVoltageCounter = (t_message[5]<<8) | t_message[6];
				m_bmsLogs.DischargeOverCurrentCounter = (t_message[7]<<8) | t_message[8];
				m_bmsLogs.ChargeOverCurrentCounter = (t_message[9]<<8) | t_message[10];
				m_bmsLogs.ChargeOverTemperatureCounter = (t_message[9]<<11) | t_message[12];
				m_bmsLogs.DischargeOverTemperatureCounter = m_bmsLogs.ChargeOverTemperatureCounter;
				break;
			}
			break;
		}
	}
}

/**
 * \brief Compute and cheking check sum in message
 * \param [in] t_message
 * \return Fact of passing the test:
 *  true - check passed;
 *  false - check not passed
 */
bool BMS::checkCheckSumRecieve(uint8_t * t_message) {

	uint16_t checkSumcompute;
	uint16_t checkSumInRecieveMessage;
	uint8_t lengthData;
	uint8_t startIndexCS;
	switch (m_thisBms) {
	case jbdChina:
		lengthData = t_message[3];
		checkSumcompute = computeCrc16JbdChina(t_message, lenDmaBuffer);
		startIndexCS = lengthData + 4;
		break;
	case tinyBms:
		lengthData = t_message[2];
		checkSumcompute = computeCrc16ModBus(t_message, lengthData+3);
		startIndexCS = lengthData + 3;
		break;
	}

	checkSumInRecieveMessage = (t_message[startIndexCS]<<8) | t_message[startIndexCS+1];

	if (checkSumcompute != checkSumInRecieveMessage)
		return false;
	return true;
}

/**
 * \brief Compute check sum in message for ModBUS protocol
 * \param[in] puchMsg	Message buffer containing binary data to be used for
 * 						generating the CRC.
 * \param[in] usDataLen	The quantity of bytes in the message buffer.
 * \return The function returns the CRC.
 * \sa http://www.modbus.org/docs/PI_MBUS_300.pdf
 */
uint16_t BMS::computeCrc16ModBus(uint8_t * puchMsg, uint8_t usDataLen)
{
	uint8_t uchCRCHi = 0xFF ; /* high byte of CRC initialized */
	uint8_t uchCRCLo = 0xFF ; /* low byte of CRC initialized */
	uint8_t uIndex ; /* will index into CRC lookup table */
	while (usDataLen--) /* pass through message buffer */
	{
		uIndex = uchCRCHi ^ *puchMsg++ ; /* calculate the CRC */
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
		uchCRCLo = auchCRCLo[uIndex] ;
	}
	return (uchCRCHi << 8 | uchCRCLo);
}

/**
* \brief Compute check sum in message for JBD protocol semiModbus :)
 * \param[in] puchMsg	Message buffer containing binary data to be used for
 * 						generating the CRC.
 * \param[in] usDataLen	The quantity of bytes in the message buffer.
 * \return The function returns the CRC.
 */
uint16_t BMS::computeCrc16JbdChina(uint8_t * puchMsg, uint8_t usDataLen)
{
	uint8_t lengthData = puchMsg[3];

	uint16_t summa = 0;

	for (int i = 4; i < lengthData + 4; i++)
		summa = summa + puchMsg[i];

	uint16_t checkSum = (summa + lengthData - 1) ^ 0xFFFF;
	return checkSum;

// 	Block by highlighting CRC from message
//	uint8_t beginIndexCS = lengthData + 4;
//
//	if ((((checkSum >> 8) & 0xFF) == t_message[beginIndexCS])
//			&& ((checkSum & 0xFF) == t_message[beginIndexCS + 1])) {
//		return true;
//	}
//	return false;
}

void BMS::goToConfig(void) {
	uint8_t requestMessage[] = { START_BYTE_JBD, WRITE, 0x00, 0x02, 0x56, 0x78,
			0xFF, 0x30, ENDLER };
	HAL_UART_Transmit(&huartBMS, requestMessage, 9, 100);
	vTaskDelay(70); // Меньше нельзя!
}

/**
 * \brief Clear logs in BMS
 */
void BMS::eraseBmsLogs(void) {
	switch(m_thisBms){
		case jbdChina:
		{
			uint8_t eraseBmsLogsMessage[] = { START_BYTE_JBD, WRITE, 0x01, 0x02, 0x28, 0x28, 0xFF, 0xAD, ENDLER };
			HAL_UART_Transmit(&huartBMS, eraseBmsLogsMessage, 9, 100);
			break;
		}
		case tinyBms:
		{
			uint8_t eraseBmsLogsMessage[] = { START_BYTE_TINY, 0x02, 0x02, 0xD1, 0x41};
			HAL_UART_Transmit(&huartBMS, eraseBmsLogsMessage, 5, 100);
			break;
		}
	}
	vTaskDelay(900); // Do not touch
}

void BMS::sendRequestMessage(void)
{
	HAL_UART_Transmit(&huartBMS, requestMessage, lenRequestMessage, 100);
}

void BMS::requstLogs(void)
{
	switch(m_thisBms){
		case jbdChina:
		{
			goToConfig();
			uint8_t requestMessage[] = { START_BYTE_JBD, READ, 0xAA, 0x00, 0xFF, 0x56, ENDLER };
			HAL_UART_Transmit(&huartBMS, requestMessage, 7, 100);
			break;
		}
		case tinyBms:
		{
			uint8_t requesRecords[] = { START_BYTE_TINY, 0x03, 0x00, 0x69, 0x00, 0x05, 0x4C, 0x0E};
			HAL_UART_Transmit(&huartBMS, requesRecords, 8, 100);
			break;
		}
	}
	vTaskDelay(80); // Do not touch
}

//GET functions
BmsData BMS::getBmsData(void) {
	return m_bmsData;
}
BmsLogs BMS::getBmsLogs(void) {
	return m_bmsLogs;
}

/**
 * \brief 	Сборка float из uint32_t
*/
float BMS::converUint32ToFloat(uint32_t number)
{
	union DataType
	{
		float f;
		uint32_t uint32t;
	};

	union DataType sample;
	sample.uint32t = number;
	return sample.f;
}
