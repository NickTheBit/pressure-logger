/**********************************************
 * @brief  Bare-bone driver for the BME280_DEFS sensor.
 * @author Nikolaos Xenofon Gkloumpos
 * @date   18/09/2023
 **********************************************/

#include "BME280.h"
#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief Initializes the BME280_DEFS class with i2c style communication with
 * the chip.
 * @param i2cBus
 * @param address
 */
BME280::BME280(I2CInterface &i2cBus, uint8_t address, uint16_t pollingRate)
	: i2cBus(i2cBus), address(address), pollingRate(pollingRate) {

	// Sensor Initialization
	uint8_t resetCommand = BME280_DEFS::SOFT_RESET_COMMAND;
	this->writeRegister(BME280_DEFS::REG_RESET, &resetCommand, 1);

	if (this->pollingRate != 0) {
		xTaskCreate(BME280::pollingTask, "BME_Reader", 128, this, 1, nullptr);
	}
}

void BME280::readRegister(uint8_t registerAddress, uint8_t *returnedData,
						  uint32_t length) const {
	this->i2cBus.write(this->address, &registerAddress, 1);
	this->i2cBus.read(this->address, length, returnedData);
}

void BME280::writeRegister(uint8_t registerAddress, uint8_t *dataToWrite,
						   uint32_t length) const {
	this->i2cBus.write(registerAddress, dataToWrite, length);
}

/**
 * @brief Probes the sensor to get latest environmental data.
 */
void BME280::getSensorData() {
	uint8_t rawData[BME280_DEFS::LEN_P_T_H_DATA] = {0};
	readRegister(BME280_DEFS::REG_DATA, rawData, BME280_DEFS::LEN_P_T_H_DATA);

	/* Variables to store the raw sensor data */
	struct rawMeasurements uncomp_data {};
	uint32_t data_xlsb;
	uint32_t data_lsb;
	uint32_t data_msb;

	/* Store the parsed register values for pressure data */
	data_msb = (uint32_t)rawData[0] << 12;
	data_lsb = (uint32_t)rawData[1] << 4;
	data_xlsb = (uint32_t)rawData[2] >> 4;
	uncomp_data.pressure = data_msb | data_lsb | data_xlsb;

	/* Store the parsed register values for temperature data */
	data_msb = (uint32_t)rawData[3] << 12;
	data_lsb = (uint32_t)rawData[4] << 4;
	data_xlsb = (uint32_t)rawData[5] >> 4;
	uncomp_data.temperature = data_msb | data_lsb | data_xlsb;

	/* Store the parsed register values for humidity data */
	data_msb = (uint32_t)rawData[6] << 8;
	data_lsb = (uint32_t)rawData[7];
	uncomp_data.humidity = data_msb | data_lsb;

	// fixme: Temporarilly avoiding adjustments
	currentMeasurements.humidity = uncomp_data.humidity;
	currentMeasurements.temperature = uncomp_data.temperature;
	currentMeasurements.pressure = uncomp_data.pressure;
}

uint8_t BME280::getChipID() {
	uint8_t rawData;
	readRegister(BME280_DEFS::REG_CHIP_ID, &rawData, 1);
	return rawData;
}

double BME280::getTemperature() {
	return this->currentMeasurements.temperature;
}

double BME280::getHumidity() {
	return this->currentMeasurements.humidity;
}

double BME280::getPressure() {
	return this->currentMeasurements.pressure;
}

/**
 * @brief	Fetches the sensors current state
 * @return	sensorPowerMode_t
 */
BME280_DEFS::sensorPowerMode_t BME280::getSensorMode() {
	uint8_t retVal;
	readRegister(BME280_DEFS::REG_PWR_CTRL, &retVal, 1);

	switch (retVal) {
		case 0:
			return BME280_DEFS::sensorPowerMode_t::NORMAL;
		case 1:
			return BME280_DEFS::sensorPowerMode_t::ASLEEP;
		case 2:
			return BME280_DEFS::sensorPowerMode_t::FORCED;
		default:
			return BME280_DEFS::sensorPowerMode_t::ASLEEP;
	}
}

/**
 * @note Non functional for now, only serves to detect pointer misuse.
 */
BME280::~BME280() {
	// todo: Implement log message to detect object deinit.
}
