/**********************************************
 * @brief  Bare-bone driver for the BME280_DEFS sensor.
 * @author Nikolaos Xenofon Gkloumpos
 * @date   18/09/2023
 **********************************************/

#pragma once

#include "BME280_bosch_driver/BME280_defs.hh"
#include "Interfaces/I2CInterface.hh"
#include <Architectures/RP2040/RP2040_I2C/RP2040_I2C.h>
#include <BME280/BME280_bosch_driver/bme280_bosch_driver.hh>
#include <cstring>

template <typename Arch> class BME280 {
  private:
	I2CInterface<Arch> &i2cBus;
	uint8_t address;
	uint16_t pollingRate;
	uint32_t measurementPeriod = 0;
	struct bme280_dev sensor;
	struct bme280_settings settings{};

	/**
	 * @brief Interface function with sensor drivers from bosch
	 * @param period_usec
	 * @param intf_ptr
	 */
	static void sensorDelay(uint32_t period_usec, void *intf_ptr) {
		auto wakeupTime = delayed_by_us(get_absolute_time(), period_usec);
		while (absolute_time_diff_us(get_absolute_time(), wakeupTime) <= 0) {
		}
	}

	/**
	 * @brief Interface function for the sensor drivers from bosch
	 * @param reg_addr
	 * @param reg_data
	 * @param length
	 * @param classPtr
	 * @return register contents.
	 */
	static int8_t writeRegister(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr) {
		const auto locThis = static_cast<BME280 *>(intf_ptr);

		/* todo: investigate probability of heapless function. */
		uint8_t totalBuffer[length + 1];
		totalBuffer[0] = reg_addr;
		memcpy(&totalBuffer[1], reg_data, length);

		locThis->i2cBus.write(locThis->address, totalBuffer, length + 1);
		return 0;
	}

	/**
	 * @brief Interface function for the sensor drivers from bosch
	 * @param reg_addr
	 * @param reg_data
	 * @param length
	 * @param intf_ptr
	 * @return Success or failure return code.
	 */
	static int8_t readRegister(uint8_t reg_addr, uint8_t *reg_data, const uint32_t length, void *intf_ptr) {
		const auto locThis = static_cast<BME280 *>(intf_ptr);
		locThis->i2cBus.write(locThis->address, &reg_addr, 1);
		locThis->i2cBus.read(locThis->address, length, reg_data);
		return 0;
	}

  public:
	/**
	 * @brief Initializes the BME280_DEFS class with i2c style communication with
	 * the chip.
	 * @param i2cBus
	 * @param address
	 */
	BME280(I2CInterface<Arch> &i2cBus, uint8_t address, uint16_t pollingRate)
		: i2cBus(i2cBus), address(address), pollingRate(pollingRate) {

		this->sensor = {.intf = BME280_I2C_INTF,
						.intf_ptr = this,
						.read = readRegister,
						.write = writeRegister,
						.delay_us = sensorDelay};

		// Sensor Initialization
		bme280_init(&this->sensor);

		// Updating sampling rate and sensor settings
		bme280_get_sensor_settings(&this->settings, &this->sensor);

		/* Configuring the over-sampling rate, filter coefficient and standby time */

		this->settings.filter = BME280_FILTER_COEFF_2;
		/* Over-sampling rate for humidity, temperature and pressure */
		this->settings.osr_h = BME280_OVERSAMPLING_1X;
		this->settings.osr_p = BME280_OVERSAMPLING_1X;
		this->settings.osr_t = BME280_OVERSAMPLING_1X;

		/* Setting the standby time */
		this->settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

		bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &this->settings, &this->sensor);

		/* switching sensor to normal */
		bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &this->sensor);

		uint8_t sensorMode = 0;
		bme280_get_sensor_mode(&sensorMode, &this->sensor);

		bme280_cal_meas_delay(&measurementPeriod, &this->settings);
	}
	~BME280() = default;

	double getTemperature() {
		bme280_data comp_data{};
		bme280_get_sensor_data(BME280_TEMP, &comp_data, &this->sensor);
		return comp_data.temperature;
	}

	double getHumidity() {
		bme280_data comp_data{};
		bme280_get_sensor_data(BME280_HUM, &comp_data, &this->sensor);
		return comp_data.humidity;
	}

	double getPressure() {
		bme280_data comp_data{};
		bme280_get_sensor_data(BME280_PRESS, &comp_data, &this->sensor);
		return comp_data.pressure;
	}
};
