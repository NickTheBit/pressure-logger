/**********************************************
 * @brief  Bare-bone driver for the BME280_DEFS sensor.
 * @author Nikolaos Xenofon Gkloumpos
 * @date   18/09/2023
 **********************************************/

#include "BME280.h"
#include "FreeRTOS.h"
#include "task.h"

#include <bits/streambuf_iterator.h>
#include <cstdio>
#include <cstring>
#include <pico/time.h>

/**
 * @brief Initializes the BME280_DEFS class with i2c style communication with
 * the chip.
 * @param i2cBus
 * @param address
 */
BME280::BME280(I2CInterface &i2cBus, uint8_t address, uint16_t pollingRate)
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

/**
 * @brief Interface function with sensor drivers from bosch
 * @param period_usec
 * @param intf_ptr
 */
void BME280::sensorDelay(uint32_t period_usec, void *intf_ptr) {
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
int8_t BME280::writeRegister(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr) {
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
int8_t BME280::readRegister(uint8_t reg_addr, uint8_t *reg_data, const uint32_t length, void *intf_ptr) {
	const auto locThis = static_cast<BME280 *>(intf_ptr);
	locThis->i2cBus.write(locThis->address, &reg_addr, 1);
	locThis->i2cBus.read(locThis->address, length, reg_data);
	return 0;
}

/**
 * @note Non functional for now, only serves to detect pointer misuse.
 */
BME280::~BME280() {
	// todo: Implement log message to detect object deinit.
}

/****************** Global Function Definitions *******************************/

/*!
 *  @brief This API is the entry point.
 *  It reads the chip-id and calibration data from the sensor.
 */
int8_t BME280::bme280_init(struct bme280_dev *dev) {
	int8_t rslt;
	uint8_t chip_id = 0;

	/* Read the chip-id of bme280 sensor */
	rslt = bme280_get_regs(BME280_REG_CHIP_ID, &chip_id, 1, dev);

	/* Check for chip id validity */
	if (rslt == BME280_OK) {
		if (chip_id == BME280_CHIP_ID) {
			dev->chip_id = chip_id;

			/* Reset the sensor */
			rslt = bme280_soft_reset(dev);

			if (rslt == BME280_OK) {
				/* Read the calibration data */
				rslt = get_calib_data(dev);
			}
		} else {
			rslt = BME280_E_DEV_NOT_FOUND;
		}
	}
	return rslt;
}

/*!
 * @brief This API reads the data from the given register address of the sensor.
 */
int8_t BME280::bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bme280_dev *dev) {
	int8_t rslt;

	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);

	if ((rslt == BME280_OK) && (reg_data != NULL)) {
		/* If interface selected is SPI */
		if (dev->intf != BME280_I2C_INTF) {
			reg_addr = reg_addr | 0x80;
		}

		/* Read the data */
		dev->intf_rslt = dev->read(reg_addr, reg_data, len, dev->intf_ptr);

		/* Check for communication error */
		if (dev->intf_rslt != BME280_INTF_RET_SUCCESS) {
			rslt = BME280_E_COMM_FAIL;
		}
	} else {
		rslt = BME280_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
int8_t BME280::bme280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint32_t len, struct bme280_dev *dev) {
	int8_t rslt;
	uint8_t temp_buff[20]; /* Typically not to write more than 10 registers */
	uint32_t temp_len;
	uint32_t reg_addr_cnt;

	if (len > BME280_MAX_LEN) {
		len = BME280_MAX_LEN;
	}

	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);

	/* Check for arguments validity */
	if ((rslt == BME280_OK) && (reg_addr != NULL) && (reg_data != NULL)) {
		if (len != 0) {
			temp_buff[0] = reg_data[0];

			/* If interface selected is SPI */
			if (dev->intf != BME280_I2C_INTF) {
				for (reg_addr_cnt = 0; reg_addr_cnt < len; reg_addr_cnt++) {
					reg_addr[reg_addr_cnt] = reg_addr[reg_addr_cnt] & 0x7F;
				}
			}

			/* Burst write mode */
			if (len > 1) {
				/* Interleave register address w.r.t data for
				 * burst write
				 */
				interleave_reg_addr(reg_addr, temp_buff, reg_data, len);
				temp_len = ((len * 2) - 1);
			} else {
				temp_len = len;
			}

			dev->intf_rslt = dev->write(reg_addr[0], temp_buff, temp_len, dev->intf_ptr);

			/* Check for communication error */
			if (dev->intf_rslt != BME280_INTF_RET_SUCCESS) {
				rslt = BME280_E_COMM_FAIL;
			}
		} else {
			rslt = BME280_E_INVALID_LEN;
		}
	} else {
		rslt = BME280_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API sets the oversampling, filter and standby duration
 * (normal mode) settings in the sensor.
 */
int8_t BME280::bme280_set_sensor_settings(uint8_t desired_settings, const struct bme280_settings *settings,
										  struct bme280_dev *dev) {
	int8_t rslt;
	uint8_t sensor_mode;

	if (settings != NULL) {
		rslt = bme280_get_sensor_mode(&sensor_mode, dev);

		if ((rslt == BME280_OK) && (sensor_mode != BME280_POWERMODE_SLEEP)) {
			rslt = put_device_to_sleep(dev);
		}

		if (rslt == BME280_OK) {
			/* Check if user wants to change oversampling
			 * settings
			 */
			if (are_settings_changed(OVERSAMPLING_SETTINGS, desired_settings)) {
				rslt = set_osr_settings(desired_settings, settings, dev);
			}

			/* Check if user wants to change filter and/or
			 * standby settings
			 */
			if ((rslt == BME280_OK) && are_settings_changed(FILTER_STANDBY_SETTINGS, desired_settings)) {
				rslt = set_filter_standby_settings(desired_settings, settings, dev);
			}
		}
	} else {
		rslt = BME280_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API gets the oversampling, filter and standby duration
 * (normal mode) settings from the sensor.
 */
int8_t BME280::bme280_get_sensor_settings(struct bme280_settings *settings, struct bme280_dev *dev) {
	int8_t rslt;
	uint8_t reg_data[4];

	if (settings != NULL) {
		rslt = bme280_get_regs(BME280_REG_CTRL_HUM, reg_data, 4, dev);

		if (rslt == BME280_OK) {
			parse_device_settings(reg_data, settings);
		}
	} else {
		rslt = BME280_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API sets the power mode of the sensor.
 */
int8_t BME280::bme280_set_sensor_mode(uint8_t sensor_mode, struct bme280_dev *dev) {
	int8_t rslt;
	uint8_t last_set_mode;

	rslt = bme280_get_sensor_mode(&last_set_mode, dev);

	/* If the sensor is not in sleep mode put the device to sleep
	 * mode
	 */
	if ((rslt == BME280_OK) && (last_set_mode != BME280_POWERMODE_SLEEP)) {
		rslt = put_device_to_sleep(dev);
	}

	/* Set the power mode */
	if (rslt == BME280_OK) {
		rslt = write_power_mode(sensor_mode, dev);
	}

	return rslt;
}

/*!
 * @brief This API gets the power mode of the sensor.
 */
int8_t BME280::bme280_get_sensor_mode(uint8_t *sensor_mode, struct bme280_dev *dev) {
	int8_t rslt;

	if (sensor_mode != NULL) {
		/* Read the power mode register */
		rslt = bme280_get_regs(BME280_REG_PWR_CTRL, sensor_mode, 1, dev);

		/* Assign the power mode to variable */
		*sensor_mode = BME280_GET_BITS_POS_0(*sensor_mode, BME280_SENSOR_MODE);
	} else {
		rslt = BME280_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API performs the soft reset of the sensor.
 */
int8_t BME280::bme280_soft_reset(struct bme280_dev *dev) {
	int8_t rslt;
	uint8_t reg_addr = BME280_REG_RESET;
	uint8_t status_reg = 0;
	uint8_t try_run = 5;

	/* 0xB6 is the soft reset command */
	uint8_t soft_rst_cmd = BME280_SOFT_RESET_COMMAND;

	/* Write the soft reset command in the sensor */
	rslt = bme280_set_regs(&reg_addr, &soft_rst_cmd, 1, dev);

	if (rslt == BME280_OK) {
		/* If NVM not copied yet, Wait for NVM to copy */
		do {
			/* As per data sheet - Table 1, startup time is 2 ms. */
			dev->delay_us(BME280_STARTUP_DELAY, dev->intf_ptr);
			rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);

		} while ((rslt == BME280_OK) && (try_run--) && (status_reg & BME280_STATUS_IM_UPDATE));

		if (status_reg & BME280_STATUS_IM_UPDATE) {
			rslt = BME280_E_NVM_COPY_FAILED;
		}
	}

	return rslt;
}

/*!
 * @brief This API reads the pressure, temperature and humidity data from the
 * sensor, compensates the data and store it in the bme280_data structure
 * instance passed by the user.
 */
int8_t BME280::bme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data *comp_data,
									  struct bme280_dev *dev) const {
	int8_t rslt;

	/* Array to store the pressure, temperature and humidity data read from
	 * the sensor
	 */
	uint8_t reg_data[BME280_LEN_P_T_H_DATA] = {0};
	struct bme280_uncomp_data uncomp_data = {0};

	if (comp_data != NULL) {
		/* Read the pressure and temperature data from the sensor */
		rslt = bme280_get_regs(BME280_REG_DATA, reg_data, BME280_LEN_P_T_H_DATA, dev);

		if (rslt == BME280_OK) {
			/* Parse the read data from the sensor */
			parse_sensor_data(reg_data, &uncomp_data);

			/* Compensate the pressure and/or temperature and/or
			 * humidity data from the sensor
			 */
			rslt = bme280_compensate_data(sensor_comp, &uncomp_data, comp_data, &dev->calib_data);
		}
	} else {
		rslt = BME280_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API is used to compensate the pressure and/or
 * temperature and/or humidity data according to the component selected
 * by the user.
 */
int8_t BME280::bme280_compensate_data(uint8_t sensor_comp, const struct bme280_uncomp_data *uncomp_data,
									  struct bme280_data *comp_data, struct bme280_calib_data *calib_data) const {
	int8_t rslt = BME280_OK;

	if ((uncomp_data != NULL) && (comp_data != NULL) && (calib_data != NULL)) {
		/* Initialize to zero */
		comp_data->temperature = 0;
		comp_data->pressure = 0;
		comp_data->humidity = 0;

		/* If pressure or temperature component is selected */
		if (sensor_comp & (BME280_PRESS | BME280_TEMP | BME280_HUM)) {
			/* Compensate the temperature data */
			comp_data->temperature = compensate_temperature(uncomp_data, calib_data);
		}

		if (sensor_comp & BME280_PRESS) {
			/* Compensate the pressure data */
			comp_data->pressure = compensate_pressure(uncomp_data, calib_data);
		}

		if (sensor_comp & BME280_HUM) {
			/* Compensate the humidity data */
			comp_data->humidity = compensate_humidity(uncomp_data, calib_data);
		}
	} else {
		rslt = BME280_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API is used to calculate the maximum delay in milliseconds
 * required for the temperature/pressure/humidity(whichever are enabled)
 * measurement to complete.
 */
int8_t BME280::bme280_cal_meas_delay(uint32_t *max_delay, const struct bme280_settings *settings) {
	int8_t rslt = BME280_OK;
	uint8_t temp_osr;
	uint8_t pres_osr;
	uint8_t hum_osr;

	/* Array to map OSR config register value to actual OSR */
	uint8_t osr_sett_to_act_osr[] = {0, 1, 2, 4, 8, 16};

	if ((settings != NULL) && (max_delay != NULL)) {
		/* Mapping osr settings to the actual osr values e.g. 0b101 -> osr X16
		 */
		if (settings->osr_t <= BME280_OVERSAMPLING_16X) {
			temp_osr = osr_sett_to_act_osr[settings->osr_t];
		} else {
			temp_osr = BME280_OVERSAMPLING_MAX;
		}

		if (settings->osr_p <= BME280_OVERSAMPLING_16X) {
			pres_osr = osr_sett_to_act_osr[settings->osr_p];
		} else {
			pres_osr = BME280_OVERSAMPLING_MAX;
		}

		if (settings->osr_h <= BME280_OVERSAMPLING_16X) {
			hum_osr = osr_sett_to_act_osr[settings->osr_h];
		} else {
			hum_osr = BME280_OVERSAMPLING_MAX;
		}

		(*max_delay) = (uint32_t)((BME280_MEAS_OFFSET + (BME280_MEAS_DUR * temp_osr) +
								   ((BME280_MEAS_DUR * pres_osr) + BME280_PRES_HUM_MEAS_OFFSET) +
								   ((BME280_MEAS_DUR * hum_osr) + BME280_PRES_HUM_MEAS_OFFSET)));
	} else {
		rslt = BME280_E_NULL_PTR;
	}

	return rslt;
}

/****************************************************************************/
/**\name                        INTERNAL APIs                               */

/*!
 * @brief This internal API sets the oversampling settings for pressure,
 * temperature and humidity in the sensor.
 */
int8_t BME280::set_osr_settings(uint8_t desired_settings, const struct bme280_settings *settings,
								struct bme280_dev *dev) {
	int8_t rslt = BME280_W_INVALID_OSR_MACRO;

	if (desired_settings & BME280_SEL_OSR_HUM) {
		rslt = set_osr_humidity_settings(settings, dev);
	}

	if (desired_settings & (BME280_SEL_OSR_PRESS | BME280_SEL_OSR_TEMP)) {
		rslt = set_osr_press_temp_settings(desired_settings, settings, dev);
	}

	return rslt;
}

/*!
 * @brief This API sets the humidity oversampling settings of the sensor.
 */
int8_t BME280::set_osr_humidity_settings(const struct bme280_settings *settings, struct bme280_dev *dev) {
	int8_t rslt;
	uint8_t ctrl_hum;
	uint8_t ctrl_meas;
	uint8_t reg_addr = BME280_REG_CTRL_HUM;

	ctrl_hum = settings->osr_h & BME280_CTRL_HUM_MSK;

	/* Write the humidity control value in the register */
	rslt = bme280_set_regs(&reg_addr, &ctrl_hum, 1, dev);

	/* Humidity related changes will be only effective after a
	 * write operation to ctrl_meas register
	 */
	if (rslt == BME280_OK) {
		reg_addr = BME280_REG_CTRL_MEAS;
		rslt = bme280_get_regs(reg_addr, &ctrl_meas, 1, dev);

		if (rslt == BME280_OK) {
			rslt = bme280_set_regs(&reg_addr, &ctrl_meas, 1, dev);
		}
	}

	return rslt;
}

/*!
 * @brief This API sets the pressure and/or temperature oversampling settings
 * in the sensor according to the settings selected by the user.
 */
int8_t BME280::set_osr_press_temp_settings(uint8_t desired_settings, const struct bme280_settings *settings,
										   struct bme280_dev *dev) {
	int8_t rslt;
	uint8_t reg_addr = BME280_REG_CTRL_MEAS;
	uint8_t reg_data;

	rslt = bme280_get_regs(reg_addr, &reg_data, 1, dev);

	if (rslt == BME280_OK) {
		if (desired_settings & BME280_SEL_OSR_PRESS) {
			fill_osr_press_settings(&reg_data, settings);
		}

		if (desired_settings & BME280_SEL_OSR_TEMP) {
			fill_osr_temp_settings(&reg_data, settings);
		}

		/* Write the oversampling settings in the register */
		rslt = bme280_set_regs(&reg_addr, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This internal API sets the filter and/or standby duration settings
 * in the sensor according to the settings selected by the user.
 */
int8_t BME280::set_filter_standby_settings(uint8_t desired_settings, const struct bme280_settings *settings,
										   struct bme280_dev *dev) {
	int8_t rslt;
	uint8_t reg_addr = BME280_REG_CONFIG;
	uint8_t reg_data;

	rslt = bme280_get_regs(reg_addr, &reg_data, 1, dev);

	if (rslt == BME280_OK) {
		if (desired_settings & BME280_SEL_FILTER) {
			fill_filter_settings(&reg_data, settings);
		}

		if (desired_settings & BME280_SEL_STANDBY) {
			fill_standby_settings(&reg_data, settings);
		}

		/* Write the oversampling settings in the register */
		rslt = bme280_set_regs(&reg_addr, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This internal API fills the filter settings provided by the user
 * in the data buffer so as to write in the sensor.
 */
void BME280::fill_filter_settings(uint8_t *reg_data, const struct bme280_settings *settings) {
	*reg_data = BME280_SET_BITS(*reg_data, BME280_FILTER, settings->filter);
}

/*!
 * @brief This internal API fills the standby duration settings provided by
 * the user in the data buffer so as to write in the sensor.
 */
void BME280::fill_standby_settings(uint8_t *reg_data, const struct bme280_settings *settings) {
	*reg_data = BME280_SET_BITS(*reg_data, BME280_STANDBY, settings->standby_time);
}

/*!
 * @brief This internal API fills the pressure oversampling settings provided by
 * the user in the data buffer so as to write in the sensor.
 */
void BME280::fill_osr_press_settings(uint8_t *reg_data, const struct bme280_settings *settings) {
	*reg_data = BME280_SET_BITS(*reg_data, BME280_CTRL_PRESS, settings->osr_p);
}

/*!
 * @brief This internal API fills the temperature oversampling settings
 * provided by the user in the data buffer so as to write in the sensor.
 */
void BME280::fill_osr_temp_settings(uint8_t *reg_data, const struct bme280_settings *settings) {
	*reg_data = BME280_SET_BITS(*reg_data, BME280_CTRL_TEMP, settings->osr_t);
}

/*!
 * @brief This internal API parse the oversampling(pressure, temperature
 * and humidity), filter and standby duration settings and store in the
 * bme280_settings structure.
 */
void BME280::parse_device_settings(const uint8_t *reg_data, struct bme280_settings *settings) {
	settings->osr_h = BME280_GET_BITS_POS_0(reg_data[0], BME280_CTRL_HUM);
	settings->osr_p = BME280_GET_BITS(reg_data[2], BME280_CTRL_PRESS);
	settings->osr_t = BME280_GET_BITS(reg_data[2], BME280_CTRL_TEMP);
	settings->filter = BME280_GET_BITS(reg_data[3], BME280_FILTER);
	settings->standby_time = BME280_GET_BITS(reg_data[3], BME280_STANDBY);
}

/*!
 *  @brief This API is used to parse the pressure, temperature and
 *  humidity data and store it in the bme280_uncomp_data structure instance.
 */
void BME280::parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data) {
	/* Variables to store the sensor data */
	uint32_t data_xlsb;
	uint32_t data_lsb;
	uint32_t data_msb;

	/* Store the parsed register values for pressure data */
	data_msb = (uint32_t)reg_data[0] << BME280_12_BIT_SHIFT;
	data_lsb = (uint32_t)reg_data[1] << BME280_4_BIT_SHIFT;
	data_xlsb = (uint32_t)reg_data[2] >> BME280_4_BIT_SHIFT;
	uncomp_data->pressure = data_msb | data_lsb | data_xlsb;

	/* Store the parsed register values for temperature data */
	data_msb = (uint32_t)reg_data[3] << BME280_12_BIT_SHIFT;
	data_lsb = (uint32_t)reg_data[4] << BME280_4_BIT_SHIFT;
	data_xlsb = (uint32_t)reg_data[5] >> BME280_4_BIT_SHIFT;
	uncomp_data->temperature = data_msb | data_lsb | data_xlsb;

	/* Store the parsed register values for humidity data */
	data_msb = (uint32_t)reg_data[6] << BME280_8_BIT_SHIFT;
	data_lsb = (uint32_t)reg_data[7];
	uncomp_data->humidity = data_msb | data_lsb;
}

/*!
 * @brief This internal API writes the power mode in the sensor.
 */
int8_t BME280::write_power_mode(uint8_t sensor_mode, struct bme280_dev *dev) {
	int8_t rslt;
	uint8_t reg_addr = BME280_REG_PWR_CTRL;

	/* Variable to store the value read from power mode register */
	uint8_t sensor_mode_reg_val;

	/* Read the power mode register */
	rslt = bme280_get_regs(reg_addr, &sensor_mode_reg_val, 1, dev);

	/* Set the power mode */
	if (rslt == BME280_OK) {
		sensor_mode_reg_val = BME280_SET_BITS_POS_0(sensor_mode_reg_val, BME280_SENSOR_MODE, sensor_mode);

		/* Write the power mode in the register */
		rslt = bme280_set_regs(&reg_addr, &sensor_mode_reg_val, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This internal API puts the device to sleep mode.
 */
int8_t BME280::put_device_to_sleep(struct bme280_dev *dev) {
	int8_t rslt;
	uint8_t reg_data[4];
	struct bme280_settings settings;

	rslt = bme280_get_regs(BME280_REG_CTRL_HUM, reg_data, 4, dev);

	if (rslt == BME280_OK) {
		parse_device_settings(reg_data, &settings);
		rslt = BME280::bme280_soft_reset(dev);

		if (rslt == BME280_OK) {
			rslt = reload_device_settings(&settings, dev);
		}
	}

	return rslt;
}

/*!
 * @brief This internal API reloads the already existing device settings in
 * the sensor after soft reset.
 */
int8_t BME280::reload_device_settings(const struct bme280_settings *settings, struct bme280_dev *dev) {
	int8_t rslt;

	rslt = set_osr_settings(BME280_SEL_ALL_SETTINGS, settings, dev);

	if (rslt == BME280_OK) {
		rslt = set_filter_standby_settings(BME280_SEL_ALL_SETTINGS, settings, dev);
	}

	return rslt;
}

#ifdef BME280_DOUBLE_ENABLE

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 */
double BME280::compensate_temperature(const struct bme280_uncomp_data *uncomp_data,
									  struct bme280_calib_data *calib_data) {
	double var1;
	double var2;
	double temperature;
	double temperature_min = -40;
	double temperature_max = 85;

	var1 = (((double)uncomp_data->temperature) / 16384.0 - ((double)calib_data->dig_t1) / 1024.0);
	var1 = var1 * ((double)calib_data->dig_t2);
	var2 = (((double)uncomp_data->temperature) / 131072.0 - ((double)calib_data->dig_t1) / 8192.0);
	var2 = (var2 * var2) * ((double)calib_data->dig_t3);
	calib_data->t_fine = (int32_t)(var1 + var2);
	temperature = (var1 + var2) / 5120.0;

	if (temperature < temperature_min) {
		temperature = temperature_min;
	} else if (temperature > temperature_max) {
		temperature = temperature_max;
	}

	return temperature;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in double data type.
 */
double BME280::compensate_pressure(const struct bme280_uncomp_data *uncomp_data,
								   const struct bme280_calib_data *calib_data) {
	double var1;
	double var2;
	double var3;
	double pressure;
	double pressure_min = 30000.0;
	double pressure_max = 110000.0;

	var1 = ((double)calib_data->t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double)calib_data->dig_p6) / 32768.0;
	var2 = var2 + var1 * ((double)calib_data->dig_p5) * 2.0;
	var2 = (var2 / 4.0) + (((double)calib_data->dig_p4) * 65536.0);
	var3 = ((double)calib_data->dig_p3) * var1 * var1 / 524288.0;
	var1 = (var3 + ((double)calib_data->dig_p2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double)calib_data->dig_p1);

	/* Avoid exception caused by division by zero */
	if (var1 > (0.0)) {
		pressure = 1048576.0 - (double)uncomp_data->pressure;
		pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
		var1 = ((double)calib_data->dig_p9) * pressure * pressure / 2147483648.0;
		var2 = pressure * ((double)calib_data->dig_p8) / 32768.0;
		pressure = pressure + (var1 + var2 + ((double)calib_data->dig_p7)) / 16.0;

		if (pressure < pressure_min) {
			pressure = pressure_min;
		} else if (pressure > pressure_max) {
			pressure = pressure_max;
		}
	} else /* Invalid case */
	{
		pressure = pressure_min;
	}

	return pressure;
}

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in double data type.
 */
double BME280::compensate_humidity(const struct bme280_uncomp_data *uncomp_data,
								   const struct bme280_calib_data *calib_data) {
	double humidity;
	double humidity_min = 0.0;
	double humidity_max = 100.0;
	double var1;
	double var2;
	double var3;
	double var4;
	double var5;
	double var6;

	var1 = ((double)calib_data->t_fine) - 76800.0;
	var2 = (((double)calib_data->dig_h4) * 64.0 + (((double)calib_data->dig_h5) / 16384.0) * var1);
	var3 = uncomp_data->humidity - var2;
	var4 = ((double)calib_data->dig_h2) / 65536.0;
	var5 = (1.0 + (((double)calib_data->dig_h3) / 67108864.0) * var1);
	var6 = 1.0 + (((double)calib_data->dig_h6) / 67108864.0) * var1 * var5;
	var6 = var3 * var4 * (var5 * var6);
	humidity = var6 * (1.0 - ((double)calib_data->dig_h1) * var6 / 524288.0);

	if (humidity > humidity_max) {
		humidity = humidity_max;
	} else if (humidity < humidity_min) {
		humidity = humidity_min;
	}

	return humidity;
}

#else

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in integer data type.
 */
int32_t compensate_temperature(const struct bme280_uncomp_data *uncomp_data, struct bme280_calib_data *calib_data) {
	int32_t var1;
	int32_t var2;
	int32_t temperature;
	int32_t temperature_min = -4000;
	int32_t temperature_max = 8500;

	var1 = (int32_t)((uncomp_data->temperature / 8) - ((int32_t)calib_data->dig_t1 * 2));
	var1 = (var1 * ((int32_t)calib_data->dig_t2)) / 2048;
	var2 = (int32_t)((uncomp_data->temperature / 16) - ((int32_t)calib_data->dig_t1));
	var2 = (((var2 * var2) / 4096) * ((int32_t)calib_data->dig_t3)) / 16384;
	calib_data->t_fine = var1 + var2;
	temperature = (calib_data->t_fine * 5 + 128) / 256;

	if (temperature < temperature_min) {
		temperature = temperature_min;
	} else if (temperature > temperature_max) {
		temperature = temperature_max;
	}

	return temperature;
}
#ifndef BME280_32BIT_ENABLE /* 64 bit compensation for pressure data */

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type with higher
 * accuracy.
 */
uint32_t compensate_pressure(const struct bme280_uncomp_data *uncomp_data, const struct bme280_calib_data *calib_data) {
	int64_t var1;
	int64_t var2;
	int64_t var3;
	int64_t var4;
	uint32_t pressure;
	uint32_t pressure_min = 3000000;
	uint32_t pressure_max = 11000000;

	var1 = ((int64_t)calib_data->t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)calib_data->dig_p6;
	var2 = var2 + ((var1 * (int64_t)calib_data->dig_p5) * 131072);
	var2 = var2 + (((int64_t)calib_data->dig_p4) * 34359738368);
	var1 = ((var1 * var1 * (int64_t)calib_data->dig_p3) / 256) + ((var1 * ((int64_t)calib_data->dig_p2) * 4096));
	var3 = ((int64_t)1) * 140737488355328;
	var1 = (var3 + var1) * ((int64_t)calib_data->dig_p1) / 8589934592;

	/* To avoid divide by zero exception */
	if (var1 != 0) {
		var4 = 1048576 - uncomp_data->pressure;
		var4 = (((var4 * INT64_C(2147483648)) - var2) * 3125) / var1;
		var1 = (((int64_t)calib_data->dig_p9) * (var4 / 8192) * (var4 / 8192)) / 33554432;
		var2 = (((int64_t)calib_data->dig_p8) * var4) / 524288;
		var4 = ((var4 + var1 + var2) / 256) + (((int64_t)calib_data->dig_p7) * 16);
		pressure = (uint32_t)(((var4 / 2) * 100) / 128);

		if (pressure < pressure_min) {
			pressure = pressure_min;
		} else if (pressure > pressure_max) {
			pressure = pressure_max;
		}
	} else {
		pressure = pressure_min;
	}

	return pressure;
}
#else                       /* 32 bit compensation for pressure data */

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type.
 */
uint32_t compensate_pressure(const struct bme280_uncomp_data *uncomp_data, const struct bme280_calib_data *calib_data) {
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	uint32_t var5;
	uint32_t pressure;
	uint32_t pressure_min = 30000;
	uint32_t pressure_max = 110000;

	var1 = (((int32_t)calib_data->t_fine) / 2) - (int32_t)64000;
	var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)calib_data->dig_p6);
	var2 = var2 + ((var1 * ((int32_t)calib_data->dig_p5)) * 2);
	var2 = (var2 / 4) + (((int32_t)calib_data->dig_p4) * 65536);
	var3 = (calib_data->dig_p3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
	var4 = (((int32_t)calib_data->dig_p2) * var1) / 2;
	var1 = (var3 + var4) / 262144;
	var1 = (((32768 + var1)) * ((int32_t)calib_data->dig_p1)) / 32768;

	/* Avoid exception caused by division by zero */
	if (var1) {
		var5 = (uint32_t)((uint32_t)1048576) - uncomp_data->pressure;
		pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;

		if (pressure < 0x80000000) {
			pressure = (pressure << 1) / ((uint32_t)var1);
		} else {
			pressure = (pressure / (uint32_t)var1) * 2;
		}

		var1 = (((int32_t)calib_data->dig_p9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
		var2 = (((int32_t)(pressure / 4)) * ((int32_t)calib_data->dig_p8)) / 8192;
		pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + calib_data->dig_p7) / 16));

		if (pressure < pressure_min) {
			pressure = pressure_min;
		} else if (pressure > pressure_max) {
			pressure = pressure_max;
		}
	} else {
		pressure = pressure_min;
	}

	return pressure;
}
#endif

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in integer data type.
 */
uint32_t compensate_humidity(const struct bme280_uncomp_data *uncomp_data, const struct bme280_calib_data *calib_data) {
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	uint32_t humidity;
	uint32_t humidity_max = 102400;

	var1 = calib_data->t_fine - ((int32_t)76800);
	var2 = (int32_t)(uncomp_data->humidity * 16384);
	var3 = (int32_t)(((int32_t)calib_data->dig_h4) * 1048576);
	var4 = ((int32_t)calib_data->dig_h5) * var1;
	var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
	var2 = (var1 * ((int32_t)calib_data->dig_h6)) / 1024;
	var3 = (var1 * ((int32_t)calib_data->dig_h3)) / 2048;
	var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
	var2 = ((var4 * ((int32_t)calib_data->dig_h2)) + 8192) / 16384;
	var3 = var5 * var2;
	var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
	var5 = var3 - ((var4 * ((int32_t)calib_data->dig_h1)) / 16);
	var5 = (var5 < 0 ? 0 : var5);
	var5 = (var5 > 419430400 ? 419430400 : var5);
	humidity = (uint32_t)(var5 / 4096);

	if (humidity > humidity_max) {
		humidity = humidity_max;
	}

	return humidity;
}
#endif

/*!
 * @brief This internal API reads the calibration data from the sensor, parse
 * it and store in the device structure.
 */
int8_t BME280::get_calib_data(struct bme280_dev *dev) {
	int8_t rslt;
	uint8_t reg_addr = BME280_REG_TEMP_PRESS_CALIB_DATA;

	/* Array to store calibration data */
	uint8_t calib_data[BME280_LEN_TEMP_PRESS_CALIB_DATA] = {0};

	/* Read the calibration data from the sensor */
	rslt = bme280_get_regs(reg_addr, calib_data, BME280_LEN_TEMP_PRESS_CALIB_DATA, dev);

	if (rslt == BME280_OK) {
		/* Parse temperature and pressure calibration data and store
		 * it in device structure
		 */
		parse_temp_press_calib_data(calib_data, dev);
		reg_addr = BME280_REG_HUMIDITY_CALIB_DATA;

		/* Read the humidity calibration data from the sensor */
		rslt = bme280_get_regs(reg_addr, calib_data, BME280_LEN_HUMIDITY_CALIB_DATA, dev);

		if (rslt == BME280_OK) {
			/* Parse humidity calibration data and store it in
			 * device structure
			 */
			parse_humidity_calib_data(calib_data, dev);
		}
	}

	return rslt;
}

/*!
 * @brief This internal API interleaves the register address between the
 * register data buffer for burst write operation.
 */
void BME280::interleave_reg_addr(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint32_t len) {
	uint32_t index;

	for (index = 1; index < len; index++) {
		temp_buff[(index * 2) - 1] = reg_addr[index];
		temp_buff[index * 2] = reg_data[index];
	}
}

/*!
 *  @brief This internal API is used to parse the temperature and
 *  pressure calibration data and store it in device structure.
 */
void BME280::parse_temp_press_calib_data(const uint8_t *reg_data, struct bme280_dev *dev) {
	struct bme280_calib_data *calib_data = &dev->calib_data;

	calib_data->dig_t1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
	calib_data->dig_t2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
	calib_data->dig_t3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
	calib_data->dig_p1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
	calib_data->dig_p2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
	calib_data->dig_p3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
	calib_data->dig_p4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
	calib_data->dig_p5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
	calib_data->dig_p6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
	calib_data->dig_p7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
	calib_data->dig_p8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
	calib_data->dig_p9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
	calib_data->dig_h1 = reg_data[25];
}

/*!
 *  @brief This internal API is used to parse the humidity calibration data
 *  and store it in device structure.
 */
void BME280::parse_humidity_calib_data(const uint8_t *reg_data, struct bme280_dev *dev) {
	struct bme280_calib_data *calib_data = &dev->calib_data;
	int16_t dig_h4_lsb;
	int16_t dig_h4_msb;
	int16_t dig_h5_lsb;
	int16_t dig_h5_msb;

	calib_data->dig_h2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
	calib_data->dig_h3 = reg_data[2];
	dig_h4_msb = (int16_t)(int8_t)reg_data[3] * 16;
	dig_h4_lsb = (int16_t)(reg_data[4] & 0x0F);
	calib_data->dig_h4 = dig_h4_msb | dig_h4_lsb;
	dig_h5_msb = (int16_t)(int8_t)reg_data[5] * 16;
	dig_h5_lsb = (int16_t)(reg_data[4] >> 4);
	calib_data->dig_h5 = dig_h5_msb | dig_h5_lsb;
	calib_data->dig_h6 = (int8_t)reg_data[6];
}

/*!
 * @brief This internal API is used to identify the settings which the user
 * wants to modify in the sensor.
 */
uint8_t BME280::are_settings_changed(uint8_t sub_settings, uint8_t desired_settings) {
	uint8_t settings_changed = FALSE;

	if (sub_settings & desired_settings) {
		/* User wants to modify this particular settings */
		settings_changed = TRUE;
	} else {
		/* User don't want to modify this particular settings */
		settings_changed = FALSE;
	}

	return settings_changed;
}

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
int8_t BME280::null_ptr_check(const struct bme280_dev *dev) {
	int8_t rslt;

	if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_us == NULL)) {
		/* Device structure pointer is not valid */
		rslt = BME280_E_NULL_PTR;
	} else {
		/* Device structure is fine */
		rslt = BME280_OK;
	}

	return rslt;
}

double BME280::getTemperature() {
	bme280_data comp_data{};
	uint8_t status_reg;

	constexpr uint8_t sampleCount{10};
	uint8_t idx = 0;

	uint8_t sensorMode = 0;
	bme280_get_sensor_mode(&sensorMode, &this->sensor);

	while (idx < sampleCount) {
		auto rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, &this->sensor);
		printf("Read status register 0x%x\n", rslt);
		if (status_reg & BME280_STATUS_MEAS_DONE) {
			sensorDelay(this->measurementPeriod, &this->sensor);
			rslt = bme280_get_sensor_data(BME280_TEMP, &comp_data, &this->sensor);

			if (rslt == BME280_OK) {
				break;
			}
		} else {
			sensorDelay(this->measurementPeriod, &this->sensor);
		}
		idx++;
	}

	return comp_data.temperature;
}
