/************************************************
 * @brief  File containing register addresses and types to operate
 * the BME280_DEFS sensor.
 * @author Nikolaos Xenofon Gkloumpos
 * @date   18/09/2023
 **********************************************/

#pragma once

#include <cstdint>

namespace BME280_DEFS {

/* API error codes */
constexpr int8_t E_NULL_PTR = -1;
constexpr int8_t E_COMM_FAIL = -2;
constexpr int8_t E_INVALID_LEN = -3;
constexpr int8_t E_DEV_NOT_FOUND = -4;
constexpr int8_t E_SLEEP_MODE_FAIL = -5;
constexpr int8_t E_NVM_COPY_FAILED = -6;

/* BME280_DEFS chip identifier */
constexpr int8_t CHIP_ID = 0x60;

/* I2C addresses */
constexpr uint8_t I2C_ADDR_PRIM = 0x76;
constexpr uint8_t I2C_ADDR_SEC = 0x77;

/* Register Address */
constexpr uint8_t REG_CHIP_ID = 0xD0;
constexpr uint8_t REG_RESET = 0xE0;
constexpr uint8_t REG_TEMP_PRESS_CALIB_DATA = 0x88;
constexpr uint8_t REG_HUMIDITY_CALIB_DATA = 0xE1;
constexpr uint8_t REG_CTRL_HUM = 0xF2;
constexpr uint8_t REG_STATUS = 0xF3;
constexpr uint8_t REG_PWR_CTRL = 0xF4;
constexpr uint8_t REG_CTRL_MEAS = 0xF4;
constexpr uint8_t REG_CONFIG = 0xF5;
constexpr uint8_t REG_DATA = 0xF7;

/* Macros related to size */
constexpr uint8_t LEN_TEMP_PRESS_CALIB_DATA = 26;
constexpr uint8_t LEN_HUMIDITY_CALIB_DATA = 7;
constexpr uint8_t LEN_P_T_H_DATA = 8;

/* Sensor Power states */
typedef enum { ASLEEP, NORMAL, FORCED } sensorPowerMode_t;

constexpr uint8_t SENSOR_MODE_MSK = 0x03;
constexpr uint8_t SENSOR_MODE_POS = 0x00;

/* Soft reset command */
constexpr uint8_t SOFT_RESET_COMMAND = 0xB6;
constexpr uint8_t STATUS_IM_UPDATE = 0x01;
constexpr uint8_t STATUS_MEAS_DONE = 0x08;

/* Oversampling options. */
constexpr uint8_t NO_OVERSAMPLING = 0x00;
constexpr uint8_t OVERSAMPLING_1X = 0x01;
constexpr uint8_t OVERSAMPLING_2X = 0x02;
constexpr uint8_t OVERSAMPLING_4X = 0x03;
constexpr uint8_t OVERSAMPLING_8X = 0x04;
constexpr uint8_t OVERSAMPLING_16X = 0x05;
constexpr uint8_t OVERSAMPLING_MAX = 16;

constexpr uint8_t CTRL_HUM_MSK = 0x07;
constexpr uint8_t CTRL_HUM_POS = 0x00;
constexpr uint8_t CTRL_PRESS_MSK = 0x1C;
constexpr uint8_t CTRL_PRESS_POS = 0x02;
constexpr uint8_t CTRL_TEMP_MSK = 0xE0;
constexpr uint8_t CTRL_TEMP_POS = 0x05;

/* Measurement delay calculation macros  */
#define MEAS_OFFSET UINT16_C(1250)
#define MEAS_DUR UINT16_C(2300)
#define PRES_HUM_MEAS_OFFSET UINT16_C(575)
#define MEAS_SCALING_FACTOR UINT16_C(1000)
#define STARTUP_DELAY UINT16_C(2000)

/* Length macros */
constexpr uint8_t MAX_LEN = 10;

/* Standby duration selection macros */
#define STANDBY_TIME_0_5_MS (0x00)
#define STANDBY_TIME_62_5_MS (0x01)
#define STANDBY_TIME_125_MS (0x02)
#define STANDBY_TIME_250_MS (0x03)
#define STANDBY_TIME_500_MS (0x04)
#define STANDBY_TIME_1000_MS (0x05)
#define STANDBY_TIME_10_MS (0x06)
#define STANDBY_TIME_20_MS (0x07)

#define STANDBY_MSK UINT8_C(0xE0)
#define STANDBY_POS UINT8_C(0x05)

/* Filter coefficient selection macros */
#define FILTER_COEFF_OFF (0x00)
#define FILTER_COEFF_2 (0x01)
#define FILTER_COEFF_4 (0x02)
#define FILTER_COEFF_8 (0x03)
#define FILTER_COEFF_16 (0x04)

#define FILTER_MSK UINT8_C(0x1C)
#define FILTER_POS UINT8_C(0x02)

struct calibrationData {
	uint16_t dig_t1;
	int16_t dig_t2;
	int16_t dig_t3;
	uint16_t dig_p1;
	int16_t dig_p2;
	int16_t dig_p3;
	int16_t dig_p4;
	int16_t dig_p5;
	int16_t dig_p6;
	int16_t dig_p7;
	int16_t dig_p8;
	int16_t dig_p9;
	uint8_t dig_h1;
	int16_t dig_h2;
	uint8_t dig_h3;
	int16_t dig_h4;
	int16_t dig_h5;
	int8_t dig_h6;
	int32_t t_fine;
};

struct deviceData {
	/*! Chip Id */
	uint8_t chip_id;
	/*! Trim data */
	struct calibrationData calib_data;
};

} // namespace BME280_DEFS