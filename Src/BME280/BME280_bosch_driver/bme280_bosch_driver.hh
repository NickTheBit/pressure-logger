/**********************************************
* @brief  Bare-bone driver for the BME280_DEFS sensor.
		  Copied and adapted from bosch sensorteks github page.
 **********************************************/

#pragma once

int8_t bme280_init(struct bme280_dev *dev);
int8_t bme280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint32_t len, struct bme280_dev *dev);
int8_t bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bme280_dev *dev);
int8_t bme280_set_sensor_settings(uint8_t desired_settings, const struct bme280_settings *settings,
								  struct bme280_dev *dev);
int8_t bme280_get_sensor_settings(struct bme280_settings *settings, struct bme280_dev *dev);
int8_t bme280_set_sensor_mode(uint8_t sensor_mode, struct bme280_dev *dev);
int8_t bme280_get_sensor_mode(uint8_t *sensor_mode, struct bme280_dev *dev);
int8_t bme280_soft_reset(struct bme280_dev *dev);
int8_t bme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data *comp_data, struct bme280_dev *dev);
int8_t bme280_compensate_data(uint8_t sensor_comp, const struct bme280_uncomp_data *uncomp_data,
							  struct bme280_data *comp_data, struct bme280_calib_data *calib_data);
int8_t bme280_cal_meas_delay(uint32_t *max_delay, const struct bme280_settings *settings);

/**\name Internal macros */
/* To identify osr settings selected by user */
#define OVERSAMPLING_SETTINGS UINT8_C(0x07)

/* To identify filter and standby settings selected by user */
#define FILTER_STANDBY_SETTINGS UINT8_C(0x18)

int8_t put_device_to_sleep(struct bme280_dev *dev);
int8_t write_power_mode(uint8_t sensor_mode, struct bme280_dev *dev);
int8_t null_ptr_check(const struct bme280_dev *dev);
void interleave_reg_addr(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint32_t len);
int8_t get_calib_data(struct bme280_dev *dev);
void parse_temp_press_calib_data(const uint8_t *reg_data, struct bme280_dev *dev);
void parse_humidity_calib_data(const uint8_t *reg_data, struct bme280_dev *dev);
uint8_t are_settings_changed(uint8_t sub_settings, uint8_t desired_settings);
int8_t set_osr_humidity_settings(const struct bme280_settings *settings, struct bme280_dev *dev);
int8_t set_osr_settings(uint8_t desired_settings, const struct bme280_settings *settings,
							   struct bme280_dev *dev);
int8_t set_osr_press_temp_settings(uint8_t desired_settings, const struct bme280_settings *settings,
										  struct bme280_dev *dev);
void fill_osr_press_settings(uint8_t *reg_data, const struct bme280_settings *settings);
void fill_osr_temp_settings(uint8_t *reg_data, const struct bme280_settings *settings);
int8_t set_filter_standby_settings(uint8_t desired_settings, const struct bme280_settings *settings,
										  struct bme280_dev *dev);
void fill_filter_settings(uint8_t *reg_data, const struct bme280_settings *settings);
void fill_standby_settings(uint8_t *reg_data, const struct bme280_settings *settings);
void parse_device_settings(const uint8_t *reg_data, struct bme280_settings *settings);
void parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data);
int8_t reload_device_settings(const struct bme280_settings *settings, struct bme280_dev *dev);

#ifdef BME280_DOUBLE_ENABLE

double compensate_pressure(const struct bme280_uncomp_data *uncomp_data,
								  const struct bme280_calib_data *calib_data);
double compensate_humidity(const struct bme280_uncomp_data *uncomp_data,
								  const struct bme280_calib_data *calib_data);
double compensate_temperature(const struct bme280_uncomp_data *uncomp_data,
									 struct bme280_calib_data *calib_data);

#else

int32_t compensate_temperature(const struct bme280_uncomp_data *uncomp_data,
									  struct bme280_calib_data *calib_data);

uint32_t compensate_pressure(const struct bme280_uncomp_data *uncomp_data,
									const struct bme280_calib_data *calib_data);

uint32_t compensate_humidity(const struct bme280_uncomp_data *uncomp_data,
									const struct bme280_calib_data *calib_data);

#endif