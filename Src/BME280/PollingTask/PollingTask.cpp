/************************
 * @brief  Freertos Task to poll the sensor for data periodically.
 * @author nick
 * @date   10/21/23
 ************************/

#include "BME280/BME280.h"
#include "FreeRTOS.h"
#include "task.h"

[[noreturn]] void BME280::pollingTask(void * arguments) {
	auto instance = (BME280*) arguments;
	const TickType_t taskDelay = 1000 / instance->pollingRate;

	while (true) {
		instance->getSensorData();

		vTaskDelay(taskDelay);
	}
}
