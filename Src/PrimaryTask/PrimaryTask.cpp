/*********************************************
 * @brief Entry point for user code in the Template, if tasks
 * spawn like a tree, this is the task that spawns them all.
 * @author Nikolaos Xenofon Gkloumpos
 * @date 25/05/2023
 *********************************************/

#include "PrimaryTask.hh"
#include "Architectures/RP2040/PR2040_GPIO/RP2040_GPIOPin.hh"
#include "Architectures/RP2040/RP2040_I2C/RP2040_I2C.h"
#include "Architectures/RP2040/RP2040_UART/RP2040_UART.hh"
#include "BME280/BME280.h"
#include "Logging/Logging.hh"
#include "hardware/uart.h"
#include "main/main.h"

[[noreturn]] void primaryTask(void *arguments) {
	// Configure the Pico's on-board LED as a heartbeat indicator.
	auto uartBus = RP2040_UART(*uart0, 115200);
	auto logger = Logging(uartBus);

	auto heartbeatPin = RP2040_GPIOPin(PICO_DEFAULT_LED_PIN, GPIO_OUT);
	auto i2cBus1 = RP2040_I2C(i2c0, 20, 21, false, 1000 * 100);

	auto bme = BME280(i2cBus1, 0x76, 10);

	while (true) {
		double humid = bme.getHumidity();
		double temperature = bme.getTemperature();
		double pressure = bme.getPressure();
		printf("Humidity: %2.2f\tTemperature: %2.2f\tPressure: %2.2f\n\r",
			   humid, temperature, pressure);

		// Starting a heartbeat to indicate proper initialization.
		heartbeatPin.write(HAL::GPIO_STATE::LOW);
		vTaskDelay(970);
		heartbeatPin.write(HAL::GPIO_STATE::HIGH);
		vTaskDelay(30);
	}
}