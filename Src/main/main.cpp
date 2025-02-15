/*********************************************
 * @brief Entry point for kernel code in the Template,
 * do not modify this file, only rtos functions and the
 * primary task should be initialized here.
 * @author Nikolaos Xenofon Gkloumpos
 * @date  25/05/2023
 *********************************************/

#include "main.h"
#include "PrimaryTask/PrimaryTask.hh"
#include "FreeRTOSConfig.h"

/*
 * RUNTIME START
 */
int main() {
	// Enable STDIO
	stdio_init_all();

	// Setting up primary Task.
	static StackType_t primaryTask_puxBuffer[configMINIMAL_STACK_SIZE];
	static StaticTask_t primaryTask_tcbBuffer;

	xTaskCreateStatic(primaryTask, "primaryTask", configMINIMAL_STACK_SIZE, nullptr, 1, primaryTask_puxBuffer, &primaryTask_tcbBuffer);

	// Reporting Firmware info on UART
	printf("*************************************\n\t%s\n----------------------"
		   "---------------\nVersion: %d.%d.%d\nCommit hash: "
		   "%x\n*************************************\n",
		   PROJECT_NAME, PROJECT_VERSION_MAJOR, PROJECT_VERSION_MINOR, PROJECT_VERSION_PATCH, GIT_HASH);

	// Start the FreeRTOS scheduler
	// Only proceed with valid tasks
	vTaskStartScheduler();

	// If the scheduler doesn't take over, it should be handled here.
	while (true) {
		// NOP
	};
}
