/*********************************************
 * @brief  SPI SD card implementation of the logger interface.
 * @author Nikolaos Xenofon Gkloumpos
 * @date 15/02/2025
 *********************************************/

#pragma once

#include "ff.h"

#include "SDLogger/hw_config.hh"
#include <FreeRTOS.h>
#include <Logger/Logger.hh>
#include <f_util.h>
#include <sd_card.h>
#include <task.h>

class SDLogger : public Logger<SDLogger> {
  private:
	sd_card_t *pSD;
	FIL fil;

  public:
	SDLogger() {
		sd_init_driver();
		pSD = sd_get_by_num(0);
		FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
		uint8_t retry_count = 0;
		if (fr != FR_OK && retry_count < 20) {
			// todo: Not a great idea to use FreeRTOS specific functions in here.
			vTaskDelay(10);
			fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
			retry_count++;
		}

		if (retry_count >= 20) {
			panic("Too many re-attempts, aborting");
		}
	}
	~SDLogger() { f_unmount(pSD->pcName); }

	void writeImpl(std::string &message) {
		uint8_t retry_count = 0;
		const auto filename = "log.txt";
		FRESULT fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
		retry_count = 0;
		while (fr != FR_OK && retry_count < 20) {
			vTaskDelay(100);
			fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
			retry_count++;
		}

		if (retry_count >= 20) {
			panic("Too many re-attempts, aborting");
		}
		if (f_printf(&fil, "%s\n", message.c_str()) < 0) {
			printf("Error");
		}
		// Closing the file, finalizing the writing.
		if (const auto fr = f_close(&fil); fr != FR_OK) {
			printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
		}
	}
};
