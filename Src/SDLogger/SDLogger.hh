/*********************************************
 * @brief  SPI SD card implementation of the logger interface.
 * @author Nikolaos Xenofon Gkloumpos
 * @date 15/02/2025
 *********************************************/

#pragma once

#include "ff.h"

#include <Logger/Logger.hh>
#include <f_util.h>
#include "SDLogger/hw_config.hh"
#include <sd_card.h>

class SDLogger : public Logger<SDLogger> {
  private:
	sd_card_t *pSD = nullptr;
	FIL fil;
  public:
	SDLogger() {
		pSD = sd_get_by_num(0);
		FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
		if (fr != FR_OK) panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);

		const auto filename = "log.txt";
		fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
		if (fr != FR_OK && fr != FR_EXIST) panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
	}
	~SDLogger() {
		if (const auto fr = f_close(&fil); fr != FR_OK) {
			printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
		}
		f_unmount(pSD->pcName);
	}

	void writeImpl(std::string &message) {
		if (f_printf(&fil, "%s\n", message.c_str()) < 0) {
			printf("Error");
		}
	}
};
