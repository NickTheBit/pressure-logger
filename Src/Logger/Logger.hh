/**********************************************************************
 * @brief  Logging utility, supports interchangeable output channels
 * @author nick
 * @date   7/10/24
 **********************************************************************/

#pragma once

#include "Interfaces/UARTInterface.hh"

template <typename Arch> class Logger {
  public:
	typedef enum { ERROR, WARNING, INFO, DEBUG } logError_t;

	void write(logError_t errorType, std::string &message) {
		switch (errorType) {
			case DEBUG: {
				message.insert(0, "DBG: ");
				break;
			}
			case INFO: {
				message.insert(0, "INF: ");
				break;
			}
			case WARNING: {
				message.insert(0, "WRN: ");
				break;
			}
			case ERROR: {
				message.insert(0, "ERR: ");
				break;
			}
		}
		static_cast<Arch *>(this)->writeImpl(message);
	}
};
