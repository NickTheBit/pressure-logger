/**********************************************************************
 * @brief  Logging utility, supports interchangeable output channels
 * @author nick
 * @date   7/10/24
 **********************************************************************/

#include "Logging.hh"

Logging::Logging(HAL::UARTInterface &outputChannel)
	: outputChannel(outputChannel) {
}

/**
 * @brief Logs output with tag and newline.
 * @param errorType
 * @param message In string form
 * @note: todo: Weaker version with char* may be appreciated.
 */
void Logging::logWrite(logError_t errorType, std::string &message) {
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
	message.append("\n");
	outputChannel.write(message);
}
