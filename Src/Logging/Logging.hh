/**********************************************************************
 * @brief  Logging utility, supports interchangeable output channels
 * @author nick
 * @date   7/10/24
 **********************************************************************/

#pragma once

#include "Interfaces/UARTInterface.hh"

class Logging {
  public:
	typedef enum { ERROR, WARNING, INFO, DEBUG } logError_t;
	explicit Logging(HAL::UARTInterface &outputChannel);
	~Logging() = default;
	void logWrite(logError_t errorType, std::string &message);

  private:
	HAL::UARTInterface &outputChannel;
};
