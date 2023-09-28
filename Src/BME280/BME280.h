/**********************************************
* @brief  Bare-bone driver for the BME280_DEFS sensor.
* @author Nikolaos Xenofon Gkloumpos
* @date   18/09/2023
**********************************************/

#pragma once

#include "Interfaces/I2CInterface.hh"
#include "BME280_defs.hh"

struct filteredMeasurements {
   double pressure;
   double temperature;
   double humidity;
};

struct rawMeasurements {
   uint32_t pressure;
   uint32_t temperature;
   uint32_t humidity;
};

class BME280 {
 private:
   I2CInterface &i2cBus;
   uint8_t address;
   struct filteredMeasurements currentMeasurements = {};

   void readRegister(uint8_t registerAddress, uint8_t *returnedData, uint32_t length);
   void writeRegister(uint8_t registerAddress, uint8_t *dataToWrite, uint32_t length);
 public:
   BME280(I2CInterface& i2cBus, uint8_t address);
   ~BME280() = default;

   BME280_DEFS::sensorPowerMode_t getSensorMode();
   void setSensorMode(BME280_DEFS::sensorPowerMode_t newState);
   void softReset();
   void getSensorData();

   double getTemperature();
   double getHumidity();
   double getPressure();
};
