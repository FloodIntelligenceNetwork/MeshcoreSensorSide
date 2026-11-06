#ifndef LIQUID_LEVEL_DETECTION_H
#define LIQUID_LEVEL_DETECTION_H

#include <Arduino.h>

class LiquidLevelDetection {
public:
  // serial: an already-created hardware port, e.g. Serial1.
  // Start the port yourself first: Serial1.begin(115200);
  LiquidLevelDetection(Stream &serial, uint8_t device_addr = 0x01);

  // Prepare the sensor link (does not configure the UART)
  bool begin();

  // Get current range (unit: meter)
  float getRange();

  // Get empty height (distance from sensor to liquid surface, unit: meter)
  float getEmptyHeight();

  // Set installation height (unit: centimeter)
  bool setInstallationHeight(float height_cm);

  // Get installation height (return unit: meter)
  float getInstallationHeight();

  // Get water level height (unit: meter)
  float getWaterLevel();

private:
  Stream *_serial;
  uint8_t _device_addr;

  // Modbus functions
  bool sendModbusCommand(uint16_t reg_addr, uint16_t value);
  float readModbusRegister(uint16_t reg_addr);
  uint16_t calculateCRC16(uint8_t *buffer, uint8_t length);
};

#endif