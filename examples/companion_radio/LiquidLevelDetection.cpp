#include "LiquidLevelDetection.h"

// Max time to wait for the sensor's reply. The function returns as soon as the
// full reply has arrived, so this is only reached when the sensor doesn't answer.
static const uint32_t RESPONSE_TIMEOUT_MS = 200;

// Collect up to 'expected' bytes into buf, or stop when the timeout expires.
// Returns the number of bytes actually received.
static size_t receiveFrame(Stream *s, uint8_t *buf, size_t maxLen, size_t expected, uint32_t timeoutMs) {
  size_t n = 0;
  uint32_t start = millis();
  while ((millis() - start) < timeoutMs && n < expected) {
    while (s->available() && n < maxLen) {
      buf[n++] = (uint8_t)s->read();
    }
    if (n < expected) {
      delay(1);
    }
  }
  return n;
}

LiquidLevelDetection::LiquidLevelDetection(Stream &serial, uint8_t device_addr)
    : _serial(&serial), _device_addr(device_addr) {}

bool LiquidLevelDetection::begin() {
  // The UART itself is started by the caller (e.g. Serial1.begin(115200)).
  // Just clear any stale bytes left in the receive buffer.
  while (_serial->available()) {
    _serial->read();
  }
  return true;
}

float LiquidLevelDetection::getRange() {
  // Register address 0x07D4, read current range (unit: meter)
  return readModbusRegister(0x07D4);
}

float LiquidLevelDetection::getEmptyHeight() {
  // Register address 0x0001, unit mm converted to m
  return readModbusRegister(0x0001) / 1000.0;
}

bool LiquidLevelDetection::setInstallationHeight(float height_cm) {
  // Register address 0x0005, input unit: centimeter
  return sendModbusCommand(0x0005, (uint16_t)height_cm);
}

float LiquidLevelDetection::getInstallationHeight() {
  // Register address 0x0005, unit cm converted to m
  return readModbusRegister(0x0005) / 100.0;
}

float LiquidLevelDetection::getWaterLevel() {
  // Register address 0x0003, unit mm converted to m
  return readModbusRegister(0x0003) / 1000.0;
}

bool LiquidLevelDetection::sendModbusCommand(uint16_t reg_addr, uint16_t value) {
  uint8_t buffer[8];
  buffer[0] = _device_addr;    // Device address
  buffer[1] = 0x06;            // Function code: write single register
  buffer[2] = reg_addr >> 8;   // Register address high byte
  buffer[3] = reg_addr & 0xFF; // Register address low byte
  buffer[4] = value >> 8;      // Data high byte
  buffer[5] = value & 0xFF;    // Data low byte

  uint16_t crc = calculateCRC16(buffer, 6);
  buffer[6] = crc & 0xFF; // CRC low byte
  buffer[7] = crc >> 8;   // CRC high byte

  // Drop any stale bytes so they can't be mistaken for the reply
  while (_serial->available()) {
    _serial->read();
  }

  _serial->write(buffer, 8);
  _serial->flush(); // wait until the last bit has left the UART

  // A successful write is confirmed by the sensor echoing the same 8 bytes
  uint8_t response[8];
  size_t n = receiveFrame(_serial, response, sizeof(response), 8, RESPONSE_TIMEOUT_MS);
  if (n != 8) {
    return false; // timeout, short reply, or a 5-byte exception frame
  }

  uint16_t rcrc = calculateCRC16(response, 6);
  if (response[6] != (rcrc & 0xFF) || response[7] != (rcrc >> 8)) {
    return false; // corrupted reply
  }
  if (response[0] != _device_addr || response[1] != 0x06) {
    return false;
  }
  for (int i = 2; i < 6; i++) {
    if (response[i] != buffer[i]) {
      return false; // echo doesn't match what we wrote
    }
  }
  return true;
}

float LiquidLevelDetection::readModbusRegister(uint16_t reg_addr) {
  uint8_t buffer[8];
  buffer[0] = _device_addr;    // Device address
  buffer[1] = 0x03;            // Function code: read holding register
  buffer[2] = reg_addr >> 8;   // Register address high byte
  buffer[3] = reg_addr & 0xFF; // Register address low byte
  buffer[4] = 0x00;            // Number of registers high byte
  buffer[5] = 0x01;            // Number of registers low byte

  uint16_t crc = calculateCRC16(buffer, 6);
  buffer[6] = crc & 0xFF; // CRC low byte
  buffer[7] = crc >> 8;   // CRC high byte

  // Drop any stale bytes so they can't be mistaken for the reply
  while (_serial->available()) {
    _serial->read();
  }

  _serial->write(buffer, 8);
  _serial->flush(); // wait until the last bit has left the UART

  // Expected reply: addr, 0x03, 0x02, data hi, data lo, crc lo, crc hi (7 bytes)
  uint8_t response[7];
  size_t n = receiveFrame(_serial, response, sizeof(response), 7, RESPONSE_TIMEOUT_MS);
  if (n != 7) {
    return 0.0; // timeout, short reply, or a 5-byte exception frame
  }

  uint16_t rcrc = calculateCRC16(response, 5);
  if (response[5] != (rcrc & 0xFF) || response[6] != (rcrc >> 8)) {
    return 0.0; // corrupted reply
  }
  if (response[0] != _device_addr || response[1] != 0x03 || response[2] != 0x02) {
    return 0.0;
  }

  // Extract data
  uint16_t value = (response[3] << 8) | response[4];
  return value;
}

uint16_t LiquidLevelDetection::calculateCRC16(uint8_t *buffer, uint8_t length) {
  uint16_t crc = 0xFFFF;

  for (uint8_t i = 0; i < length; i++) {
    crc ^= buffer[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}