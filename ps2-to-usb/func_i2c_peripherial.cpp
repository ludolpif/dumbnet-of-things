#include "config.h"
#include <Wire.h>

uint16_t i2c_recv_word(uint8_t addr, uint8_t *readlen) {
  uint8_t reqlen = 2, _readlen = 0;
  uint16_t data = 0;
  while (_readlen < reqlen && Wire.available()) {
    uint8_t r = Wire.read(); // receive a byte
    data = data << 8 | r;
    _readlen++;
  }
  if (readlen) *readlen = _readlen;
  return data;
}
