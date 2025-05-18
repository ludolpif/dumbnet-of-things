/*
 *  Copyright (c) 2025 ludolpif <ludolpif@gmail.com>
 *  This file is part of 1keyboard_2computers.
 *
 *  1keyboard_2computers is free software: you can redistribute it and/or modify it under the terms of
 *  the GNU General Public License as published by the Free Software Foundation,
 *  either version 3 of the License, or (at your option) any later version.
 *
 *  1keyboard_2computers is distributed in the hope that it will be useful, but WITHOUT ANY
 *  WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 *  more details.
 *
 *  You should have received a copy of the GNU General Public License along with
 *  1keyboard_2computers. If not, see <https://www.gnu.org/licenses/>
 */
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
