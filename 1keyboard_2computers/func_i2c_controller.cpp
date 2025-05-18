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

uint8_t i2c_send_word(uint8_t addr, uint16_t data) {
  Wire.beginTransmission(addr);
  Wire.write(data>>8);
  Wire.write(data & 0xFF);
  return Wire.endTransmission();
}

uint8_t i2c_command(uint8_t addr, uint16_t command, uint8_t *err) {
  uint8_t reply, readlen, res;
  int count = 0;
  res = i2c_send_word(addr, command);
  if ( res > 0 ) {
    if (err) *err = res;
    return 0;
  }
  delay(1); // FIXME find a better way to wait remote command processing completion
  Wire.requestFrom(addr, (uint8_t)1);  // request 1 byte from peripheral device #addr
  if (Wire.available()) {     // peripheral may send less than requested
    reply = Wire.read();
    if (err) *err = 0;
    return reply;
  } else {
    if (err) *err = I2C_WARN_TIMEOUT;
    return I2C_WARN_TIMEOUT;
  }
}

uint8_t i2c_send_key(uint16_t ps2keycode) {
  //TODO should check the ps2keycode do not clash with a defined command
  return i2c_send_word(I2C_PERIPHERIAL_ADDR, ps2keycode);
}

uint8_t i2c_query_leds() {
  uint8_t err = 0;
  uint8_t reply = i2c_command(I2C_PERIPHERIAL_ADDR, I2C_QUERY_LEDS, &err);
  return (err==0)?reply:0;
}

uint8_t i2c_release_all() {
  uint8_t err = 0;
  uint8_t reply = i2c_command(I2C_PERIPHERIAL_ADDR, I2C_RELEASE_ALL, &err);
  return (err==0)?reply:0;
}
