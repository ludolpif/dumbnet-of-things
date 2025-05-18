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
#include <PS2KeyAdvanced.h>
#include <HID-Project.h>
#include <Wire.h>
#include "config.h"

/* Global variables */
uint8_t serial_pending_command = SERIAL_COMMAND_NONE;
uint8_t leds;
#ifdef HAS_PS2_KEYBOARD_ATTACHED
PS2KeyAdvanced ps2kbd;
uint8_t ps2kbd_status = PS2KBD_UNKNOWN;
#endif
#if BOARD == 1
bool send_remotely;
#endif
#if BOARD == 2
uint16_t last_keycode_or_command = 0;
uint8_t last_command_reply = 0;
#endif

/* Global helpers functions to keep this .ino file "small" */
// from func_i2c_controller.cpp
uint8_t i2c_send_key(uint16_t ps2keycode);
uint8_t i2c_query_leds();
uint8_t i2c_release_all();
// from func_i2c_peripherial.cpp
uint16_t i2c_recv_word(uint8_t addr, uint8_t *readlen);
// from func_ps2.cpp
uint8_t ps2kbd_detect_keyboard(PS2KeyAdvanced ps2kbd);
void ps2key_trace(uint16_t ps2keycode);
// from func_usb.cpp
void usbkey_trace( enum KeyboardKeycode usbkeycode);
uint8_t usb_query_leds();
int usb_release_all();
enum KeyboardKeycode map_ps2_to_usb_key(uint16_t PS2KeyAdvancedKeyCode);

void setup() {
  // Configure the USB keyboard emulator library
  BootKeyboard.begin();
  // USB Serial, no baudrate
  Serial.begin(0);
#ifdef I2C_CONTROLLER
  Wire.begin();
  Wire.setWireTimeout(1000,false); // 1ms timeout
  pinMode(PIN_NOTIFY, INPUT_PULLUP); // Pull Up to let this other BOARD reboot without driving this line (no garbage reading)
#endif
#ifdef I2C_PERIPHERIAL
  Wire.begin(I2C_PERIPHERIAL_ADDR);
  Wire.onReceive(i2c_on_receive_keycode_or_command);
  Wire.onRequest(i2c_on_request_command_reply);
  digitalWrite(PIN_NOTIFY, NOTIFY_NONE);
  pinMode(PIN_NOTIFY, OUTPUT);
#endif
#ifdef HAS_PS2_KEYBOARD_ATTACHED
  // Configure the PS2 keyboard host emulator library
  ps2kbd.begin(PS2_DATA_PIN, PS2_IRQ_PIN);
  ps2kbd_status = ps2kbd_detect_keyboard(ps2kbd);
  // set keyboard typematic (auto-repeat pressed keys) to lowest rate as we don't use it
  ps2kbd.typematic(0x1F, 3);
  // and set no repeat on CTRL, ALT, SHIFT, GUI while outputting
  ps2kbd.setNoRepeat(1);
#endif
#if BOARD == 1
  switch_keyboard(true);
#endif
}

#if BOARD == 1
void loop() {
  task_input_ps2kbd();
  task_input_serial();
  task_output_serial();
  task_update_leds();
  delay(1); // Poor's man powersaving
}
#endif /*# BOARD == 1 */

#if BOARD == 2
void loop() {
  task_input_serial();
  task_output_serial();
  task_process_last_keycode_or_command();
  task_check_leds();
  delay(1); // Poor's man powersaving
}
#endif /*# BOARD == 2 */

void task_input_serial() {
  while ( Serial.available() ) {
    Serial.read(); // Dump banner whatever the provided input
    serial_pending_command = SERIAL_COMMAND_DUMP;
  }
}

void task_output_serial() {
  switch (serial_pending_command) {
    case SERIAL_COMMAND_DUMP:
      Serial.println(__FILE__);
      Serial.println(BOARD_STR);
#ifdef SERIAL_DEBUG
      Serial.println("SERIAL_DEBUG is defined");
#endif
#ifdef HAS_PS2_KEYBOARD_ATTACHED
      Serial.print("ps2kbd_status: ");
      switch (ps2kbd_status) {
        case PS2KBD_UNKNOWN: Serial.println("PS2KBD_UNKNOWN"); break;
        case PS2KBD_ALIVE: Serial.println("PS2KBD_ALIVE"); break;
        case PS2KBD_NOT_FOUND: Serial.println("PS2KBD_NOT_FOUND"); break;
        case PS2KBD_FAULTY: Serial.println("PS2KBD_FAULTY"); break;
      }
#endif
      break;
    default:
      break;
  }
  serial_pending_command = SERIAL_COMMAND_NONE;
}

void emulate_usb(uint16_t ps2keycode) {
  enum KeyboardKeycode usbkeycode;
  usbkeycode = map_ps2_to_usb_key(ps2keycode);

#ifdef SERIAL_DEBUG
  ps2key_trace(ps2keycode);
  usbkey_trace(usbkeycode);
#endif

  if ( !usbkeycode ) return;

  switch ( (ps2keycode & 0xFF) ) {
    case PS2_KEY_NUM:   // Case where the PS2 lib returns only a single event
    case PS2_KEY_CAPS:  // Case where the PS2 lib returns only a single event
    case PS2_KEY_PAUSE: // PS2 keyboard don't send break code for Pause key when the key is released
      BootKeyboard.write(usbkeycode);
      break;
    //TODO PS2 lib dont send make code for Ctrl+Pause (Break) key, only break code
    default:
      // All others keys, press or release the mapped USB keycode
      if (ps2keycode & PS2_BREAK) {
        BootKeyboard.release(usbkeycode);
      } else {
        BootKeyboard.press(usbkeycode);
      }
  }
}

#if BOARD == 1
void switch_keyboard(bool remotely) {
      if ( remotely ) {
        usb_release_all();
        leds = i2c_query_leds();
        // "& 0x07" to not fall in a known bug : https://github.com/techpaul/PS2KeyAdvanced/issues/39
        ps2kbd.setLock(leds & 0x07 | PS2_LOCK_SCROLL);
        send_remotely = true;
      } else {
        i2c_release_all();
        BootKeyboard.wakeupHost(); //TODO useful ?
        leds = usb_query_leds();
        ps2kbd.setLock(leds & 0x07 & ~PS2_LOCK_SCROLL);
        send_remotely = false;
      }
      send_remotely = remotely;
}

void task_input_ps2kbd() {
  uint16_t ps2keycode;
  uint8_t ps2key;
  enum KeyboardKeycode usbkeycode;

  while ( ps2kbd.available() ) {
    ps2keycode = ps2kbd.read();
    if ( !ps2keycode ) continue;
    ps2key = ps2keycode & 0xFF;
    // Keyboard will send ACK when PS2KeyAdvanced set it's LEDs for Num/Caps/Scroll Locks
    if ( ps2key == PS2_KEY_ACK ) continue;
    // If keyboard is reset, it will send boot test result (BAT) on success
    if ( ps2key == PS2_KEY_BAT ) continue;
    // It may send PS2_KEY_ERROR if boot test fail or if an other general error occurs while running
    if ( ps2key == PS2_KEY_ERROR ) {
    Serial.println("Received PS2_KEY_ERROR, reseting Keyboard");
      ps2kbd.resetKey();
      break;
    }
    if ( ps2key == PS2_KEY_RESEND ) {
      Serial.println("Received PS2_KEY_RESEND, may need to investigate !");
      delay(100);
      break;
    }
    if ( ps2key == PS2_KEY_SCROLL) {
      // ScrollLock key pressed, the library have updated PS2 lock state by itself
      switch_keyboard( ps2kbd.getLock() & PS2_LOCK_SCROLL );
      continue;
    }
    if ( send_remotely ) {
      i2c_send_key(ps2keycode);
    } else {
      emulate_usb(ps2keycode);
    }
  }
}

void task_update_leds() {
  uint8_t new_leds = leds;
  if ( send_remotely && digitalRead(PIN_NOTIFY) == NOTIFY_PENDING ) {
      new_leds = i2c_query_leds();
#ifdef SERIAL_DEBUG
      Serial.print("task_update_leds, remotely, NOTIFY_PENDING, new_leds: 0x");
      Serial.println(new_leds, HEX);
#endif
  }
  if ( !send_remotely ) {
    new_leds = usb_query_leds();
  }
  if ( new_leds != leds ) {
    leds = new_leds;
    if ( send_remotely ) {
      // "& 0x07" to not fall in a known bug : https://github.com/techpaul/PS2KeyAdvanced/issues/39
      ps2kbd.setLock(leds & 0x07 | PS2_LOCK_SCROLL);
    } else {
      ps2kbd.setLock(leds & 0x07 & ~PS2_LOCK_SCROLL);
    }
  }
}
#endif /*# BOARD == 1 */

#if BOARD == 2
/* I2C interrupt callback, be minimalist */
void i2c_on_receive_keycode_or_command(int _len) {
  uint8_t readlen = 0;
  uint16_t ps2keycode_or_command;

  if ( _len != 2 ) {
    last_keycode_or_command = I2C_WARN_SHORT_READ;
    return;
  }
  ps2keycode_or_command = i2c_recv_word(I2C_PERIPHERIAL_ADDR, &readlen);
  if ( readlen != 2 ) {
    last_keycode_or_command = I2C_WARN_SHORT_READ;
    return;
  }
  last_keycode_or_command = last_keycode_or_command?I2C_WARN_OVERRUN:ps2keycode_or_command;
}

void i2c_on_request_command_reply() {
  Wire.write(last_command_reply);
}

void task_process_last_keycode_or_command() {
  uint16_t ps2keycode_or_command = last_keycode_or_command;
  last_keycode_or_command = 0;
  switch (ps2keycode_or_command) {
    case 0:
      /* Nothing to process for now */
      break;
    case I2C_WARN_OVERRUN:
      Serial.println("I2C overrun");
      break;
    case I2C_WARN_SHORT_READ:
      Serial.println("I2C short read");
      break;
    case I2C_QUERY_LEDS:
      BootKeyboard.wakeupHost(); //TODO useful ?
      last_command_reply = usb_query_leds();
      digitalWrite(PIN_NOTIFY, NOTIFY_NONE);
#ifdef SERIAL_DEBUG
      Serial.print("I2C query LEDS returns 0x");
      Serial.println(last_command_reply, HEX);
#endif
      break;
    case I2C_RELEASE_ALL:
      last_command_reply = usb_release_all()?I2C_COMMAND_SUCCESS:I2C_COMMAND_FAILED;
#ifdef SERIAL_DEBUG
      Serial.print("I2C release all returns 0x");
      Serial.println(last_command_reply, HEX);
#endif
      break;
    default:
      emulate_usb(ps2keycode_or_command);
  }
}

void task_check_leds() {
    uint8_t new_leds = usb_query_leds();
    if ( new_leds != leds ) {
      leds = new_leds;
#ifdef SERIAL_DEBUG
      Serial.println("digitalWrite(PIN_NOTIFY, NOTIFY_PENDING);");
#endif
      digitalWrite(PIN_NOTIFY, NOTIFY_PENDING);
    }
}

#endif /*# BOARD == 2 */
