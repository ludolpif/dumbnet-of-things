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
#include <PS2KeyAdvanced.h>

uint8_t ps2kbd_detect_keyboard(PS2KeyAdvanced ps2kbd) {
  ps2kbd.echo(); // ping keyboard to see if there
  delay(6);
  uint16_t ps2keycode = ps2kbd.read();

  if( (ps2keycode & 0xFF) == 0 ) {
    // Maybe we are in a cold boot, and keyboard is booting, PS2 spec allow 400ms for that
    delay(400);
    ps2kbd.echo();
    delay(6);
    ps2keycode = ps2kbd.read( );
  }

  // BAT is a PS2 protocol keycode to signal that Keyboard self-autotest result is PASSED.
  if( (ps2keycode & 0xFF) == PS2_KEY_ECHO || (ps2keycode & 0xFF) == PS2_KEY_BAT ) {
    return PS2KBD_ALIVE;
  }
  if( (ps2keycode & 0xFF) == 0 ) {
    return PS2KBD_NOT_FOUND;
  }
  return PS2KBD_FAULTY;
}

/*
About ps2keycode flags (from PS2KeyAdvanced.h):
    Top Byte is 8 bits denoting as follows with defines for bit code

        Define name bit     description
        PS2_BREAK   15      1 = Break key code
                   (MSB)    0 = Make Key code
        PS2_SHIFT   14      1 = Shift key pressed as well (either side)
                            0 = NO shift key
        PS2_CTRL    13      1 = Ctrl key pressed as well (either side)
                            0 = NO Ctrl key
        PS2_CAPS    12      1 = Caps Lock ON
                            0 = Caps lock OFF
        PS2_ALT     11      1 = Left Alt key pressed as well
                            0 = NO Left Alt key
        PS2_ALT_GR  10      1 = Right Alt (Alt GR) key pressed as well
                            0 = NO Right Alt key
        PS2_GUI      9      1 = GUI key pressed as well (either)
                            0 = NO GUI key
        PS2_FUNCTION 8      1 = FUNCTION key non-printable character (plus space, tab, enter)
                            0 = standard character key
  Note defines starting
            PS2_KC_*  are internal defines for codes from the keyboard
            PS2_KEY_* are the codes this library returns
            PS2_*     remaining defines for use in higher levels
 */
#ifdef SERIAL_DEBUG
void ps2key_trace(uint16_t ps2keycode) {
  Serial.print("PS2_KEY 0x");
  if ( (ps2keycode&0xFF) < 0x10 ) Serial.print("0");
  Serial.print( (ps2keycode&0xFF), HEX);

  if ( ps2keycode & PS2_BREAK ) {
    Serial.print(" PS2_BREAK");
  } else {
    Serial.print("          ");
  }
  if ( ps2keycode & PS2_SHIFT ) {
    Serial.print(" PS2_SHIFT");
  } else {
    Serial.print("          ");
  }
  if ( ps2keycode & PS2_CTRL ) {
    Serial.print(" PS2_CTRL");
  } else {
    Serial.print("         ");
  }
  if ( ps2keycode & PS2_CAPS ) {
    Serial.print(" PS2_CAPS");
  } else {
    Serial.print("         ");
  }
  if ( ps2keycode & PS2_ALT ) {
    Serial.print(" PS2_ALT");
  } else {
    Serial.print("        ");
  }
  if ( ps2keycode & PS2_ALT_GR ) {
    Serial.print(" PS2_ALT_GR");
  } else {
    Serial.print("           ");
  }
  if ( ps2keycode & PS2_GUI ) {
    Serial.print(" PS2_GUI");
  } else {
    Serial.print("        ");
  }
  if ( ps2keycode & PS2_FUNCTION ) {
    Serial.print(" PS2_FUNCTION ");
  } else {
    Serial.print("              ");
  }
}
#endif /* SERIAL_DEBUG */
