#include "config.h"
#include <PS2KeyAdvanced.h>

void ps2kbd_detect_keyboard(PS2KeyAdvanced ps2kbd) {
  ps2kbd.echo( );              // ping keyboard to see if there
  delay( 6 );
  uint16_t ps2keycode = ps2kbd.read( );
  // BAT is a PS2 protocol keycode to signal that Keyboard self-autotest result is PASSED.
  if( (ps2keycode & 0xFF) == PS2_KEY_ECHO || (ps2keycode & 0xFF) == PS2_KEY_BAT ) {
    Serial.println( "PS2 keyboard OK.." );
  } else {
    if( ( ps2keycode & 0xFF ) == 0 ) {
      Serial.println( "PS2 keyboard Not Found" );
    } else {
      Serial.print( "Invalid Code received: 0x" );
      Serial.println( ps2keycode, HEX );
    }
  }
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
