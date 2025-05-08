#include <PS2KeyAdvanced.h>
#include <HID-Project.h>
#include <Wire.h>
#include "config.h"

/* Global variables */
uint8_t serial_pending_command = SERIAL_COMMAND_DUMP;
uint8_t remote_host_leds;
uint8_t local_host_leds;
#ifdef HAS_PS2_KEYBOARD_ATTACHED
PS2KeyAdvanced ps2kbd;
#endif
#if BOARD == 1
bool send_remotely = true;
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
void ps2kbd_detect_keyboard(PS2KeyAdvanced ps2kbd);
void ps2key_trace(uint16_t ps2keycode);
// from func_usb.cpp
void usbkey_trace( enum KeyboardKeycode usbkeycode);
uint8_t usb_query_leds();
int usb_release_all();
enum KeyboardKeycode map_ps2_to_usb_key(uint16_t PS2KeyAdvancedKeyCode);

void setup() {
  Serial.begin(115200);
  delay(2000); //FIXME Attendre que le Serial soit prêt pour le reporting du clavier, pourrait être asynchrone mais while (!Serial) coute plus de 10ms
#ifdef I2C_CONTROLLER
  Wire.begin();
  Wire.setWireTimeout(1000,false); // 1ms timeout
#endif
#ifdef I2C_PERIPHERIAL
  Wire.begin(I2C_PERIPHERIAL_ADDR);
  Wire.onReceive(i2c_on_receive_keycode_or_command);
  Wire.onRequest(i2c_on_request_command_reply);
#endif
#ifdef HAS_PS2_KEYBOARD_ATTACHED
  // Configure the PS2 keyboard host emulator library
  ps2kbd.begin(PS2_DATA_PIN, PS2_IRQ_PIN);
  ps2kbd_detect_keyboard(ps2kbd);
  // set keyboard typematic (auto-repeat pressed keys) to lowest rate as we don't use it
  ps2kbd.typematic(0x1F, 3);
  // and set no repeat on CTRL, ALT, SHIFT, GUI while outputting
  ps2kbd.setNoRepeat(1);
  // By default light up Scroll Lock (indicating non-standard keyboard), and send keys at the other arduino
  ps2kbd.setLock(PS2_LOCK_SCROLL);
#endif
  // Configure the USB keyboard emulator library
  BootKeyboard.begin();
}

#if BOARD == 1
void loop() {
  task_input_ps2kbd();
  // task_input_serial(); FIXME too slow
  task_output_serial();
  delay(1); // Poor's man powersaving
}
#endif /*# BOARD == 1 */

#if BOARD == 2
void loop() {
  // task_input_serial(); FIXME too slow
  task_output_serial();
  task_process_last_keycode_or_command();
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

  // TODO rewrite condition with usbkeycode
  switch ( (ps2keycode & 0xFF) ) {
    case PS2_KEY_NUM:
      BootKeyboard.write(usbkeycode);
      /* TODO get usb lock status and PS2 to always check if is it in sync ?
      if ( ps2kbd.getLock() & PS2_LOCK_NUM ) {
        Serial.println("PS_LOCK_NUM set");
      } else {
        Serial.println("PS_LOCK_NUM unset");
      }
      */
      break;
    case PS2_KEY_CAPS:
      BootKeyboard.write(usbkeycode);
      /* TODO get usb lock status and PS2 to always check if is it in sync ?
      if ( ps2kbd.getLock() & PS2_LOCK_NUM ) {
        Serial.println("PS_LOCK_CAPS set");
      } else {
        Serial.println("PS_LOCK_CAPS unset");
      }
      */
      break;
    case PS2_KEY_PAUSE:
      // PS2 keyboard don't send break code for Pause key when the key is released
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
      if ( ps2kbd.getLock() & PS2_LOCK_SCROLL ) {
        usb_release_all();
        remote_host_leds = i2c_query_leds();
        // "& 0x07" to not fall in a known bug : https://github.com/techpaul/PS2KeyAdvanced/issues/39
        ps2kbd.setLock(remote_host_leds & 0x07 | PS2_LOCK_SCROLL);
        send_remotely = true;
      } else {
        i2c_release_all();
        //TODO is useful and working ?
        BootKeyboard.wakeupHost();
        local_host_leds = usb_query_leds();
        ps2kbd.setLock(local_host_leds & 0x07 & ~PS2_LOCK_SCROLL);
        send_remotely = false;
      }
      continue;
    }

    if ( send_remotely ) {
      /*
      Serial.print("i2c_send_key(0x");
      Serial.print(ps2keycode, HEX);
      Serial.println(");");
      */
      i2c_send_key(ps2keycode);
    } else {
      emulate_usb(ps2keycode);
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
  last_keycode_or_command = ps2keycode_or_command;
}

void i2c_on_request_command_reply() {
  Wire.write(last_command_reply);
}

void task_process_last_keycode_or_command() {
  switch (last_keycode_or_command) {
    case 0:
      /* Nothing to process for now */
      break;
    case I2C_WARN_SHORT_READ:
      Serial.println("I2C short read");
      break;
    case I2C_QUERY_LEDS:
      last_command_reply = usb_query_leds();
      break;
    case I2C_RELEASE_ALL:
      last_command_reply = usb_release_all()?I2C_COMMAND_SUCCESS:I2C_COMMAND_FAILED;
      break;
    default:
      emulate_usb(last_keycode_or_command);
  }
  last_keycode_or_command = 0;
}
#endif /*# BOARD == 2 */
