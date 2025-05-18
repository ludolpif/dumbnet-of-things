#pragma once
#include <Arduino.h>
#if F_CPU != 16000000L
#error "This design targets 5V boards only, you may brick or smoke a 3.3V based board"
#endif

/* Configurable constants */
#define BOARD 1
//#define SERIAL_DEBUG
#define PS2_DATA_PIN 4
#define PS2_IRQ_PIN 7
#define I2C_PERIPHERIAL_ADDR 2
#define PIN_NOTIFY 9

#define NOTIFY_NONE HIGH
#define NOTIFY_PENDING LOW

/* Notes:
 * PS2 Keyboard is connected on BOARD 1
 * on Pro Micro digitalPinToInterrupt(PIN) expands to:
 * ((PIN) == 0 ? 2 : ((PIN) == 1 ? 3 : ((PIN) == 2 ? 1 : ((PIN) == 3 ? 0 : ((PIN) == 7 ? 4 : -1)))))
 *
 * Connections between BOARD1 and BOARD2, through 2.2kΩ resistors
 * I²C
 *  PIN 2: SDA
 *  PIN 3: SCL
 * SPI (in case I²C makes me mad, but not used yet)
 *  PIN 14: CIPO
 *  PIN 16: COPI
 *  PIN 15: SLK
 *  PIN 10: CS
 * Extra
 *  GND (near RAW pin)
 *  PIN 9: GPIO (free)
 */

/* Non-configurable constants */
#if BOARD == 1
  #define I2C_CONTROLLER
  #define HAS_PS2_KEYBOARD_ATTACHED
#elif BOARD == 2
  #define I2C_PERIPHERIAL
#else
  #error "BOARD must be 1 or 2"
#endif
#define xstr(s) str(s)
#define str(s) #s
#define BOARD_STR "BOARD " xstr(BOARD) " - Sparkfun Pro Micro ATmega32U4 (5V, 16Mhz)"

/* Constants for I2C messages using reserved/undefined codes of PS2 protocol */
#define I2C_QUERY_LEDS 0xED
#define I2C_RELEASE_ALL 0xEF
#define I2C_WARN_SHORT_READ 0xF0
#define I2C_WARN_TIMEOUT 0xF1
#define I2C_WARN_OVERRUN 0xF2
#define I2C_COMMAND_SUCCESS 0xFA
#define I2C_COMMAND_FAILED 0xFB

/* Constants for Serial port command queuing */
#define SERIAL_COMMAND_NONE 0
#define SERIAL_COMMAND_DUMP 1

/* Constants for PS2 Keyboard status */
#define PS2KBD_UNKNOWN 0
#define PS2KBD_ALIVE 1
#define PS2KBD_NOT_FOUND 2
#define PS2KBD_FAULTY 3
