/*
 * backlight_midi, Transform old Dell P1917S screens into cheap USBMIDI remote controllable light panels.
 * Copyright (C) 2023-2025  Ludolpif <ludolpif@gmail.com>
 *
 * This file is part of backlight_midi.
 *
 *  backlight_midi is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  backlight_midi is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with backlight_midi.  If not, see <http://www.gnu.org/licenses/>
 *
 * This program transform old Dell P1917S screens into cheap USBMIDI remote controllable light panels.
 *   You need to disassemble the screen, remove it's motherboard and LCD screen, keep USB dauther board.
 *   Add a 3.3V clone of Arduino, with native USB capabilities (atmega32u4 based boards or ARM boards)
 *   and with PWM capable to get around 10 kHz. I use Sparkfun Pro Micro 3.3V 8Mhz.
 *  This proram outputs "enable" and PWM signals needed by screen's power supply board for backlight.
 *  from a value from USB MIDI input ("Control Change" message) or screen buttons (via original USB board)
 *
 * Soldering USB Module CN603 PIN 16 => 10 kOhm Resistor => USB Module CN603 PIN 10
 *   (to keep USB module out of sleep mode and power up our micro-controller)
 *
 * Connections to make from this module to original screen's modules
 *  (arrow indicates which module give power or signal to which other module)
 *  Screen USB A Port from it's hub           => Our board, USB Port (for power and data)
 *  Screen USB B Port from it's hub           <= A computer (for flashing program, and USBMIDI data routing)
 *  Dupont M/F POWER Supply PIN 1 (red GND !) => USB Module CN603 PIN 11 (GND, 7-8-11-12-13 tied together)
 *  Dupont M/F POWER Supply PIN 2 (black GND) => USB Module CN603 PIN 12
 *  Dupont M/F POWER Supply PIN 3 (black GND) => Our board, PIN GND
 *  No wire    POWER Supply PIN 4 (+5V VCC)   => unconnected, Dupont are thicker than target connector spacing
 *  Dupont M/F POWER Supply PIN 5 (+5V VCC)   => USB Module CN603 PIN 15 (VCC, 14-15-16 tied together)
 *  Dupont M/F POWER Supply PIN 6 (+5V VCC)   => USB Module CN603 PIN 16
 *  Dupont M/F CONFIG_PIN_BACKLIGHT_ENA       => POWER Supply PIN 7
 *  Dupont M/F CONFIG_PIN_BACKLIGHT_PWM       => POWER Supply PIN 8
 *  Dupont F/F CONFIG_PIN_POWER_LED           => USB Module CN603 PIN 1 (= CN504 PIN 1, light ON on HIGH)
 *  Dupont F/F CONFIG_PIN_SCREEN_BTN_5_POWER  <= USB Module CN603 PIN 2 (= CN504 PIN 2)
 *  Dupont F/F CONFIG_PIN_SCREEN_BTN_3_4      <= USB Module CN603 PIN 3 (= CN504 PIN 3)
 *  Dupont F/F CONFIG_PIN_SCREEN_BTN_2        <= USB Module CN603 PIN 4 (= CN504 PIN 4)
 *  Dupont F/F CONFIG_PIN_SCREEN_BTN_1        <= USB Module CN603 PIN 5 (= CN504 PIN 5)
 *  Dupont F/F Our board, PIN GND             => USB Module CN603 PIN 6 (= CN504 PIN 6, button's circuit GND)
 *
 * Bugfix:
 *  On thoses Dell screens, around 0.15W need to be drawn from 5V rail if enabling the backlight power rail
 *   If not, the power supply cannot be stable by design, and cycle off an on (regulator goes out of range ?)
 *  Dupont M/F USB Module CN603 PIN 13 (GND) => Some 0.15W load (LED or resistor)
 *  Dupont M/F USB Module CN603 PIN 14 (+5V) => Some 0.15W load (LED or resistor)
 *  My calculus for the resistor. Seeking R in Ohms for U=5.0 Volts and P=0.15 Watts
 *   P=U*I², I²=P/U, I=sqrt(P/U)
 *   U=R*I, R=U/I, R=U/sqrt(P/U)
 *   R=5.0/sqrt(0.15/5.0)
 *   R=29 Ohms (take a not-so-tiny one, for thermal dissipation)
 */
#include <EEPROM.h>
#include "MIDIUSB.h" // Need a native USB System-On-Chip ( AVR ATMEGA16U4 CPU based for example )

// Begin of compile-time config section

// Default pinout is suitable for Sparkfun Pro Micro 3.3V
#define CONFIG_PIN_LOOP_WATCHDOG       2 // Prefer a non PWM capable PIN
#define CONFIG_PIN_BACKLIGHT_ENA       4 // Prefer a non PWM capable PIN
#define CONFIG_PIN_BACKLIGHT_PWM       5 // This PIN use Timer 3 in my case, original design is at 10kHz
#define CONFIG_PIN_POWER_LED           6 // Prefer a basic PWM pin to allow dimming the LED
#define CONFIG_PIN_SCREEN_BTN_1       14 // Prefer a non PWM capable PIN
#define CONFIG_PIN_SCREEN_BTN_2       15 // Prefer a non PWM capable PIN
#define CONFIG_PIN_SCREEN_BTN_3_4     A0 // Original design : 2 buttons on 1 signal trace
#define CONFIG_PIN_SCREEN_BTN_5_POWER 16 // Prefer a non PWM capable PIN

#define CONFIG_SCREEN_BTN_COUNT 5
#define CONFIG_SCREEN_POWER_LED_SOLID_LEVEL 64

// Original PWM frequency for backlight control on Dell P1917S screen is 10kHz (exactly)
// Default PWM frequency with Arduino-like board are between 0.24 and 0.98 kHz
#define CONFIG_PIN_BACKLIGHT_PWM_FREQ_TUNE ( TCCR3B = TCCR3B & B11111000 | B001 )
/* With this tune on Sparkfun Pro Micro 3.3V 8Mhz, I get 15.7 kHz
   This setting set Timer 3 prescaler to /1 instead of the default /64
   It's a bit specific to your chip, board, pinout. If using a ATMEGA16U4 ou ATMEGA32U4 based board,
    see atmel-7766-8-bit-avr-atmega16u4-32u4_datasheet.pdf, 14.10.4 Timer/Counter3 Control Register B – TCCR3B
   Note : changing TCCR0B usually affect delay() function, try to find a PWM PIN *not* using Timer 0
*/
// Configurable code macro, to keep loop() run fast as useful but power efficient (here 8ms, so below 125Hz)
#define CONFIG_LOOP_END_DELAY delay(8)
// Configurable code macro to wait USB Serial, but in a non-infinite loop for no-USB plugged boots
#define CONFIG_SERIAL_BEGIN_DELAY delay(2000)
#define CONFIG_LEARNING_MAX_LOOP_COUNT 1000

// End of compile-time config section

// Global variables, notably the runtime configuration
struct config_struct {
  char magic[25]="backlight_midi_config_v1";
  // If EEPROM is lost or invalid, default to ~10% of backlight luminosity
  int backlight_setpoint = 26;
  bool backlight_enable = true;
  // Channel 0, control 0x15 is the first rotating button on Novation LaunchKey Mini USB MIDI controller
  byte midi_channel = 0x00;
  byte midi_control = 0x15;
};
struct config_struct config_live, config_eeprom, config_defaults;
unsigned int loop_count = 0, loop_last_mode_transition = 0;
#define MODE_LISTENING 0
#define MODE_LEARNING 1
//TODO refactor to use a MODE_SLEEPING but write it in config_live to keep it on power loss/restore
byte running_mode = MODE_LISTENING;

// Function declarations to keep setup() and loop() definitions first
void load_EEPROM(bool force_defaults);
void task_input_MIDI();
void task_input_buttons();
void task_output_backlight();
void task_output_LEDs();
void task_update_EEPROM();

void setup() {
  Serial.begin(115200);
  CONFIG_SERIAL_BEGIN_DELAY;
  Serial.println("setup()");

  pinMode(CONFIG_PIN_LOOP_WATCHDOG, OUTPUT);
  pinMode(CONFIG_PIN_BACKLIGHT_ENA, OUTPUT);
  //pinMode(CONFIG_PIN_BACKLIGHT_PWM, OUTPUT); // Will be done by first analogWrite() call
  //pinMode(CONFIG_PIN_POWER_LED, OUTPUT);     // Will be done by first analogWrite() call
  pinMode(CONFIG_PIN_SCREEN_BTN_1, INPUT_PULLUP);
  pinMode(CONFIG_PIN_SCREEN_BTN_2, INPUT_PULLUP);
  pinMode(CONFIG_PIN_SCREEN_BTN_3_4, INPUT_PULLUP);
  pinMode(CONFIG_PIN_SCREEN_BTN_5_POWER, INPUT_PULLUP);

  analogWrite(CONFIG_PIN_POWER_LED, 0);
  analogWrite(CONFIG_PIN_BACKLIGHT_PWM, 0);
  // Serial.print("TCCR3B == 0x"); Serial.println(TCCR3B, HEX);
  CONFIG_PIN_BACKLIGHT_PWM_FREQ_TUNE;
  // Serial.print("TCCR3B == 0x"); Serial.println(TCCR3B, HEX);

  // You can keep power button down while booting to force write default config to eeprom
  bool force_defaults = (digitalRead(CONFIG_PIN_SCREEN_BTN_5_POWER) == LOW);
  load_EEPROM(force_defaults);
}

void set_running_mode(byte mode) {
  running_mode = mode;
  loop_last_mode_transition = loop_count;
  Serial.print("running_mode = ");
  switch (mode) {
    case MODE_LISTENING:  Serial.println("MODE_LISTENING"); break;
    case MODE_LEARNING:   Serial.println("MODE_LEARNING"); break;
    default:              Serial.println("MODE_UNKNOWN"); break;
  }
}

void loop() {
  // Update running_mode state machine
  if ( running_mode == MODE_LEARNING && (loop_count - loop_last_mode_transition) > CONFIG_LEARNING_MAX_LOOP_COUNT ) {
    set_running_mode(MODE_LISTENING);
  }
  // Process inputs (and set some config_live attributes)
  task_input_MIDI();
  task_input_buttons();
  // Update output registers (from config_live)
  task_output_backlight();
  task_output_LEDs();
  // Misc stuff
  task_update_EEPROM();
  // Throttle this processing loop
  CONFIG_LOOP_END_DELAY;
  // and trace it on a PIN to check for program health and smoothness with an oscilloscope
  digitalWrite(CONFIG_PIN_LOOP_WATCHDOG, loop_count % 2);
  loop_count++;
}


// This program will filter out all incoming USB MIDI messages
//  except "Control Change" for config_live.midi_channel / config_live.midi_control
// If a MIDI message match, then set config_live.backlight_setpoint with it's values
void task_input_MIDI() {
  midiEventPacket_t rx;
  // Process all pending MIDI events
  do {
    // Pop a pending event from queue (non blocking call, rx.header == 0 if no messages)
    rx = MidiUSB.read();
    // http://www.music.mcgill.ca/~ich/classes/mumt306/StandardMIDIfileformat.html#BMA1_
    if (rx.header == 0x0B /* == control change */) {
      byte channel = rx.byte1 & 0x0F  /* channel number 0-15 is the 4 lower bits */;
      byte control = rx.byte2;        /* controller number 0-127 */
      byte midi_value = rx.byte3;     /* controller new value 0-127 */
      /*
      Serial.print("Received MIDI Control Change, channel: "); Serial.print(channel);
      Serial.print(", control: "); Serial.print(control);
      Serial.print(", midi_value: "); Serial.println(midi_value);
      */
      switch (running_mode) {
        case MODE_LISTENING:
          // Considering only MIDI Control Change messages that match our current channel/control filter
          if ( channel == config_live.midi_channel && control == config_live.midi_control ) {
            // Update the desired backlight value (setpoint)
            config_live.backlight_setpoint = midi_value << 1 | 1; // Scale up from 0-127 to 1-255
          }
        break;
        case MODE_LEARNING:
          // Accept any MIDI Control Change message and set our channel/control filter with it's values
          config_live.midi_channel = channel;
          config_live.midi_control = control;
          Serial.print("config_live.midi_channel == 0x"); Serial.println(config_live.midi_channel, HEX);
          Serial.print("config_live.midi_control == 0x"); Serial.println(config_live.midi_control, HEX);
          set_running_mode(MODE_LISTENING);
        break;
      }
    }
  } while (rx.header != 0);
}

void screen_btn_1_handler(int prev_state, int last_state) {
  if ( !config_live.backlight_enable ) return;
  if ( prev_state == HIGH && last_state == LOW ) {
    // key press
  } else if ( prev_state == LOW && last_state == HIGH ) {
    // key release
    set_running_mode(MODE_LEARNING);
  } else {
    // key repeat
  }
}

void screen_btn_2_handler(int prev_state, int last_state) {
  if ( !config_live.backlight_enable ) return;
  if ( prev_state == HIGH && last_state == LOW ) {
    // key press
  } else if ( prev_state == LOW && last_state == HIGH ) {
    // key release
  } else {
    // key repeat
  }
}

void screen_btn_3_handler(int prev_state, int last_state) {
  if ( !config_live.backlight_enable ) return;
  if ( prev_state == HIGH && last_state == LOW ) {
    // key press
  } else if ( prev_state == LOW && last_state == HIGH ) {
    // key release
  } else {
    // key repeat
    if ( config_live.backlight_setpoint > 1 ) config_live.backlight_setpoint--;
  }
}

void screen_btn_4_handler(int prev_state, int last_state) {
  if ( !config_live.backlight_enable ) return;
  if ( prev_state == HIGH && last_state == LOW ) {
    // key press
  } else if ( prev_state == LOW && last_state == HIGH ) {
    // key release
  } else {
    // key repeat
    if ( config_live.backlight_setpoint < 255 ) config_live.backlight_setpoint++;
  }
}

void screen_btn_5_power_handler(int prev_state, int last_state) {
  if ( prev_state == HIGH && last_state == LOW ) {
    // key press
  } else if ( prev_state == LOW && last_state == HIGH ) {
    // key release
    config_live.backlight_enable = !config_live.backlight_enable;
  } else {
    // key repeat
  }
}

void task_input_buttons() {
  // array of function pointers to call event processing procedures
  typedef void(*btn_event_handler_type)(int, int);
  btn_event_handler_type btn_event_handler[CONFIG_SCREEN_BTN_COUNT] = { screen_btn_5_power_handler,
    screen_btn_1_handler, screen_btn_2_handler, screen_btn_3_handler, screen_btn_4_handler};
  // state variables
  static int btn_last_state[CONFIG_SCREEN_BTN_COUNT] = { HIGH, HIGH, HIGH, HIGH, HIGH };
  static int btn_reading[CONFIG_SCREEN_BTN_COUNT];
  // temporary variables
  int i, reading;

  // Read all inputs "at once"
  unsigned long reading_time = millis();
  btn_reading[0] = digitalRead(CONFIG_PIN_SCREEN_BTN_5_POWER);
  btn_reading[1] = digitalRead(CONFIG_PIN_SCREEN_BTN_1);
  btn_reading[2] = digitalRead(CONFIG_PIN_SCREEN_BTN_2);
  // Special case for analog pin for 2 buttons CONFIG_PIN_SCREEN_BTN_3_4
  // Original Dell P1917S design use a 10kOhm resistor for button 3 and 6kOhm resistor for button 4
  reading = analogRead(CONFIG_PIN_SCREEN_BTN_3_4);
  //Serial.print("(analog) reading == "); Serial.println(reading);
  if ( reading < 200 ) {
    btn_reading[3] = HIGH;
    btn_reading[4] = LOW;
  } else if ( reading < 400 ) {
    btn_reading[3] = LOW;
    btn_reading[4] = HIGH;
  } else {
    btn_reading[3] = HIGH;
    btn_reading[4] = HIGH;
  }
  /*
  // All button state for debbugging
  for (i=0; i<CONFIG_SCREEN_BTN_COUNT; i++) {
    Serial.print("btn_reading["); Serial.print(i); Serial.print("] = "); Serial.println(btn_reading[i]);
    Serial.print(", btn_last_state["); Serial.print(i); Serial.print("] = "); Serial.println(btn_last_state[i]);
  }
  */
  // TODO Add some debounce logic for all btn_reading[]

  // Call the correct btn_event_handler function if button state change or button kept pressed
  for (i=0; i<CONFIG_SCREEN_BTN_COUNT; i++) {
    // If the switch changed, due to noise or pressing:
    if (btn_last_state[i] != HIGH || btn_reading[i] != HIGH) {
      if (btn_event_handler[i]) {
        // Call the function pointed by the item i in the array of function pointers, with the correct arguments
        btn_event_handler[i](btn_last_state[i], btn_reading[i]);
      }
      btn_last_state[i] = btn_reading[i];
    }
  }
}

void task_output_backlight() {
  static int backlight_last_set = 0;
  if ( !config_live.backlight_enable ) {
    analogWrite(CONFIG_PIN_BACKLIGHT_PWM, 0);
    digitalWrite(CONFIG_PIN_BACKLIGHT_ENA, LOW);
    backlight_last_set = 0;
    return;
  }
  // Update outputs (from global variables)
  if ( backlight_last_set != config_live.backlight_setpoint ) {
    int val;
    if ( config_live.backlight_setpoint >= backlight_last_set + 4) {
      val = backlight_last_set + 4;
    } else if ( config_live.backlight_setpoint > backlight_last_set) {
      val = backlight_last_set + 1;
    } else {
      val = config_live.backlight_setpoint;
    }
    analogWrite(CONFIG_PIN_BACKLIGHT_PWM, val);
    digitalWrite(CONFIG_PIN_BACKLIGHT_ENA, HIGH);
    backlight_last_set = val;
  }
}

void task_output_LEDs() {
  int val;
  if ( config_live.backlight_enable ) {
    switch (running_mode) {
      case MODE_LISTENING:
        // Dimmed but solid on
        val = CONFIG_SCREEN_POWER_LED_SOLID_LEVEL;
        break;
      case MODE_LEARNING:
        // Blinking
        int blink_loop_counter = (loop_count - loop_last_mode_transition) % 100;
        val = ( blink_loop_counter < 50 )?CONFIG_SCREEN_POWER_LED_SOLID_LEVEL:1;
        break;
    }
  } else {
    // Slow breathing animation
    val = (loop_count%512) >> 3;
    if ( val > 32 ) {
      val = 64 - val;
    }
  }
  analogWrite(CONFIG_PIN_POWER_LED, val);
}

void task_update_EEPROM() {
  static unsigned long config_last_change_time = 0;
  static unsigned long config_last_write_time = 0;
  unsigned long now = millis();

  // Update EEPROM config if needed
  if ( memcmp(&config_eeprom, &config_live, sizeof(config_eeprom)) != 0 ) {
    if ( config_last_change_time == 0 ) {
      Serial.println("Current config changed");
      config_last_change_time = now;
    } else if ( (now - config_last_change_time) > 750 && (now - config_last_write_time) > 10000) {
      Serial.println("Writing current config to EEPROM");
      EEPROM.put(0, config_live);
      EEPROM.get(0, config_eeprom);
      config_last_change_time = 0;
      config_last_write_time = now;
    }
  }
}

void blink_blocking(int pin, int times, int delay_ms, int val) {
  for (int i=0; i<times; i++) {
    analogWrite(pin, val);
    delay(delay_ms);
    analogWrite(pin, 0);
    delay(delay_ms);
  }
  analogWrite(pin, val);
}

void load_EEPROM(bool force_defaults) {
  // Try to restore config from EEPROM to config_eeprom
  EEPROM.get(0, config_eeprom);
  if ( force_defaults || strncmp(config_live.magic, config_eeprom.magic, strlen(config_live.magic)) != 0 ) {
    Serial.print("Writing default config to EEPROM, force_defaults == "); Serial.println(force_defaults);
    EEPROM.put(0, config_defaults);
    blink_blocking(CONFIG_PIN_POWER_LED, 2, 250, CONFIG_SCREEN_POWER_LED_SOLID_LEVEL);
  }
  // Load again in config_live this time
  // TODO unlikely garbaged but should have a CRC somehow
  EEPROM.get(0, config_live);
  Serial.print("config_live.backlight_setpoint == "); Serial.println(config_live.backlight_setpoint);
  Serial.print("config_live.backlight_enable == "); Serial.println(config_live.backlight_enable);
  Serial.print("config_live.midi_channel == 0x"); Serial.println(config_live.midi_channel, HEX);
  Serial.print("config_live.midi_control == 0x"); Serial.println(config_live.midi_control, HEX);
}
