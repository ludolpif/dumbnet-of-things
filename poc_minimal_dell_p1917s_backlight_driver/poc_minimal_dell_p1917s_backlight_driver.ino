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
 * This program is a proof of concept about driving Dell P1917S backlight from a
 *  Sparkfun Pro Micro 3.3V 8Mhz (ATMEGA2560 based Arduino clone).
 *
 * Wiring from the Sparkfun Pro Micro to Dell 1917S PSU connector
 *  Sparkfun PIN ... PSU connector
 *           GND <=> PIN 1 (red wire *is* GND on this PSU, as pin 2 and 3)
 *            D4  => PIN 7 : enable backlight signal
 *            D5  => PIN 8 : 10 kHz PWM to transmit backlight luminosity
 *  Sparkfun PIN ... USB Module CN603 (original screen's USB and buttons daughter board)
 *            D6  => PIN 1 (= CN504 PIN 1, light ON on HIGH)
 *           GND <=> PIN 6 (= CN504 PIN 6, button's circuit GND)
 */
// Pinout config
#define CONFIG_PIN_BACKLIGHT_ENA       4 // Prefer a non PWM capable PIN
#define CONFIG_PIN_BACKLIGHT_PWM       5 // This PIN use Timer 3 in my case, original design is at 10kHz
#define CONFIG_PIN_POWER_LED           6 // Prefer a basic PWM pin to allow dimming the LED

// For ATMEGA2560 : set D4 (unused) and D13 PWM frequency, 7812.50 Hz
// it affects delay() function (~10 times faster)
#define SET_PWM_FREQ_AROUND_10KHZ (TCCR0B = TCCR0B & B11111000 | B00000010)

// Run-time data
uint8_t backlight_setpoint;
uint8_t backlight_now;

void setup() {
  pinMode(CONFIG_PIN_BACKLIGHT_ENA, OUTPUT);
  //pinMode(CONFIG_PIN_BACKLIGHT_PWM, OUTPUT); // Will be done by first analogWrite() call
  //pinMode(CONFIG_PIN_POWER_LED, OUTPUT);     // Will be done by first analogWrite() call

  // Set enable signal to off during 0.5 second (let screen PSU stabilise)
  digitalWrite(CONFIG_PIN_BACKLIGHT_ENA, LOW);
  delay(500);

  backlight_now = 0;
  backlight_setpoint = 20; // Should be read from somewhere and user-tunable

  // Set PWM pin without emitting data
  analogWrite(CONFIG_PIN_BACKLIGHT_PWM, backlight_now); // analogWrite set pinMode (and PWM)
  SET_PWM_FREQ_AROUND_10KHZ;

  // Assert enable pin to true (backlight will go on with minimal lighning)
  digitalWrite(CONFIG_PIN_BACKLIGHT_ENA, HIGH);
  delay(500);
}

byte led_level = 0;
void loop() {
  // Set backlight luminosity gradually
  if ( backlight_now < backlight_setpoint ) {
    backlight_now++;
    analogWrite(CONFIG_PIN_BACKLIGHT_PWM, backlight_now);
  } else if ( backlight_now > backlight_setpoint ) {
    backlight_now--;
    analogWrite(CONFIG_PIN_BACKLIGHT_PWM, backlight_now);
  }
  // Animate the screen power led if connected via the USB daughter board
  analogWrite(CONFIG_PIN_POWER_LED, led_level);
  led_level = (led_level+1)%80;
  delay(300);
}
