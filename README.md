# My Dumbnet Of Things

Here I share my arduino-like programs for automation I use for streaming.

## `1keyboard_2computers`

This program transform an old IBM SK8820 PS2 Keyboard into a dual USB keyboard to control two computers.
I plan to use it in a streaming setup (a streaming laptop and a dekstop computer to show things on stream).

I've put 2 Sparkfun Pro Micro inside the original keyboard case, wiring is detailled in `config.h`.

## `backlight_midi`

This program transform an old Dell P1917S screens into cheap USBMIDI remote controllable light panels.

You need to disassemble the screen, remove it's motherboard and LCD screen, keep USB dauther board.

Add a 3.3V clone of Arduino, with native USB capabilities (atmega32u4 based boards or ARM boards)
and with PWM capable to get around 10 kHz. I use Sparkfun Pro Micro 3.3V 8Mhz.

This proram outputs "enable" and PWM signals needed by screen's power supply board for backlight
from a value from USB MIDI input ("Control Change" message) or screen buttons (via original USB board)
