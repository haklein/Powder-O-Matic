Powder'O'Matic Step Readme
==========================

## Overview

This Sketch controls two stepper motors via two TCM2208 stepper drivers. The drivers 
are controlled via dir/step pins and configured via Serial 2 and 3. Serial 1 is being
used to fetch values from a the scale. The target value is adjusted with a KY-040
rotary encoder. Dispensing can be started by pressing the push button of the rotary
encoder. A SSD1306 OLED display is used to display the current state, actual and target
value. The Kern scale needs to be configured for command mode ("rE Cr") with 19200 baud. 
The default values are set up for grain.
  
The KY-040 is evaluated by attaching a pin interrupt to the CLK signal (rotaryIsr())
The Stepper A is being turned via UART commands. Stepper B uses the AccelStep library.

Operation can be seen here: https://youtu.be/FhTphJCxM40 (prototype with a KERN PCB100-3)
and here: https://youtu.be/DZrE01YJ6Ys (final version in nice case with A&D FX120i balance)

The Autotrickler hardware (trickler and stepper motor adapter for the Lee Powder Thrower)
can be bought here: https://www.autotrickler.com

While Adam's Autotrickler hardware is the preferred option, you can find a motor adapter 
for the RCBS Powder Trickler #2 in the designs folder. A pulley for the RCBS pipe and a
case for the provided PCB layout can also be seen there. The case is ready-to-use for the
PCB, an SSD1309 1,54" OLED Display, the rotary, two subminiature switches (power and auto
mode), a DC socket and two G36 4-pin sockets for the stepper motors.

A complete BOM can be found here: https://github.com/haklein/Powder-O-Matic/blob/master/BOM.md

As I've been aksed multiple times, these config settings for the FX120i do work fine for me:

 btPr 2, bPs 5, Prt 3, Spd 20, trc 0, cond 0.

## Requirements

### Arduino libraries

- ArduinoMenu library
- TimerOne
- ClickEncoder - https://github.com/0xPIT/encoder
- Adafruit GFX Library
- TMC2208Stepper
- AccelStepper
- Adafruit SSD1306

