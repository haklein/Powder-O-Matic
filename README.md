Powder'O'Matic Readme
=====================

## Overview

This Sketch controls two stepper motors via two TCM2208 stepper drivers. The drivers 
are controlled via dir/step pins and configured via Serial 2 and 3. Serial 1 is being
used to fetch values from a Kern PCB scale. The target value is adjusted with a KY-040
rotary encoder. Dispensing can be started by pressing the push button of the rotary
encoder. A SSD1306 OLED display is used to display the current state, actual and target
value. The Kern scale needs to be configured for command mode ("rE Cr") with 19200 baud. 
The default values are set up for grain.
  
The KY-040 is evaluated by attaching a pin interrupt to the CLK signal (rotaryIsr())
The Stepper A is being turned via UART commands. Stepper B uses the AccelStep library.

Operation can be seen here: https://youtu.be/FhTphJCxM40
