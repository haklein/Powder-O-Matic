Bill of Material
================

- P'o'M PCB (see schematics folder) or breadboard/prototype PCB
- RobotDyn Mega 2560 PRO (Embed) fitting the P'o'M PCB (or any other Mega 2560 based arduino if you do your own PCB)
- TMC2208 Board (I'm using the 1.0 from fysetc.com, make sure it's one where you can solder-bridge the NC pin to the
  serial input of the TMC (as that's used on the P'o'M PCB). You need one board for the trickler and one for the thrower.
- piezzo buzzer (U4) - I'm using a KEPO KPT-G1420A-K8437 
- capacitor (220 µF)
- resistors (R1 + R2) - 1k
- screw terminalls - Watch the spacing of the PCB, I'm using DECA MA212-350M02 
- pin headers and sockets
- RS232 ttl level converter, watch out, some of the ebay PCBs run insanely hot and don't work reliable (mostly SMD ones 
  without LED). The ones with MAX232 in DIP and LEDs worked fine for me
- DC socket
- GX16-4 plug and socket for the steppers
- 3d printed case from the designs folder (perfect fit for the P'o'M PCB)
