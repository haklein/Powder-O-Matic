/*  
 *   
 *   Powder'O'Matic Step
 *
 *  Copyright (c) 2018 Harald Klein <hari@vt100.at>
 *
 *  Licensed under GPL Version 3 - https://www.gnu.org/licenses/gpl-3.0.html
 *
 *  This Sketch controls two stepper motors via two TCM2208 stepper drivers. The drivers 
 *  are controlled via dir/step pins and configured via Serial 2 and 3. Serial 1 is being
 *  used to fetch values from a Kern PCB scale. The target value is adjusted with a KY-040
 *  rotary encoder. Dispensing can be started by pressing the push button of the rotary
 *  encoder. A SSD1306 OLED display is used to display the current state, actual and target
 *  value. The Kern scale needs to be configured for command mode ("rE Cr") with 19200 baud. 
 *  The default values are set up for grain.
 *  
 *  The KY-040 is evaluated by attaching a pin interrupt to the CLK signal (rotaryIsr())
 *  The Stepper A is being turned via a timer interrupt (timer 5). Speed is adjusted by the
 *  microsteps() function of the TMC2208Stepper library.
*/

/*
  // gramm values
  #define MASSFILL_FAST_DELTA 0.05
  #define MASSFILL_SLOW_DELTA 0.02
  #define TRICKLE_DELTA 0.01
*/

// grain values
#define MASSFILL_FAST_DELTA 1.2
#define MASSFILL_SLOW_DELTA 0.3
#define TRICKLE_DELTA 0.07

#define debug 20

// trickler
#define MOTA_DIR 8
#define MOTA_STEP 11
#define MOTA_ENA 13
#define MOTA_MSTEPS_FAST 16
#define MOTA_MSTEPS_SLOW 64
#define MOTA_MSTEPS_TRICKLE 256

// thrower
#define MOTB_DIR 5
#define MOTB_STEP 3
#define MOTB_ENA 6
#define MOTB_MSTEPS 32

#define K040_SW 22  // D22, pin 74
#define K040_DT 24 // D24, pin 72
#define K040_CLK 2   //  int pin  alt: // D26, pin 70

#include <TMC2208Stepper.h>                       
TMC2208Stepper driverA = TMC2208Stepper(&Serial2);
TMC2208Stepper driverB = TMC2208Stepper(&Serial3);

#include <AccelStepper.h>
AccelStepper stepperA(AccelStepper::DRIVER, MOTA_STEP, MOTA_DIR);
AccelStepper stepperB(AccelStepper::DRIVER, MOTB_STEP, MOTB_DIR);

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET -1
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

volatile bool turn = false;
volatile float targetValue = 4;
volatile int virtualPosition = 0;

void rotaryISR() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();

  // debounce 5ms
  if (interruptTime - lastInterruptTime > 5) {
    if (digitalRead(K040_DT) == LOW)
    {
      targetValue = targetValue - 0.1;
    }
    else {
      targetValue = targetValue + 0.1;
    }
    lastInterruptTime = interruptTime;
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(19200);
  Serial2.begin(115200);
  Serial3.begin(115200);

  Serial.println("Powder'O'Matic Step startup...");
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println("SSD1306 allocation failed");
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.println("Powder'O'Matic Stp v1");
  display.setCursor(0, 14);
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.println("Start..");

  display.display();

  pinMode(K040_DT, INPUT);
  pinMode(K040_SW, INPUT_PULLUP);
  pinMode(K040_CLK, INPUT);

  attachInterrupt(digitalPinToInterrupt(K040_CLK), rotaryISR, LOW);

  driverA.push();
  driverB.push();                // Reset registers

  // Prepare pins
  pinMode(MOTA_ENA, OUTPUT);
  pinMode(MOTB_ENA, OUTPUT);

  pinMode(MOTA_STEP, OUTPUT);
  pinMode(MOTA_DIR, OUTPUT);

  pinMode(MOTB_STEP, OUTPUT);
  pinMode(MOTB_DIR, OUTPUT);

  digitalWrite(MOTA_ENA, HIGH);   // Disable driver in hardware
  digitalWrite(MOTB_ENA, HIGH);

  driverA.pdn_disable(true);     // Use PDN/UART pin for communication
  driverA.I_scale_analog(true); // Use internal voltage reference
  driverA.rms_current(300);      // Set driver current 500mA
  // driverB.en_spreadCycle(1);
  driverA.toff(2);               // Enable driver in software
  driverA.mstep_reg_select(true); // ignore MS1 + MS2
  driverA.microsteps(MOTA_MSTEPS_FAST);
  driverA.ihold(0);
  digitalWrite(MOTA_DIR, LOW);

  driverB.pdn_disable(true);     // Use PDN/UART pin for communication
  driverB.I_scale_analog(true); // Use internal voltage reference
  driverB.rms_current(300);      // Set driver current 500mA
  // driverB.en_spreadCycle(1);
  driverB.toff(2);               // Enable driver in software
  driverB.mstep_reg_select(true); // ignore MS1 + MS2
  driverB.microsteps(MOTB_MSTEPS);
  driverB.ihold(0);


  digitalWrite(MOTA_ENA, LOW);
  digitalWrite(MOTB_ENA, LOW);    // Enable driver in hardware

  if (debug > 9) {
    uint32_t data = 0;
    Serial.print("Mot A DRV_STATUS = 0x");
    driverA.DRV_STATUS(&data);
    Serial.println(data, HEX);
    data = 0;
    Serial.print("Mot B DRV_STATUS = 0x");
    driverB.DRV_STATUS(&data);
    Serial.println(data, HEX);
  }

  stepperA.setMaxSpeed(20000000);
  stepperA.setAcceleration(1000);
  //  stepperA.moveTo(5000);
  //   stepperA.runToPosition();

  stepperB.setMaxSpeed(20000000);
  stepperB.setAcceleration(200000);
  stepperB.moveTo(200 * MOTB_MSTEPS);
  // stepperB.runToPosition();

  //setup Timer5 for stepper A steps
  TCCR5A = 0b00000000; // no pwm
  /*
      CS12  CS11  CS10
         0     0     0    - Stop timer/counter
         0     0     1    - No Prescaler
         0     1     0    - Divide clock by 8
         0     1     1    - by 64
         1     0     0    - by 256
         1     0     1    - by 1024
         1     1     0    - inc falling
         1     1     1    - inc rising
  */
  TCCR5B = 0b00001101;     // clear timer counter on compare, CS12-CS10 == 1024 divisor
  TIMSK5 |= 0b00000010;    // set for output compare interrupt
  sei();                   // enable interrups

  turn = false;

}

// timer interrupt to toggle direction output for stepper A
ISR(TIMER5_COMPA_vect) {
  if (turn) {
    digitalWrite(MOTA_STEP,  !digitalRead(MOTA_STEP));
  }
}

// method to read a value from the serially attached KERN scale
int readScale(float *returnValue) {
  if (debug > 39) Serial.println("read scale");
  Serial1.write("w");
  Serial1.flush();
  delay(300);
  if (Serial1.available()) {
    // while (!(Serial1.available() > 0)) {};
    // if (1) {
    bool negative = false;
    bool stable = false;
    String scaleOutput = Serial1.readStringUntil('\n');
    if (debug > 19) Serial.println(scaleOutput);

    if ((scaleOutput.indexOf('.') == 8) || scaleOutput.indexOf('.') == 9) { // comma separator at pos 9 indicates good string [ for gn, 8 for g]
      if (scaleOutput.indexOf('-') != -1) {
        negative = true;
      }
      if (scaleOutput.indexOf("g") != -1) {
        stable = true;
      }
      String valueString = scaleOutput.substring(2, 12);
      valueString.trim();
      //Serial.print("Valuestring: ");
      //Serial.println(valueString);
      float value = valueString.toFloat();
      if (negative) value = value * -1;
      if (debug > 29) Serial.print("Float: ");
      if (debug > 29) Serial.println(value, 3);
      *returnValue = value;
      if (stable) return 1;
      return 0;
    } else {
      return -1;
    }
  } else {
    if (debug > 39) Serial.println("no data from scale");
    return -1;
  }
}

// helper to loop until a value has been read from the scale
int readScaleStableOrUnstable(float *returnValue) {
  int stable = 0;
  int errcount = 0;
  float value;
  stable = readScale(&value);
  while (stable == -1) {
    stable = readScale(&value);
    if (errcount++ > 10) {
      turn = false;
      return -1;
    }
  }
  *returnValue = value;
  return stable;
}

// helper to loop until a stable value has been read
int readScaleStable(float *returnValue) {
  int stable = 0;
  int errcount = 0;
  float value;
  while (stable != 1) {
    stable = readScale(&value);
    if (errcount++ > 10) {
      turn = false;
      return -1;
    }
  }
  *returnValue = value;
  return stable;
}

// tara the scale
void taraScale() {
  while (Serial1.available()) Serial1.read();
  Serial.println("Tara scale");
  Serial1.write("t");
  Serial1.flush();
  delay(2500);
}

enum state {
  IDLE,
  WAIT_FOR_TARE,
  MASSFILL_FAST,
  MASSFILL_SLOW,
  TRICKLE,
  FINISHED,
  ERROR
};

state currentState = IDLE;
float value = -1.0;

void updateDisplay() {
  //Serial.println("update display start");
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.println("Powder'O'Matic v0.12");
  display.setCursor(0, 11);
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.print("I:");
  for (int i = 0; i < (8 - String(value).length()); i++) display.print(" ");
  display.print(String(value));
  display.print("S:");
  for (int i = 0; i < (8 - String(targetValue).length()); i++) display.print(" ");
  display.print(String(targetValue));
  String status;
  switch (currentState) {
    case 0: status = "  BEREIT  "; break;
    case 1: status = "  TARIERE "; break;
    case 2: status = "  SCHNELL "; break;
    case 3: status = "  LANGSAM "; break;
    case 4: status = "  TRICKLE "; break;
    case 5: status = "  FERTIG  "; break;
    case 6: status = "  FEHLER  "; break;
    default: status = "  STATUS? ";
  }
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.print(status);
  display.setTextColor(WHITE);
  display.display();
  // Serial.println("update display end");
}

void loop() {
  switch (currentState) {
    case IDLE:
      Serial.println("STATE:IDLE");

      if (digitalRead(K040_SW) == LOW) {
        currentState = WAIT_FOR_TARE;
        taraScale();
      }

      break;
    case WAIT_FOR_TARE:
      Serial.println("STATE:WAIT_FOR_TARE");
      while (readScaleStable(&value) != 1);
      if (value == 0.0) {
        currentState = MASSFILL_FAST;
        driverA.microsteps(MOTA_MSTEPS_FAST);
        turn = true;
      }
      break;
    case MASSFILL_FAST:
      Serial.println("STATE:MASSFILL_FAST");
      if (readScaleStableOrUnstable(&value) != -1) {

        Serial.print("Value: ");
        Serial.println(value);

        if (value > targetValue - MASSFILL_FAST_DELTA) {
          currentState = MASSFILL_SLOW;
          driverA.microsteps(MOTA_MSTEPS_SLOW);
        }
      }
      break;
    case MASSFILL_SLOW:
      Serial.println("STATE:MASSFILL_SLOW");
      if (readScaleStableOrUnstable(&value) != -1) {

        if (value > targetValue - MASSFILL_SLOW_DELTA) {
          currentState = TRICKLE;
          turn = false;
          driverA.microsteps(MOTA_MSTEPS_TRICKLE);
        }
      }
      break;
    case TRICKLE:
      Serial.println("STATE:TRICKLE");
      if (readScaleStableOrUnstable(&value) == 1) {
        if (value > targetValue - TRICKLE_DELTA) {
          turn = false;
          currentState = FINISHED;
        } else {
          Serial.println("trickling..");
          turn = true;
        }
      } else {
        Serial.println("no stable");
        turn = false;
        delay(100);
      }
      break;
    case FINISHED:
      Serial.println("STATE:FINISHED");
      if (digitalRead(K040_SW) == LOW) {
        currentState = IDLE;
      } else if (readScaleStableOrUnstable(&value) == 1) {
        if (value < 1 && value > -1)
          currentState = IDLE;
      }
      break;
    case ERROR:
      Serial.println("STATE:ERROR");
      turn = false;
      break;
    default:
      break;
  }

  updateDisplay();
}
