/*

     Powder'O'Matic Step

    Copyright (c) 2018 Harald Klein <hari@vt100.at>

    Licensed under GPL Version 3 - https://www.gnu.org/licenses/gpl-3.0.html

    This Sketch controls two stepper motors via two TCM2208 stepper drivers. The drivers
    are controlled via dir/step pins and configured via Serial 2 and 3. Serial 1 is being
    used to fetch values from a Kern PCB scale. The target value is adjusted with a KY-040
    rotary encoder. Dispensing can be started by pressing the push button of the rotary
    encoder. A SSD1306 OLED display is used to display the current state, actual and target
    value. The Kern scale needs to be configured for command mode ("rE Cr") with 19200 baud.
    The default values are set up for grain.

    The KY-040 is evaluated by attaching a pin interrupt to the CLK signal (rotaryIsr())
    The Stepper A is being turned via a timer interrupt (timer 5). Speed is adjusted by the
    microsteps() function of the TMC2208Stepper library.
*/

/*
  // gramm values
  #define MASSFILL_FAST_DELTA 0.05
  #define MASSFILL_SLOW_DELTA 0.02
  #define TRICKLE_DELTA 0.01
*/

// #define BALANCE_TYPE 1 // kern PCB 100-3
// #define BALANCE_TYPE 2 // A&D FX120i
#define BALANCE_TYPE 2

// settings for Kern PCB
#if BALANCE_TYPE == 1
// grain values
#define MASSFILL_FAST_DELTA 1.4  // 1.2
#define MASSFILL_SLOW_DELTA 0.3
#define TRICKLE_DELTA 0.06
#define TRICKLE_DELAY 200

// settings for A&D FX120i
#elif BALANCE_TYPE == 2
#define MASSFILL_FAST_DELTA 0.8  // 1.2
#define MASSFILL_SLOW_DELTA 0.12
#define TRICKLE_DELTA 0.04
#define TRICKLE_DELAY 0

#endif


#define MOTA_SPEED_MAX 210000
#define MOTA_SPEED_MIN 15000

#define AUTOMODE_PIN 35 // set to 0 if no automode switch is connected

#define debug 20

// trickler stepper setup
#define MOTA_DIR 24
#define MOTA_STEP 23
#define MOTA_ENA 22
#define MOTA_MSTEPS 256 // 16
#define MOTA_SERIAL Serial2
#define MOTA_CURRENT_MA 500

// thrower stepper setup
#define MOTB_DIR 27
#define MOTB_STEP 26
#define MOTB_ENA 25
#define MOTB_MSTEPS 2
#define MOTB_SERIAL Serial3
#define MOTB_CURRENT_MA 300

#define THROWER_POSITION 170
#define THROWER_DELAY 300
#define THROWER_ENABLED true

#define BUZZER_PIN 9
#define BUZZER_SUCCESS_FREQ 2000
#define BUZZER_SUCCESS_DURATION 500
#define BUZZER_ERROR_FREQ 4000
#define BUZZER_ERROR_DURATION 300
#define BUZZER_ERROR_DELAY 300
#define BUZZER_ERROR_BEEP_COUNT 5

// baudrates
#define TMC2208_BAUDRATE 460800
#define KERN_PCB_BAUDRATE 19200

#define CONSOLE_BAUDRATE 115200

// display settings
#define OLED_RESET 34 // -1  Set to -1 for 0,96" SSD1306 display. 1.54" SSD1309 needs reset, pin 34 is recommended
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define DISPLAY_I2C_ADDRESS 0x3D // 0x3c 0.96" 

// rotary encoder settings
#define K040_SW 4  // D22, pin 74
#define K040_DT 3 // D24, pin 72
#define K040_CLK 2   //  int pin  alt: // D26, pin 70

#define ROTARY_DELTA_VALUE -0.1  // change sign to adjust for rotary direction
#define ROTARY_DEBOUNCE_MILLIS 5

// config section end



#include <TMC2208Stepper.h>
TMC2208Stepper driverA = TMC2208Stepper(&MOTA_SERIAL);
TMC2208Stepper driverB = TMC2208Stepper(&MOTB_SERIAL);

#include <AccelStepper.h>
// AccelStepper stepperA(AccelStepper::DRIVER, MOTA_STEP, MOTA_DIR);
AccelStepper stepperB(AccelStepper::DRIVER, MOTB_STEP, MOTB_DIR);

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

volatile float targetValue = 5;
bool automaticModeEnabled = AUTOMODE_PIN != 0 ? true : false;

void rotaryISR() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  unsigned long delta = millis() - lastInterruptTime;
  int multi = sq(500 - min(delta, 500)) / 25000 + 1;

  // debounce 5ms
  if (delta > ROTARY_DEBOUNCE_MILLIS) {

    if (digitalRead(K040_DT) == LOW)
    {
      targetValue = targetValue - ROTARY_DELTA_VALUE * multi;
    }
    else {

      targetValue = targetValue + ROTARY_DELTA_VALUE * multi;
    }
    lastInterruptTime = interruptTime;
  }
}

void configMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.println("Configuration Setting");
//display.println("Powder'O'Matic Stp v1");
  display.display();

  while (true);
}
void setup() {
  Serial.begin(CONSOLE_BAUDRATE);
  Serial1.begin(KERN_PCB_BAUDRATE);
  Serial2.begin(TMC2208_BAUDRATE);
  Serial3.begin(TMC2208_BAUDRATE);

  Serial.println("Powder'O'Matic Step startup...");
  if (!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_I2C_ADDRESS)) { // Address 0x3C
    Serial.println("SSD1306 allocation failed");
  }

  if (digitalRead(K040_SW) == LOW) {
    Serial.println("Setup mode");
    configMenu();
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

  pinMode(K040_DT, INPUT_PULLUP);
  pinMode(K040_SW, INPUT_PULLUP);
  pinMode(K040_CLK, INPUT_PULLUP);

  if (AUTOMODE_PIN != 0) pinMode(AUTOMODE_PIN, INPUT_PULLUP);

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

  if (BUZZER_PIN)
    pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(MOTA_ENA, HIGH);   // Disable driver in hardware
  digitalWrite(MOTB_ENA, HIGH);

  driverA.pdn_disable(true);     // Use PDN/UART pin for communication
  driverA.I_scale_analog(true); // Use internal voltage reference
  driverA.rms_current(MOTA_CURRENT_MA);      // Set driver current 500mA
  // driverB.en_spreadCycle(1);
  driverA.toff(2);               // Enable driver in software
  driverA.mstep_reg_select(true); // ignore MS1 + MS2
  driverA.microsteps(MOTA_MSTEPS);
  driverA.ihold(0);
  digitalWrite(MOTA_DIR, LOW);

  driverB.pdn_disable(true);     // Use PDN/UART pin for communication
  driverB.I_scale_analog(true); // Use internal voltage reference
  driverB.rms_current(MOTB_CURRENT_MA);      // Set driver current 500mA
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

  stepperB.setMaxSpeed(20000000);
  stepperB.setAcceleration(5000);


  // setup end
}

void throwPowder() {
  stepperB.moveTo(THROWER_POSITION * MOTB_MSTEPS);
  stepperB.runToPosition();
  delay(THROWER_DELAY);
  stepperB.moveTo(0 * MOTB_MSTEPS);
  stepperB.runToPosition();

}

// derive motor speed from value
uint32_t speedFromValue(float currentValue, float targetValue) {
  float delta = targetValue - currentValue;
  if (delta > MASSFILL_FAST_DELTA) {
    Serial.println("MAX Speed");
    return MOTA_SPEED_MAX;
  }
  if (delta < MASSFILL_SLOW_DELTA) {
    Serial.println("MIN Speed");
    return MOTA_SPEED_MIN;
  }
  uint32_t newSpeed = map(delta * 10, MASSFILL_SLOW_DELTA * 10, MASSFILL_FAST_DELTA * 10, MOTA_SPEED_MIN, MOTA_SPEED_MAX);
  Serial.print("Speed: ");
  Serial.println(newSpeed);
  return newSpeed;
}

// method to read a value from the serially attached KERN scale
int readScale(float *returnValue) {
  if (debug > 39) Serial.println("read scale");
  if (BALANCE_TYPE ==  1) {
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
  } else { // A&D FX120i
    Serial1.write("Q\r\n");
    delay(50);
    if (Serial1.available()) {
      //Serial.println("data available");
      bool negative = false;
      bool stable = false;
      String scaleOutput = Serial1.readStringUntil('\n');
      if (debug > 29) {
        Serial.print("SCALE OUTPUT:");
        Serial.println(scaleOutput);
        Serial.println(scaleOutput.indexOf("."));
      }
      if ((scaleOutput.indexOf(".") == 8) || (scaleOutput.indexOf('.') == 9)) {
        if (scaleOutput.indexOf('ST') != -1) {
          //Serial.println("stable");
          stable = true;
        }
        if (scaleOutput.indexOf('-') != -1) {
          negative = true;

        }
        String valueString = scaleOutput.substring(4, 12);
        if (debug > 39) {
          Serial.println(valueString);

        }
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
  // wrong scale configuration
  return -1;
}

// helper to loop until a value has been read from the scale
int readScaleStableOrUnstable(float * returnValue) {
  int stable = 0;
  int errcount = 0;
  float value;
  stable = readScale(&value);
  while (stable == -1) {
    stable = readScale(&value);
    if (errcount++ > 10) {
      return -1;
    }
  }
  *returnValue = value;
  return stable;
}

// helper to loop until a stable value has been read
int readScaleStable(float * returnValue) {
  int stable = 0;
  int errcount = 0;
  float value;
  while (stable != 1) {
    stable = readScale(&value);
    if (errcount++ > 10) {
      return -1;
    }
  }
  *returnValue = value;
  return stable;
}

// tara the scale
void taraScale() {
  if (BALANCE_TYPE == 1) { // KERN PCB
    while (Serial1.available()) Serial1.read();
    Serial.println("Tara scale");
    Serial1.write("t");
    Serial1.flush();
    delay(2500);
  } else {  // A&D FX120i
    // T - Tare, Z - ReZero,
    Serial1.write("Z\r\n");
    Serial1.flush();
    // delay(500);

  }

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

String stateToString(int myState) {
  switch (myState) {
    case 0: return "BEREIT";
    case 1: return "TARIERE";
    case 2: return "SCHNELL";
    case 3: return "LANGSAM";
    case 4: return "TRICKLE";
    case 5: return "FERTIG";
    case 6: return "FEHLER";
    default: return "STATUS?";
  }
}

state currentState = IDLE;
state lastState = IDLE;
float value = -1.0;

void updateDisplay() {
  //Serial.println("update display start");
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.println("Powder'O'Matic Stp v1");
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
    case 0: status = " BEREIT  "; break;
    case 1: status = " TARIERE "; break;
    case 2: status = " SCHNELL "; break;
    case 3: status = " LANGSAM "; break;
    case 4: status = " TRICKLE "; break;
    case 5: status = " FERTIG  "; break;
    case 6: status = " FEHLER  "; break;
    default: status = " STATUS? ";
  }
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.print(automaticModeEnabled ? "A" : " ");
  display.print(status);
  display.setTextColor(WHITE);
  display.display();
  // Serial.println("update display end");
}
unsigned long startTimestamp;

void loop() {
  // #define TESTMODE 1

#ifdef TESTMODE
  driverA.VACTUAL(speedFromValue(5, 50));
  while (true) {
    Serial.println("throw");
    throwPowder();
    Serial.println("sleep");
    tone(BUZZER_PIN, BUZZER_SUCCESS_FREQ, BUZZER_SUCCESS_DURATION);

    delay(3000);
  }
#endif

  if (currentState != lastState) {
    Serial.print("State change: ");
    /*    String status;
        switch (currentState) {
          case 0: status = " BEREIT  "; break;
          case 1: status = " TARIERE "; break;
          case 2: status = " SCHNELL "; break;
          case 3: status = " LANGSAM "; break;
          case 4: status = " TRICKLE "; break;
          case 5: status = " FERTIG  "; break;
          case 6: status = " FEHLER  "; break;
          default: status = " STATUS? ";
        } */
    Serial.println(stateToString(currentState));
    lastState = currentState;
  }
  switch (currentState) {
    case IDLE:
      //Serial.println("STATE:IDLE");
      readScaleStableOrUnstable(&value);
      if (AUTOMODE_PIN != 0)
        automaticModeEnabled = (digitalRead(AUTOMODE_PIN) == LOW) ? true : false;
      if ((digitalRead(K040_SW) == LOW) || (automaticModeEnabled && (value < 0.1 && value > -0.1)) ) {
        currentState = WAIT_FOR_TARE;
        updateDisplay();
        taraScale();
      }

      break;
    case WAIT_FOR_TARE:
      //Serial.println("STATE:WAIT_FOR_TARE");
      while (readScaleStable(&value) != 1)
        updateDisplay();
      if (value == 0.0) {
        startTimestamp = millis();
        currentState = MASSFILL_FAST;
        updateDisplay();
        driverA.VACTUAL(speedFromValue(value, targetValue));
        if (THROWER_ENABLED) throwPowder();
      }
      break;
    case MASSFILL_FAST:
      //Serial.println("STATE:MASSFILL_FAST");
      if (readScaleStableOrUnstable(&value) != -1) {
        /*
                Serial.print("Value: ");
                Serial.println(value);
        */
        if (value > targetValue - MASSFILL_FAST_DELTA) {
          currentState = MASSFILL_SLOW;
          driverA.VACTUAL(speedFromValue(value, targetValue));
        }
      }
      break;
    case MASSFILL_SLOW:
      //Serial.println("STATE:MASSFILL_SLOW");
      if (readScaleStableOrUnstable(&value) != -1) {
        driverA.VACTUAL(speedFromValue(value, targetValue));
        if (value > targetValue - MASSFILL_SLOW_DELTA) {
          currentState = TRICKLE;
          if (BALANCE_TYPE == 1) // give the kern pcb some time to get stable
            driverA.VACTUAL((uint32_t) 0);
        }
      }
      break;
    case TRICKLE:
      //Serial.println("STATE:TRICKLE");
      if (BALANCE_TYPE == 1) {
        if (readScaleStableOrUnstable(&value) == 1) {
          if (value > targetValue - TRICKLE_DELTA) {
            currentState = FINISHED;
            driverA.VACTUAL((uint32_t) 0);
          } else {
            Serial.println("trickling..");
            driverA.VACTUAL(speedFromValue(value, targetValue));
          }
        } else {
          Serial.println("no stable");
          driverA.VACTUAL((uint32_t) 0);
          delay(TRICKLE_DELAY);
        }
      } else {
        // A&D FX120i
        readScaleStableOrUnstable(&value);
        if (value > targetValue - TRICKLE_DELTA) {
          currentState = FINISHED;
          driverA.VACTUAL((uint32_t) 0);

        }

      }
      break;
    case FINISHED:
      //Serial.println("STATE:FINISHED");
      driverA.VACTUAL((uint32_t) 0);
      Serial.print("Total time: ");
      Serial.println((millis() - startTimestamp) / 1000);
      delay(500);;
      while (readScaleStableOrUnstable(&value) != 1);
      if (value < targetValue + TRICKLE_DELTA) {
        // success
#ifdef BUZZER_PIN
        tone(BUZZER_PIN, BUZZER_SUCCESS_FREQ, BUZZER_SUCCESS_DURATION);
#endif
        currentState = IDLE;
      } else {
        currentState = ERROR;
        updateDisplay();

      }
      break;
    case ERROR:
      //Serial.println("STATE:ERROR");
      driverA.VACTUAL((uint32_t) 0);
      for (int i = 0; i < BUZZER_ERROR_BEEP_COUNT; i++) {
#ifdef BUZZER_PIN
        tone(BUZZER_PIN, BUZZER_ERROR_FREQ, BUZZER_ERROR_DURATION);
#endif
        delay(BUZZER_ERROR_DELAY);
      }
      currentState = IDLE;
      break;
    default:
      break;
  }

  updateDisplay();
}
