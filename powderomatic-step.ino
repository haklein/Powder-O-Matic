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

// grain values
#define MASSFILL_FAST_DELTA 1.4  // 1.2
#define MASSFILL_SLOW_DELTA 0.3
#define TRICKLE_DELTA 0.06
#define TRICKLE_DELAY 200

#define MOTA_SPEED_MAX 210000
#define MOTA_SPEED_MIN 15000

#define AUTOMODE false

#define debug 20

// trickler stepper setup
#define MOTA_DIR 8
#define MOTA_STEP 11
#define MOTA_ENA 13
#define MOTA_MSTEPS 256 // 16
//#define MOTA_MSTEPS_SLOW 64
//#define MOTA_MSTEPS_TRICKLE 256
#define MOTA_SERIAL Serial2
#define MOTA_CURRENT_MA 300

// thrower stepper setup
#define MOTB_DIR 5
#define MOTB_STEP 3
#define MOTB_ENA 6
#define MOTB_MSTEPS 2
#define MOTB_SERIAL Serial3
#define MOTB_CURRENT_MA 300

#define THROWER_POSITION 170
#define THROWER_DELAY 300
#define THROWER_ENABLED true

#define BUZZER_PIN -1
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
#define OLED_RESET -1
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define DISPLAY_I2C_ADDRESS 0x3C

// rotary encoder settings
#define K040_SW 22  // D22, pin 74
#define K040_DT 24 // D24, pin 72
#define K040_CLK 2   //  int pin  alt: // D26, pin 70

#define ROTARY_DELTA_VALUE 0.1
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

volatile bool turn = false;
volatile float targetValue = 10;
bool automaticModeEnabled = AUTOMODE;

void rotaryISR() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();

  // debounce 5ms
  if (interruptTime - lastInterruptTime > ROTARY_DEBOUNCE_MILLIS) {
    if (digitalRead(K040_DT) == LOW)
    {
      targetValue = targetValue - ROTARY_DELTA_VALUE;
    }
    else {
      targetValue = targetValue + ROTARY_DELTA_VALUE;
    }
    lastInterruptTime = interruptTime;
  }
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
  switch (currentState) {
    case IDLE:
      Serial.println("STATE:IDLE");
      readScaleStableOrUnstable(&value);

      if ((digitalRead(K040_SW) == LOW) || (automaticModeEnabled && (value < 0.1 && value > -0.1)) ) {
        currentState = WAIT_FOR_TARE;
        updateDisplay();
        taraScale();
      }

      break;
    case WAIT_FOR_TARE:
      Serial.println("STATE:WAIT_FOR_TARE");
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
      Serial.println("STATE:MASSFILL_FAST");
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
      Serial.println("STATE:MASSFILL_SLOW");
      if (readScaleStableOrUnstable(&value) != -1) {
        driverA.VACTUAL(speedFromValue(value, targetValue));
        if (value > targetValue - MASSFILL_SLOW_DELTA) {
          currentState = TRICKLE;
          driverA.VACTUAL((uint32_t) 0);
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
          driverA.VACTUAL(speedFromValue(value, targetValue));
        }
      } else {
        Serial.println("no stable");
        driverA.VACTUAL((uint32_t) 0);
        delay(TRICKLE_DELAY);
      }
      break;
    case FINISHED:
      Serial.println("STATE:FINISHED");
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
      Serial.println("STATE:ERROR");
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
