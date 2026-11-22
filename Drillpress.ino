#include <hd44780.h>
#include <hd44780ioClass/hd44780_pinIO.h>

// =========================================================================
// PIN DEFINITIONS
// =========================================================================
#define Temp_Pin A2
#define Cruise_Pin 1
#define Direction_Pin A4
#define Throttle_Pin 11
#define Speed_Pin 2  // Hall sensor input for RPM
#define Current_Pin A3
#define Pump_Pin 3
#define LED_Pin 12
#define Button_Pin 13
#define Pedal_Pin A5
#define Voltage_Pin A1
#define Keypad_Pin A0

// =========================================================================
// ORDERED PARAMETERIZATION CONSTANTS (Easy Tuning)
// =========================================================================
// --- Speed & RPM Limits ---
#define MIN_TARGET_RPM 90
#define MAX_TARGET_RPM 3000
#define TAPP_FWD_RPM 90.0f
#define TAPP_REV_RPM 90.0f
#define TAPP_RETRACT_RPM 90.0f
#define HALL_PULSES_PER_REV 10.0f

// --- Torque Calculation & Tapping Thresholds ---
#define MAX_SAFE_TORQUE_NM 20.0f
#define TAPP_STARTUP_GRACE_MS 500
#define TAPP_STARTUP_TORQUE_MULT 1.5f
#define TAPP_PULSE_FWD_REV 15
#define TAPP_PULSE_REV_STEP 5
#define TAPP_SUBSTATE_TIMEOUT_MS 5000  // Prevents livelock if motor stalls or pulses stop
#define MIN_PULSE_INTERVAL_US       1920UL
#define MIN_TAPP_PULSE_INTERVAL_US  20000UL

// --- Cooling & Pump Timings ---
#define COOLING_DUR_1 20       // ms
#define COOLING_DUR_2 50       // ms
#define COOLING_DUR_3 100      // ms
#define COOLING_DUR_4 200      // ms
#define COOLING_PAUSE_MS 2000  // ms between automatic bursts

// --- Shunt, Op-Amp & ADC Calibration ---
#define SHUNT_RESISTANCE_MOHM 2.0f
#define OP_AMP_GAIN 27.0f
#define V_REF_MV 1100.0f
#define V_CC_MV 5000.0f
#define R_TOP_OHMS 100000.0f
#define R_BOTTOM_OHMS 100.0f
#define DIVIDER_OFFSET_MV (V_CC_MV * (R_BOTTOM_OHMS / (R_TOP_OHMS + R_BOTTOM_OHMS)))

#define ADC_ZERO_COUNTS 93.0f
#define ADC_03A_COUNTS 97.0f
#define AMPS_PER_ADC_COUNT (0.3f / (ADC_03A_COUNTS - ADC_ZERO_COUNTS))

//#define PLOTTER_INTERVAL_MS 50

#if defined(ARDUINO_ARCH_ESP8266)
const int rs = D8, en = D9, db4 = D4, db5 = D5, db6 = D6, db7 = D7;
#else
const int rs = 8, en = 9, db4 = 4, db5 = 5, db6 = 6, db7 = 7;
#endif
hd44780_pinIO lcd(rs, en, db4, db5, db6, db7);

const int LCD_COLS = 16;
const int LCD_ROWS = 2;

// Keypad Button identifiers
enum KeyState { KEY_NONE = 0,
                KEY_SELECT,
                KEY_LEFT,
                KEY_UP,
                KEY_DOWN,
                KEY_RIGHT };

// Operation Modes
enum OperationMode { MODE_DRILL,
                     MODE_CHAMPFER,
                     MODE_TAPP };
OperationMode currentMode = MODE_DRILL;

// Drill & Tapping sizes
const int drillSizes[] = { 3, 4, 5, 6, 8, 10, 12, 15, 20, 26, 32, 40 };
const int numDrillSizes = 12;

const int tappSizes[] = { 3, 4, 5, 6, 8, 10, 12 };
int tappTorqueLimits[] = { 1, 2, 5, 7, 10, 15, 20 };
const int numTappSizes = 7;
int sizeIndex = 3;  // Default to 6mm at boot

// Cooling Global Variables
int coolingMode = 0;  // 0 = manual, 1..4 = auto levels
const float coolingScaling[] = { 1.0f, 1.3f, 1.7f, 2.2f };
unsigned long lastPumpOffTime = 0;
const unsigned long tappingCoolingDurations[] = { 0, COOLING_DUR_1, COOLING_DUR_2, COOLING_DUR_3, COOLING_DUR_4 };
unsigned long pumpOffTime = 0;

// UI Focus & Line 2 Metric
int activeFocus = 0;  // 0 = Size, 1 = Target RPM/Torque, 2 = Cooling, 3 = Line 2 Metric
int line2Metric = 0;  // 0 = Voltage, 1 = Power, 2 = Measured RPM (U)

// Manual Target RPM override per mode
int manualDrillRPM = 1000;
int manualChampferRPM = 400;

// Motor Run State, Thermal State & Tapping States
bool motorRunning = false;
bool thermalFault = false;

enum TappingSubState {
  SUB_FWD,
  SUB_CLASSIC_WAIT_1,
  SUB_WAIT_SETTLE,
  SUB_REV_DIR,
  SUB_BWD,
  SUB_WAIT_FWD,
  SUB_OIL
};

// Forward declaration so callers above the definition see it and its default argument
void enterTappingSubState(TappingSubState nextState, int pulseTargetCount = 0);

enum TappingState {
  TAPP_IDLE,
  TAPP_RUNNING,
  TAPP_CLASSIC_AUTO,
  TAPP_PAUSED_INSPECT,
  TAPP_RETRACTING
};

TappingState tappStateEnum = TAPP_IDLE;
TappingState resumedTappMode = TAPP_RUNNING;

TappingSubState tappSubState = SUB_FWD;

unsigned long tappStateTimer = 0;
unsigned long tappSubStateStartMs = 0;
unsigned long tappPassStartTime = 0;
int tappPulseTarget = 0;
int tappPulseStartCount = 0;

// Button tracking variables
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
const unsigned long longPressTime = 1000;
int lastButtonState = HIGH;
int buttonState = HIGH;
bool longPressRegistered = false;
unsigned long buttonDownTimestamp = 0;

// Global sensor variables
float motorTemp = 17.0f;
float systemVoltage = 0.0f;
float systemCurrent = 0.0f;
float spindleRPM = 0.0f;

volatile unsigned long isrLastPulseMicros = 0;
volatile unsigned long isrLastPulseMillis = 0;
volatile float isrInstantRPM = 0.0f;
volatile bool isrNewPulse = false;
volatile unsigned long totalPulseCount = 0;

void speedPulseISR() {
  unsigned long nowUs = micros();
  unsigned long dtUs = nowUs - isrLastPulseMicros;

  // Allow up to 2500 RPM for speed measurement & display
  if (dtUs >= MIN_PULSE_INTERVAL_US) {
    isrLastPulseMicros = nowUs;
    isrLastPulseMillis = millis();
    isrInstantRPM = 60000000.0f / (float)dtUs; // Adjust divisor if using multiple magnets/rev
    isrNewPulse = true;

    // Count pulses spaced for slow tapping (less than 200 RPM)
    if (dtUs >= MIN_TAPP_PULSE_INTERVAL_US) {
      totalPulseCount++;
    }
  }
}

// Forward declarations
float calculateOptimalRPM(OperationMode mode, int diameterMM);
float getActiveTargetRPM();
float calculateTorque();
void setSpeed(float targetRPM, float currentRPM, bool enabled);
int safeAnalogRead(uint8_t pin);

void setup() {
  //Serial.begin(115200);
  lcd.begin(LCD_COLS, LCD_ROWS);
  analogReference(INTERNAL);

  pinMode(Button_Pin, INPUT_PULLUP);
  pinMode(Pedal_Pin, INPUT_PULLUP);

  pinMode(Speed_Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(Speed_Pin), speedPulseISR, RISING);
  pinMode(Temp_Pin, INPUT);
  pinMode(Current_Pin, INPUT);
  pinMode(Voltage_Pin, INPUT);
  pinMode(Keypad_Pin, INPUT);
  pinMode(Direction_Pin, INPUT);

  pinMode(Pump_Pin, OUTPUT);
  pinMode(LED_Pin, OUTPUT);
  pinMode(Throttle_Pin, OUTPUT);
  digitalWrite(Pump_Pin, LOW);
  digitalWrite(LED_Pin, LOW);
  digitalWrite(Throttle_Pin, LOW);

  // Settle internal ADC reference
  for (int i = 0; i < 50; i++) {
    safeAnalogRead(Current_Pin);
    delay(10);
  }

  manualDrillRPM = constrain((int)calculateOptimalRPM(MODE_DRILL, drillSizes[sizeIndex]), MIN_TARGET_RPM, MAX_TARGET_RPM);
  manualChampferRPM = constrain((int)calculateOptimalRPM(MODE_CHAMPFER, drillSizes[sizeIndex]), MIN_TARGET_RPM, MAX_TARGET_RPM);
}

void loop() {
  // 1. Slow Sensors (Voltage & Temp) at 2Hz (500ms)
  static unsigned long lastSensorReadTime = 0;
  if (millis() - lastSensorReadTime >= 500) {
    lastSensorReadTime = millis();
    motorTemp = readMotorTemperature();
    systemVoltage = readVoltage();
  }

  // 2. Keypad Navigation & Mode Switching
  KeyState key = readKeypad();
  handleKeypadInput(key);

  // 3. Thermal Protection
  checkTemp();

  // 4. Spindle RPM Measurement
  spindleRPM = getSpindleRPM();

  // 5. Button Actions (Start/Stop/Pause/Resume/Retract)
  int btnAction = checkButton();

  if (btnAction == 1 && !thermalFault) {  // Short Press
    if (currentMode == MODE_TAPP) {
      if (tappStateEnum == TAPP_IDLE) {
        tappStateEnum = TAPP_RUNNING;
        resumedTappMode = TAPP_RUNNING;
        pinMode(Direction_Pin, INPUT);
        enterTappingSubState(SUB_FWD);
        tappPassStartTime = millis();
        motorRunning = true;
        digitalWrite(LED_Pin, HIGH);
      } else if (tappStateEnum == TAPP_RUNNING || tappStateEnum == TAPP_CLASSIC_AUTO) {
        resumedTappMode = tappStateEnum;
        tappStateEnum = TAPP_PAUSED_INSPECT;
        motorRunning = false;
        pinMode(Direction_Pin, INPUT);
        setSpeed(0, spindleRPM, false);
      } else if (tappStateEnum == TAPP_PAUSED_INSPECT) {
        tappStateEnum = TAPP_RETRACTING;
        tappSubStateStartMs = millis();
        pinMode(Direction_Pin, OUTPUT);
        digitalWrite(Direction_Pin, LOW);
        motorRunning = true;
        digitalWrite(LED_Pin, HIGH);
      } else if (tappStateEnum == TAPP_RETRACTING) {
        tappStateEnum = TAPP_IDLE;
        motorRunning = false;
        digitalWrite(LED_Pin, LOW);
        pinMode(Direction_Pin, INPUT);
        setSpeed(0, spindleRPM, false);
      }
    } else {
      motorRunning = !motorRunning;
      if (!motorRunning) {
        setSpeed(0, spindleRPM, false);  // Also clears Cruise_Pin and LED_Pin via setCruiseActive(false)
      }
    }
  } else if (btnAction == 2 && !thermalFault) {  // Long Press (>= 1s)
    if (currentMode == MODE_TAPP) {
      if (tappStateEnum == TAPP_IDLE) {
        tappStateEnum = TAPP_CLASSIC_AUTO;
        resumedTappMode = TAPP_CLASSIC_AUTO;
        pinMode(Direction_Pin, INPUT);
        enterTappingSubState(SUB_FWD);
        tappPassStartTime = millis();
        motorRunning = true;
        digitalWrite(LED_Pin, HIGH);
      } else if (tappStateEnum == TAPP_PAUSED_INSPECT) {
        tappStateEnum = resumedTappMode;
        pinMode(Direction_Pin, INPUT);
        enterTappingSubState(SUB_FWD);
        tappPassStartTime = millis();
        motorRunning = true;
        digitalWrite(LED_Pin, HIGH);
      }
    }
  }
  // 6. Fast Control & High-Frequency Current Sampling Loop (100Hz / 10ms)
  static unsigned long lastControlTime = 0;
  if (millis() - lastControlTime >= 10) {
    lastControlTime = millis();

    systemCurrent = readCurrent();

    if (thermalFault) {
      setSpeed(0, spindleRPM, false);
    } else if (currentMode == MODE_TAPP) {
      runTappingStateMachine();
    } else {
      float targetRPM = getActiveTargetRPM();
      setSpeed(targetRPM, spindleRPM, motorRunning);
    }
  }

  // 7. Cooling Management every 100ms (Drill / Chamfer modes)
  static unsigned long lastCoolingCheck = 0;
  if (millis() - lastCoolingCheck >= 100) {
    lastCoolingCheck = millis();
    if (currentMode != MODE_TAPP) {
      updateCooling();
    }
  }

  // 8. Display Rendering (500ms)
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate >= 500) {
    lastDisplayUpdate = millis();
    renderDisplayLine1();
    renderDisplayLine2();
  }
/*
  // 9. Continuous Serial Plotter Torque Output
  static unsigned long lastPlotTime = 0;
  if (millis() - lastPlotTime >= PLOTTER_INTERVAL_MS) {
    lastPlotTime = millis();
    //Serial.print(F("Torque_Nm:"));
    //Serial.println(calculateTorque(), 2);
  }
*/
  // 10. Centralized Pump Arbitration
  bool pedalPressed = (digitalRead(Pedal_Pin) == LOW);
  if (pedalPressed || millis() < pumpOffTime) {
    digitalWrite(Pump_Pin, HIGH);
  } else {
    digitalWrite(Pump_Pin, LOW);
  }
}

/* =========================================================================
   HELPER: REQUEST A SOLID PUMP BURST FOR X MILLISECONDS
   ========================================================================= */
void requestPumpBurst(unsigned long duration_ms) {
  unsigned long targetOff = millis() + duration_ms;
  if (targetOff > pumpOffTime) {
    pumpOffTime = targetOff;
  }
}

/* =========================================================================
   ROBUST ADC MULTIPLEXING HELPER
   ========================================================================= */
int safeAnalogRead(uint8_t pin) {
  analogRead(pin);
  delayMicroseconds(150);
  analogRead(pin);
  delayMicroseconds(50);
  return analogRead(pin);
}

/* =========================================================================
   1. KEYPAD READER & NAVIGATION HANDLER
   ========================================================================= */
KeyState readKeypad() {
  int rawVal = safeAnalogRead(Keypad_Pin);

  if (rawVal >= 800) return KEY_NONE;
  else if (rawVal >= 380 && rawVal <= 435) return KEY_SELECT;
  else if (rawVal >= 250 && rawVal <= 295) return KEY_LEFT;
  else if (rawVal >= 150 && rawVal <= 195) return KEY_DOWN;
  else if (rawVal >= 45 && rawVal <= 90) return KEY_UP;
  else if (rawVal >= 0 && rawVal <= 25) return KEY_RIGHT;

  return KEY_NONE;
}

void handleKeypadInput(KeyState key) {
  static unsigned long lastPressTime = 0;
  if (key == KEY_NONE || millis() - lastPressTime < 300) return;
  lastPressTime = millis();

  if (key == KEY_SELECT) {
    // Preserve sizeIndex across Drilling, Countersinking (Champfer), and Tapping
    if (currentMode == MODE_DRILL) {
      currentMode = MODE_CHAMPFER;
    } else if (currentMode == MODE_CHAMPFER) {
      currentMode = MODE_TAPP;
      tappStateEnum = TAPP_IDLE;
      motorRunning = false;
      digitalWrite(LED_Pin, LOW);
    } else {
      currentMode = MODE_DRILL;
      motorRunning = false;
      digitalWrite(LED_Pin, LOW);
    }
    syncRPMForCurrentMode();
  } else if (key == KEY_LEFT) {
    activeFocus = (activeFocus - 1 + 4) % 4;
  } else if (key == KEY_RIGHT) {
    activeFocus = (activeFocus + 1) % 4;
  } else if (key == KEY_UP) {
    if (activeFocus == 0) {
      int maxIdx = (currentMode == MODE_TAPP) ? (numTappSizes - 1) : (numDrillSizes - 1);
      if (sizeIndex < maxIdx) sizeIndex++;
      if (currentMode == MODE_DRILL) {
        manualDrillRPM = constrain((int)calculateOptimalRPM(currentMode, drillSizes[sizeIndex]), MIN_TARGET_RPM, MAX_TARGET_RPM);
      } else if (currentMode == MODE_CHAMPFER) {
        manualChampferRPM = constrain((int)calculateOptimalRPM(currentMode, drillSizes[sizeIndex]), MIN_TARGET_RPM, MAX_TARGET_RPM);
      }
    } else if (activeFocus == 1) {
      if (currentMode == MODE_TAPP) {
        if (tappTorqueLimits[sizeIndex] < 20) tappTorqueLimits[sizeIndex]++;
      } else if (currentMode == MODE_DRILL) {
        manualDrillRPM = constrain(manualDrillRPM + 25, MIN_TARGET_RPM, MAX_TARGET_RPM);
      } else {
        manualChampferRPM = constrain(manualChampferRPM + 25, MIN_TARGET_RPM, MAX_TARGET_RPM);
      }
    } else if (activeFocus == 2) {
      if (coolingMode < 4) coolingMode++;
    } else if (activeFocus == 3) {
      line2Metric = (line2Metric + 1) % 3;
    }
  } else if (key == KEY_DOWN) {
    if (activeFocus == 0) {
      if (sizeIndex > 0) sizeIndex--;
      if (currentMode == MODE_DRILL) {
        manualDrillRPM = constrain((int)calculateOptimalRPM(currentMode, drillSizes[sizeIndex]), MIN_TARGET_RPM, MAX_TARGET_RPM);
      } else if (currentMode == MODE_CHAMPFER) {
        manualChampferRPM = constrain((int)calculateOptimalRPM(currentMode, drillSizes[sizeIndex]), MIN_TARGET_RPM, MAX_TARGET_RPM);
      }
    } else if (activeFocus == 1) {
      if (currentMode == MODE_TAPP) {
        if (tappTorqueLimits[sizeIndex] > 1) tappTorqueLimits[sizeIndex]--;
      } else if (currentMode == MODE_DRILL) {
        manualDrillRPM = constrain(manualDrillRPM - 25, MIN_TARGET_RPM, MAX_TARGET_RPM);
      } else {
        manualChampferRPM = constrain(manualChampferRPM - 25, MIN_TARGET_RPM, MAX_TARGET_RPM);
      }
    } else if (activeFocus == 2) {
      if (coolingMode > 0) coolingMode--;
    } else if (activeFocus == 3) {
      line2Metric = (line2Metric - 1 + 3) % 3;
    }
  }
}

/* =========================================================================
   2. SENSOR FUNCTIONS, TORQUE & THERMAL PROTECTION
   ========================================================================= */
float readMotorTemperature() {
  static float smoothedTemp = 17.0f;
  int rawADC = safeAnalogRead(Temp_Pin);
  if (rawADC <= 0) return smoothedTemp;
  if (rawADC >= 1023) return 300.0f;

  float voltage = (rawADC / 1023.0f) * 1.1f;
  float rPullup = 6000.0f;
  float rSensor = rPullup * (voltage / (1.1f - voltage));
  float r17 = rPullup * ((414.0f / 1023.0f) / (1.0f - (414.0f / 1023.0f)));
  float rawTempC = 17.0f + ((rSensor - r17) / 9.5f);

  smoothedTemp = (0.99f * smoothedTemp) + (0.01f * rawTempC);
  return smoothedTemp;
}

void checkTemp() {
  if (motorTemp > 130.0f) {
    if (!thermalFault) {
      thermalFault = true;
      motorRunning = false;
      tappStateEnum = TAPP_IDLE;
      digitalWrite(LED_Pin, LOW);
      analogWrite(Throttle_Pin, 0);
      pinMode(Direction_Pin, INPUT);
    }
  } else if (thermalFault && motorTemp < 120.0f) {
    thermalFault = false;
  }
}

float readVoltage() {
  static float smoothedVoltage = 0.0f;
  int rawADC = safeAnalogRead(Voltage_Pin);
  float rawVoltage = (float)rawADC * (49.9f / 565.0f);

  if (smoothedVoltage == 0.0f) smoothedVoltage = rawVoltage;
  else smoothedVoltage = (0.99f * smoothedVoltage) + (0.01f * rawVoltage);

  return smoothedVoltage;
}

float readCurrent() {
  long adcSum = 0;
  const int samples = 8;
  for (int i = 0; i < samples; i++) {
    adcSum += safeAnalogRead(Current_Pin);
    delayMicroseconds(25);
  }
  float rawADC = (float)adcSum / (float)samples;

  static float smoothedADC = rawADC;
  smoothedADC = (0.75f * smoothedADC) + (0.25f * rawADC);

  float calculatedAmps = (smoothedADC - ADC_ZERO_COUNTS) * AMPS_PER_ADC_COUNT;
  if (calculatedAmps < 0.0f) {
    calculatedAmps = 0.0f;
  }

  static float smoothedCurrent = 0.0f;
  if (smoothedCurrent == 0.0f) smoothedCurrent = calculatedAmps;
  else smoothedCurrent = (0.99f * smoothedCurrent) + (0.01f * calculatedAmps);

  return smoothedCurrent;
}

float getPower() {
  return systemVoltage * systemCurrent;
}

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
  if (inMax == inMin) return outMin;
  float t = constrain((x - inMin) / (inMax - inMin), 0.0f, 1.0f);
  return outMin + t * (outMax - outMin);
}

float calculateTorque() {
  if (!motorRunning) return 0.0f;

  float effectiveRPM = getActiveTargetRPM();
  if (effectiveRPM <= 1.0f) return 0.0f;

  float powerWatts = getPower();
  float omega = effectiveRPM * (2.0f * 3.14159f / 60.0f);
  float torqueNm = powerWatts / omega;

  // Subtract 1.0 Nm at MIN_TARGET_RPM (90) down to 0.1 Nm at MAX_TARGET_RPM (3000)
  float offsetNm = mapFloat(
    effectiveRPM,
    (float)MIN_TARGET_RPM,
    (float)MAX_TARGET_RPM,
    1.0f,
    0.1f);
  torqueNm -= offsetNm;

  if (torqueNm < 0.0f) return 0.0f;
  if (torqueNm > MAX_SAFE_TORQUE_NM) return MAX_SAFE_TORQUE_NM;
  return torqueNm;
}

/* =========================================================================
   3. HIGH-FREQUENCY SPINDLE RPM READER
   ========================================================================= */
float getSpindleRPM() {
  unsigned long lastPulseMs;
  float latestRPM;
  bool gotPulse;

  noInterrupts();
  lastPulseMs = isrLastPulseMillis;
  latestRPM = isrInstantRPM;
  gotPulse = isrNewPulse;
  isrNewPulse = false;
  interrupts();

  static float smoothedRPM = 0.0f;
  unsigned long nowMs = millis();

  // If no pulses arrive for > 300 ms, treat spindle as stopped / near 0 RPM
  if (lastPulseMs == 0 || (nowMs - lastPulseMs) > 300) {
    smoothedRPM = 0.0f;
    return 0.0f;
  }

  if (gotPulse && latestRPM >= 0.0f && latestRPM <= 5000.0f) {
    smoothedRPM = (0.9f * smoothedRPM) + (0.1f * latestRPM);
  }

  if (smoothedRPM < 5.0f) {
    smoothedRPM = 0.0f;
  }

  return smoothedRPM;
}

/* =========================================================================
   4. VOLTAGE-COMPENSATED PWM MAPPING & STEP-CONTROLLER
   ========================================================================= */
float calculateOptimalRPM(OperationMode mode, int diameterMM) {
  if (mode == MODE_TAPP) {
    return TAPP_FWD_RPM;
  }
  float vc = (mode == MODE_DRILL) ? 25.0f : 10.0f;
  return (vc * 1000.0f) / (3.14159f * (float)diameterMM);
}

float getActiveTargetRPM() {
  if (currentMode == MODE_DRILL) return (float)manualDrillRPM;
  if (currentMode == MODE_CHAMPFER) return (float)manualChampferRPM;
  if (currentMode == MODE_TAPP) {
    if (tappStateEnum == TAPP_RETRACTING || tappSubState == SUB_REV_DIR || tappSubState == SUB_BWD) {
      return TAPP_REV_RPM;
    }
    return TAPP_FWD_RPM;
  }
  return 1000.0f;
}

float pwmFromRPM(float rpm) {
  float basePWM = 70.0f;
  if (rpm <= 60.0f) basePWM = 70.0f;
  else if (rpm <= 511.0f) basePWM = map(rpm, 60, 511, 70, 90);
  else if (rpm <= 1263.0f) basePWM = map(rpm, 511, 1263, 90, 130);
  else if (rpm <= 1700.0f) basePWM = map(rpm, 1263, 1700, 130, 170);
  else basePWM = map(rpm, 1700, 2700, 170, 240);

  float v = (systemVoltage > 10.0f) ? systemVoltage : 49.5f;
  float compensatedPWM = basePWM * (49.5f / v);
  return constrain(compensatedPWM, 70.0f, 255.0f);
}

void setCruiseActive(bool active) {
  if (active) {
    digitalWrite(Cruise_Pin, LOW);
    pinMode(Cruise_Pin, OUTPUT);
  } else {
    pinMode(Cruise_Pin, INPUT);
    digitalWrite(Cruise_Pin, LOW);
  }

  if (currentMode == MODE_DRILL || currentMode == MODE_CHAMPFER) {
    digitalWrite(LED_Pin, active ? HIGH : LOW);
  }
}

void setSpeed(float targetRPM, float currentRPM, bool enabled) {
  (void)currentRPM;  // Cruise control manages speed regulation after spin-up

  static unsigned long motorStartTime = 0;
  static bool initialSpinUpDone = false;
  static bool lastEnabledState = false;

  if (!enabled || thermalFault) {
    setCruiseActive(false);
    initialSpinUpDone = false;
    motorStartTime = 0;
    lastEnabledState = false;
    analogWrite(Throttle_Pin, 0);
    return;
  }

  if (!lastEnabledState && enabled) {
    setCruiseActive(false);
    motorStartTime = millis();
    initialSpinUpDone = false;
  }
  lastEnabledState = enabled;

  int throttlePWM = (int)pwmFromRPM(targetRPM);
  analogWrite(Throttle_Pin, throttlePWM);

  if (!initialSpinUpDone && (millis() - motorStartTime >= 3000)) {
    initialSpinUpDone = true;
    setCruiseActive(true);
  } else if (initialSpinUpDone) {
    setCruiseActive(true);
  }
}

/* =========================================================================
   5. UNIFIED TAPPING STATE MACHINE (Torque Trigger + Optional Periodic Classic)
   ========================================================================= */

void enterTappingSubState(TappingSubState nextState, int pulseTargetCount = 0) {
  tappSubState = nextState;
  tappSubStateStartMs = millis();
  tappPulseStartCount = totalPulseCount;
  tappPulseTarget = pulseTargetCount;
  if (nextState == SUB_OIL) {
    tappStateTimer = millis();
  }
}

void runTappingCycle(bool classic) {
  unsigned long now = millis();
  bool inGrace = (now - tappPassStartTime <= TAPP_STARTUP_GRACE_MS);
  int tappedIdx = constrain(sizeIndex, 0, numTappSizes - 1);
  float activeTorqueLimit = inGrace
                              ? ((float)tappTorqueLimits[tappedIdx] * TAPP_STARTUP_TORQUE_MULT)
                              : (float)tappTorqueLimits[tappedIdx];
  bool torqueExceeded = (!inGrace && calculateTorque() >= activeTorqueLimit);

  unsigned long activeOilDuration = (coolingMode > 0 && coolingMode <= 4) ? tappingCoolingDurations[coolingMode] : 0;

  // Torque overload triggers the shared stop -> reverse -> half-turn back -> oil sequence
  if ((tappSubState == SUB_FWD || tappSubState == SUB_CLASSIC_WAIT_1) && torqueExceeded) {
    setSpeed(0, spindleRPM, false);
    enterTappingSubState(SUB_WAIT_SETTLE);
    return;
  }

  switch (tappSubState) {
    case SUB_FWD:
      pinMode(Direction_Pin, INPUT);
      setSpeed(TAPP_FWD_RPM, spindleRPM, true);
      if (classic) {
        enterTappingSubState(SUB_CLASSIC_WAIT_1, TAPP_PULSE_FWD_REV);
      }
      break;

    case SUB_CLASSIC_WAIT_1:
      setSpeed(TAPP_FWD_RPM, spindleRPM, true);
      if (((int)(totalPulseCount - tappPulseStartCount) >= tappPulseTarget) || (now - tappSubStateStartMs >= TAPP_SUBSTATE_TIMEOUT_MS)) {
        setSpeed(0, spindleRPM, false);
        enterTappingSubState(SUB_WAIT_SETTLE);
      }
      break;

    case SUB_WAIT_SETTLE:
      setSpeed(0, spindleRPM, false);
      if (spindleRPM < 5.0f || (now - tappSubStateStartMs >= TAPP_SUBSTATE_TIMEOUT_MS)) {
        enterTappingSubState(SUB_REV_DIR);
      }
      break;

    case SUB_REV_DIR:
      pinMode(Direction_Pin, OUTPUT);
      digitalWrite(Direction_Pin, LOW);
      enterTappingSubState(SUB_BWD, TAPP_PULSE_REV_STEP);
      break;

    case SUB_BWD:
      setSpeed(TAPP_REV_RPM, spindleRPM, true);
      if (((int)(totalPulseCount - tappPulseStartCount) >= tappPulseTarget) || (now - tappSubStateStartMs >= TAPP_SUBSTATE_TIMEOUT_MS)) {
        setSpeed(0, spindleRPM, false);
        enterTappingSubState(SUB_WAIT_FWD);
      }
      break;

    case SUB_WAIT_FWD:
      setSpeed(0, spindleRPM, false);
      if (spindleRPM < 5.0f || (now - tappSubStateStartMs >= TAPP_SUBSTATE_TIMEOUT_MS)) {
        pinMode(Direction_Pin, INPUT);
        if (coolingMode > 0 && activeOilDuration > 0) {
          enterTappingSubState(SUB_OIL);
        } else {
          tappPassStartTime = now;
          enterTappingSubState(SUB_FWD);
        }
      }
      break;

    case SUB_OIL:
      if (coolingMode > 0 && activeOilDuration > 0) {
        if (now - tappStateTimer <= activeOilDuration) {
          requestPumpBurst(activeOilDuration);
        } else {
          tappPassStartTime = now;
          enterTappingSubState(SUB_FWD);
        }
      } else {
        tappPassStartTime = now;
        enterTappingSubState(SUB_FWD);
      }
      break;
  }
}

void runTappingStateMachine() {
  if (tappStateEnum == TAPP_IDLE || tappStateEnum == TAPP_PAUSED_INSPECT || thermalFault) {
    setSpeed(0, spindleRPM, false);
    pinMode(Direction_Pin, INPUT);
    return;
  }

  if (tappStateEnum == TAPP_RETRACTING) {
    pinMode(Direction_Pin, OUTPUT);
    digitalWrite(Direction_Pin, LOW);
    setSpeed(TAPP_RETRACT_RPM, spindleRPM, true);
    return;
  }

  runTappingCycle(tappStateEnum == TAPP_CLASSIC_AUTO);
}

/* =========================================================================
   6. COOLING MANAGEMENT SYSTEM (Drill / Chamfer Modes)
   ========================================================================= */
void updateCooling() {
  if (coolingMode == 0 || calculateTorque() <= 1.0f) {
    return;
  }

  unsigned long currentMillis = millis();
  if (currentMillis - lastPumpOffTime < COOLING_PAUSE_MS) {
    return;
  }

  static unsigned long pumpOnTime = 0;
  static bool pumpActive = false;

  float power = getPower();
  unsigned long baseDuration = COOLING_DUR_1;
  if (coolingMode == 2) baseDuration = COOLING_DUR_2;
  else if (coolingMode == 3) baseDuration = COOLING_DUR_3;
  else if (coolingMode == 4) baseDuration = COOLING_DUR_4;

  float scalingFactor = coolingScaling[coolingMode - 1];
  unsigned long activeDuration = (unsigned long)(baseDuration * scalingFactor * (1.0f + (power / 50.0f)));

  if (!pumpActive) {
    pumpActive = true;
    pumpOnTime = currentMillis;
  } else {
    if (currentMillis - pumpOnTime >= activeDuration) {
      pumpActive = false;
      lastPumpOffTime = currentMillis;
    }
  }

  if (pumpActive) {
    requestPumpBurst(activeDuration);
  }
}

/* =========================================================================
   7. DISPLAY & BUTTON UTILITIES
   ========================================================================= */
void printPadded4(int val) {
  if (val < 10) lcd.print(F("   "));
  else if (val < 100) lcd.print(F("  "));
  else if (val < 1000) lcd.print(F(" "));
  lcd.print(val);
}

void renderDisplayLine1() {
  lcd.setCursor(0, 0);
  bool blink = (millis() / 300) % 2 == 0;

  if (currentMode == MODE_DRILL) lcd.print(F("D"));
  else if (currentMode == MODE_CHAMPFER) lcd.print(F("C"));
  else lcd.print(F("T"));

  if (activeFocus == 0 && !blink) lcd.print(' ');
  else lcd.print(':');

  int size = (currentMode == MODE_TAPP)
               ? tappSizes[constrain(sizeIndex, 0, numTappSizes - 1)]
               : drillSizes[constrain(sizeIndex, 0, numDrillSizes - 1)];
  if (size < 10) lcd.print('0');
  lcd.print(size);
  lcd.print(' ');

  if (currentMode == MODE_TAPP) {
    lcd.print(F("TRQ"));
    if (activeFocus == 1 && !blink) lcd.print(' ');
    else lcd.print(':');
    int trq = tappTorqueLimits[constrain(sizeIndex, 0, numTappSizes - 1)];
    if (trq < 10) lcd.print(' ');
    lcd.print(trq);
    lcd.print(F("Nm "));
  } else {
    int targetRPM = (int)getActiveTargetRPM();
    lcd.print(F("RPM"));

    if (activeFocus == 1 && !blink) lcd.print(' ');
    else lcd.print(':');

    if (targetRPM < 100) lcd.print(F("  "));
    else if (targetRPM < 1000) lcd.print(F(" "));
    lcd.print(targetRPM);
    lcd.print(' ');
  }

  if (activeFocus == 2 && !blink) lcd.print(' ');
  else lcd.print('C');
  lcd.print(coolingMode);
}

void renderDisplayLine2() {
  lcd.setCursor(0, 1);
  bool blink = (millis() / 300) % 2 == 0;

  if (line2Metric == 0) {
    lcd.print(systemVoltage, 1);
    if (activeFocus == 3 && !blink) lcd.print(F("  "));
    else lcd.print(F("V "));
  } else if (line2Metric == 1) {
    printPadded4((int)getPower());
    if (activeFocus == 3 && !blink) lcd.print(F("  "));
    else lcd.print(F("W "));
  } else {
    printPadded4((int)spindleRPM);
    if (activeFocus == 3 && !blink) lcd.print(F("  "));
    else lcd.print(F("U "));
  }

  int torqueInt = (int)calculateTorque();
  if (torqueInt < 10) lcd.print(' ');
  lcd.print(torqueInt);
  lcd.print(F("Nm "));

  int tempInt = (int)motorTemp;
  if (tempInt < 100) lcd.print(' ');
  lcd.print(tempInt);
  lcd.print((char)223);
  lcd.print(F("C  "));
}

int checkButton() {
  int reading = digitalRead(Button_Pin);
  int action = 0;

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        buttonDownTimestamp = millis();
        longPressRegistered = false;
      } else {
        if (!longPressRegistered) {
          action = 1;
        }
      }
    }

    if (buttonState == LOW && !longPressRegistered) {
      if ((millis() - buttonDownTimestamp) >= longPressTime) {
        longPressRegistered = true;
        action = 2;
      }
    }
  }

  lastButtonState = reading;
  return action;
}

void syncRPMForCurrentMode() {
  int idx = constrain(sizeIndex, 0, numDrillSizes - 1);
  int diameterMM = drillSizes[idx];

  if (currentMode == MODE_DRILL) {
    manualDrillRPM = constrain(
      (int)calculateOptimalRPM(MODE_DRILL, diameterMM),
      MIN_TARGET_RPM,
      MAX_TARGET_RPM);
  } else if (currentMode == MODE_CHAMPFER) {
    manualChampferRPM = constrain(
      (int)calculateOptimalRPM(MODE_CHAMPFER, diameterMM),
      MIN_TARGET_RPM,
      MAX_TARGET_RPM);
  }
}
