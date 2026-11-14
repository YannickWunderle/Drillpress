#include <hd44780.h>
#include <hd44780ioClass/hd44780_pinIO.h>

#define Temp_Pin A2
#define Cruise_Pin 1
#define Direction_Pin 2
#define Throttle_Pin 11
#define Speed_Pin A4        // Hall sensor input for RPM
#define Current_Pin A3
#define Pump_Pin 3
#define LED_Pin 12
#define Button_Pin 13
#define Pedal_Pin A5
#define Voltage_Pin A1
#define Keypad_Pin A0

#if defined (ARDUINO_ARCH_ESP8266)
const int rs=D8, en=D9, db4=D4, db5=D5, db6=D6, db7=D7;
#else
const int rs=8, en=9, db4=4, db5=5, db6=6, db7=7;
#endif
hd44780_pinIO lcd(rs, en, db4, db5, db6, db7);

const int LCD_COLS = 16;
const int LCD_ROWS = 2;

// Keypad Button identifiers
enum KeyState { KEY_NONE = 0, KEY_SELECT, KEY_LEFT, KEY_UP, KEY_DOWN, KEY_RIGHT };

// Operation Modes
enum OperationMode { MODE_DRILL, MODE_CHAMPFER, MODE_TAPP };
OperationMode currentMode = MODE_DRILL;

// Drill sizes arrays
const int drillSizes[] = {3, 4, 5, 6, 8, 10, 12, 15, 20, 26, 32, 40};
const int numDrillSizes = 12;

const int tappSizes[] = {3, 4, 5, 6, 8, 10, 12};
const int numTappSizes = 7;
int sizeIndex = 3; // Default to 6mm for Drill/Champfer, 8mm for Tapping

// Cooling Global Variables & Constants
int coolingMode = 0; // Default to manual (0)
#define COOLING_DUR_1 20   // ms
#define COOLING_DUR_2 50   // ms
#define COOLING_DUR_3 100  // ms
#define COOLING_DUR_4 200  // ms
const float coolingScaling[] = {1.0f, 1.3f, 1.7f, 2.2f};

// Tapping specific cooling
#define TAPPING_COOLING_DUR 800 // ms for oil reservoir pump pulse

// UI Focus: 0 = Size, 1 = Target RPM, 2 = Cooling Mode, 3 = Line 2 Left Metric (Volt / Power / U)
int activeFocus = 0; 
int line2Metric = 0; // 0 = Voltage, 1 = Power, 2 = Measured RPM (U)

// Manual Target RPM override per mode
int manualDrillRPM = 1000;
int manualChampferRPM = 800;
int manualTappRPM = 1200;

// Motor Run State Toggle & Tapping States
bool motorRunning = false;
enum TappingState { TAPP_IDLE, TAPP_FWD, TAPP_WAIT_1, TAPP_REV_DIR, TAPP_BWD, TAPP_WAIT_2, TAPP_OIL, TAPP_MANUAL_REV, TAPP_PAUSED };
TappingState tappState = TAPP_IDLE;
unsigned long tappStateTimer = 0;
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
float motorTemp = 17.0;
float systemVoltage = 0.0;
float systemCurrent = 0.0;
float spindleRPM = 0.0;
unsigned long totalPulseCount = 0; // Cumulative pulse counter for precision moves

void setup() {
  Serial.begin(9600);
  lcd.begin(LCD_COLS, LCD_ROWS);
  analogReference(INTERNAL); // Set internal 1.1V reference

  // Configure button and pedal pins with internal pull-up resistors
  pinMode(Button_Pin, INPUT_PULLUP);
  pinMode(Pedal_Pin, INPUT_PULLUP);
  
  // Configure pins as inputs
  pinMode(Speed_Pin, INPUT);
  pinMode(Temp_Pin, INPUT);
  pinMode(Current_Pin, INPUT);
  pinMode(Voltage_Pin, INPUT);
  pinMode(Keypad_Pin, INPUT);
  pinMode(Direction_Pin, INPUT); // Floating input by default

  // Configure control and indicator outputs
  pinMode(Pump_Pin, OUTPUT);
  pinMode(LED_Pin, OUTPUT);
  pinMode(Throttle_Pin, OUTPUT);
  digitalWrite(Pump_Pin, LOW);
  digitalWrite(LED_Pin, LOW);
  digitalWrite(Throttle_Pin, LOW);

  // Initialize manual RPM defaults based on standard calc
  manualDrillRPM = (int)calculateOptimalRPM(MODE_DRILL, drillSizes[sizeIndex]);
  manualChampferRPM = (int)calculateOptimalRPM(MODE_CHAMPFER, drillSizes[sizeIndex]);
  manualTappRPM = 120;
}

void loop() {
  // 1. Read Keypad & Serial Override & Handle Navigation / Mode Switching
  KeyState key = readKeypad();
  if (Serial.available() > 0) {
    char sChar = Serial.read();
    if (sChar == 'S' || sChar == 's') key = KEY_SELECT;
    else if (sChar == 'L' || sChar == 'l') key = KEY_LEFT;
    else if (sChar == 'R' || sChar == 'r') key = KEY_RIGHT;
    else if (sChar == 'U' || sChar == 'u') key = KEY_UP;
    else if (sChar == 'D' || sChar == 'd') key = KEY_DOWN;
  }
  handleKeypadInput(key);

  // 2. Read Sensors every 500ms with EMA smoothing
  static unsigned long lastSensorReadTime = 0;
  if (millis() - lastSensorReadTime >= 500) {
    lastSensorReadTime = millis();
    motorTemp = readMotorTemperature();
    systemVoltage = readVoltage();
    systemCurrent = readCurrent();
  }

  // 3. Read Spindle RPM & Button Action
  spindleRPM = getSpindleRPM();
  int btnAction = checkButton();

  // Handle Button Actions
  if (btnAction == 1) { // Short Press
    if (currentMode == MODE_TAPP) {
      if (tappState == TAPP_PAUSED) {
        tappState = TAPP_MANUAL_REV; 
      } else if (tappState == TAPP_MANUAL_REV) {
        tappState = TAPP_PAUSED;
        motorRunning = false;
      } else {
        tappState = TAPP_PAUSED;
        motorRunning = false;
      }
    } else {
      motorRunning = !motorRunning;
    }
  } else if (btnAction == 2) { // Long Press (> 1s)
    if (currentMode == MODE_TAPP) {
      tappState = TAPP_FWD; 
      motorRunning = true;
    }
  }

  // 4. Speed & Operation Control (100Hz / 10ms loop)
  static unsigned long lastControlTime = 0;
  if (millis() - lastControlTime >= 10) {
    lastControlTime = millis();
    
    if (currentMode == MODE_TAPP) {
      runTappingStateMachine();
    } else {
      float targetRPM = getActiveTargetRPM();
      setSpeed(targetRPM, spindleRPM, motorRunning);
    }
  }

  // 5. Update Cooling every 100ms
  static unsigned long lastCoolingCheck = 0;
  if (millis() - lastCoolingCheck >= 100) {
    lastCoolingCheck = millis();
    updateCooling();
  }

  // 6. Display Rendering
  renderDisplayLine1();
  renderDisplayLine2();
}

/* =========================================================================
   ROBUST ADC MULTIPLEXING HELPER (Prevents cross-talk)
   ========================================================================= */
int safeAnalogRead(uint8_t pin) {
  analogRead(pin);             // 1st dummy read to switch MUX channel
  delayMicroseconds(150);      // Stabilization delay
  analogRead(pin);             // 2nd dummy read to ensure charge dissipation
  delayMicroseconds(50);
  return analogRead(pin);      // Clean read
}

/* =========================================================================
   1. KEYPAD READER & NAVIGATION HANDLER + DIAGNOSTICS
   ========================================================================= */
KeyState readKeypad() {
  int rawVal = safeAnalogRead(Keypad_Pin);

  // Diagnostic print for keypad debugging via Serial Monitor
  static unsigned long lastKeyDebug = 0;
  if (millis() - lastKeyDebug > 500) {
    lastKeyDebug = millis();
    Serial.print(F("Keypad Raw ADC: ")); Serial.println(rawVal);
  }

  KeyState currentKey = KEY_NONE;
  if (rawVal >= 800)                     currentKey = KEY_NONE;   // Idle / None (~1023)
  else if (rawVal >= 380 && rawVal <= 435) currentKey = KEY_SELECT; // Target ~406
  else if (rawVal >= 250 && rawVal <= 295) currentKey = KEY_LEFT;   // Target ~270
  else if (rawVal >= 150 && rawVal <= 195) currentKey = KEY_DOWN;   // Target ~173
  else if (rawVal >= 45  && rawVal <= 90)  currentKey = KEY_UP;     // Target ~67
  else if (rawVal >= 0   && rawVal <= 25)  currentKey = KEY_RIGHT;  // Target ~0

  return currentKey;
}

void handleKeypadInput(KeyState key) {
  static unsigned long lastPressTime = 0;
  if (key == KEY_NONE || millis() - lastPressTime < 300) return; 
  lastPressTime = millis();

  if (key == KEY_SELECT) {
    // Cycle modes: Drill -> Champfer -> Tapp -> Drill
    if (currentMode == MODE_DRILL) {
      currentMode = MODE_CHAMPFER;
      sizeIndex = 3; 
    } else if (currentMode == MODE_CHAMPFER) {
      currentMode = MODE_TAPP;
      sizeIndex = 3; 
      tappState = TAPP_FWD;
      motorRunning = true;
    } else {
      currentMode = MODE_DRILL;
      sizeIndex = 3;
      motorRunning = false;
    }
  }
  else if (key == KEY_LEFT) {
    activeFocus = (activeFocus - 1 + 4) % 4; // Cycle focus: 0 -> 3 -> 2 -> 1 -> 0
  }
  else if (key == KEY_RIGHT) {
    activeFocus = (activeFocus + 1) % 4; // Cycle focus: 0 -> 1 -> 2 -> 3 -> 0
  }
  else if (key == KEY_UP) {
    if (activeFocus == 0) {
      if (currentMode == MODE_TAPP) {
        if (sizeIndex < numTappSizes - 1) sizeIndex++;
      } else {
        if (sizeIndex < numDrillSizes - 1) sizeIndex++;
      }
      // Auto-update manual RPM defaults when size changes
      if (currentMode == MODE_DRILL) manualDrillRPM = (int)calculateOptimalRPM(currentMode, drillSizes[sizeIndex]);
      else if (currentMode == MODE_CHAMPFER) manualChampferRPM = (int)calculateOptimalRPM(currentMode, drillSizes[sizeIndex]);
    } else if (activeFocus == 1) {
      // Adjust Target RPM
      if (currentMode == MODE_DRILL) manualDrillRPM += 25;
      else if (currentMode == MODE_CHAMPFER) manualChampferRPM += 25;
      else manualTappRPM += 10;
    } else if (activeFocus == 2) {
      if (coolingMode < 4) coolingMode++;
    } else if (activeFocus == 3) {
      line2Metric = (line2Metric + 1) % 3; 
    }
  }
  else if (key == KEY_DOWN) {
    if (activeFocus == 0) {
      if (sizeIndex > 0) sizeIndex--;
      if (currentMode == MODE_DRILL) manualDrillRPM = (int)calculateOptimalRPM(currentMode, drillSizes[sizeIndex]);
      else if (currentMode == MODE_CHAMPFER) manualChampferRPM = (int)calculateOptimalRPM(currentMode, drillSizes[sizeIndex]);
    } else if (activeFocus == 1) {
      // Adjust Target RPM downwards
      if (currentMode == MODE_DRILL) manualDrillRPM = max(10, manualDrillRPM - 25);
      else if (currentMode == MODE_CHAMPFER) manualChampferRPM = max(10, manualChampferRPM - 25);
      else manualTappRPM = max(10, manualTappRPM - 10);
    } else if (activeFocus == 2) {
      if (coolingMode > 0) coolingMode--;
    } else if (activeFocus == 3) {
      line2Metric = (line2Metric - 1 + 3) % 3;
    }
  }
}

/* =========================================================================
   2. SENSOR FUNCTIONS
   ========================================================================= */
float readMotorTemperature() {
  static float smoothedTemp = 17.0; 
  int rawADC = safeAnalogRead(Temp_Pin);
  if (rawADC <= 0) return smoothedTemp;
  if (rawADC >= 1023) return 300.0;

  float voltage = (rawADC / 1023.0) * 1.1;
  float rPullup = 6000.0; 
  float rSensor = rPullup * (voltage / (1.1 - voltage));
  float r17 = rPullup * ( (414.0 / 1023.0) / (1.0 - (414.0 / 1023.0)) ); 
  float rawTempC = 17.0 + ((rSensor - r17) / 9.5);

  smoothedTemp = (0.99 * smoothedTemp) + (0.01 * rawTempC);
  return smoothedTemp;
}

float readVoltage() {
  static float smoothedVoltage = 0.0;
  int rawADC = safeAnalogRead(Voltage_Pin);
  float rawVoltage = (float)rawADC * (49.9 / 565.0);
  
  if (smoothedVoltage == 0.0) smoothedVoltage = rawVoltage;
  else smoothedVoltage = (0.99 * smoothedVoltage) + (0.01 * rawVoltage);
  
  return smoothedVoltage;
}

float readCurrent() {
  static float smoothedCurrent = 0.0;
  int rawADC = safeAnalogRead(Current_Pin);
  float rawCurrent = (float)rawADC * (20.0 / 512.0); 
  
  if (smoothedCurrent == 0.0) smoothedCurrent = rawCurrent;
  else smoothedCurrent = (0.99 * smoothedCurrent) + (0.01 * rawCurrent);
  
  return smoothedCurrent;
}

float getPower() {
  return systemVoltage * systemCurrent; 
}

float calculateTorque() {
  if (spindleRPM <= 10.0) return 0.0;
  float powerWatts = getPower();
  float omega = spindleRPM * (2.0 * 3.14159 / 60.0);
  return powerWatts / omega;
}

/* =========================================================================
   3. SPINDLE RPM & PULSE READER
   ========================================================================= */
float getSpindleRPM() {
  static unsigned long lastTime = 0;
  static unsigned long pulseCount = 0;
  static int lastState = HIGH;
  static float currentRPM = 0.0;

  int currentState = digitalRead(Speed_Pin);
  if (currentState == HIGH && lastState == LOW) {
    pulseCount++;
    totalPulseCount++;
  }
  lastState = currentState;

  unsigned long currentTime = millis();
  unsigned long interval = currentTime - lastTime;
  
  if (interval >= 200) {
    currentRPM = ((float)pulseCount / 10.0) * (60000.0 / interval);
    pulseCount = 0;
    lastTime = currentTime;
  }

  return currentRPM;
}

/* =========================================================================
   4. CALCULATIONS & SPEED CONTROL
   ========================================================================= */
float calculateOptimalRPM(OperationMode mode, int diameterMM) {
  if (mode == MODE_TAPP) {
    return 120.0; 
  }
  float vc = 25.0; // HSS Co5 optimal cutting speed
  return (vc * 1000.0) / (3.14159 * (float)diameterMM);
}

float getActiveTargetRPM() {
  if (currentMode == MODE_DRILL) return (float)manualDrillRPM;
  if (currentMode == MODE_CHAMPFER) return (float)manualChampferRPM;
  return (float)manualTappRPM;
}

void setSpeed(float targetRPM, float currentRPM, bool enabled) {
  static int currentPWM = 50;
  
  if (!enabled || targetRPM <= 0) {
    currentPWM = 50;
    analogWrite(Throttle_Pin, 0);
    return;
  }

  float error = targetRPM - currentRPM;
  if (error > 5.0 && currentPWM < 255) {
    currentPWM++;
  } else if (error < -5.0 && currentPWM > 50) {
    currentPWM--;
  }

  currentPWM = constrain(currentPWM, 50, 255);
  analogWrite(Throttle_Pin, currentPWM);
}

/* =========================================================================
   5. TAPPING STATE MACHINE
   ========================================================================= */
void runTappingStateMachine() {
  if (tappState == TAPP_PAUSED) {
    setSpeed(0, spindleRPM, false);
    pinMode(Direction_Pin, INPUT); 
    return;
  }

  if (tappState == TAPP_MANUAL_REV) {
    pinMode(Direction_Pin, OUTPUT);
    digitalWrite(Direction_Pin, LOW); 
    setSpeed((float)manualTappRPM, spindleRPM, true);
    return;
  }

  switch (tappState) {
    case TAPP_FWD:
      pinMode(Direction_Pin, INPUT); 
      setSpeed((float)manualTappRPM, spindleRPM, true);
      tappPulseStartCount = totalPulseCount;
      tappPulseTarget = 10; 
      tappState = TAPP_WAIT_1;
      break;

    case TAPP_WAIT_1:
      setSpeed((float)manualTappRPM, spindleRPM, true);
      if ((int)(totalPulseCount - tappPulseStartCount) >= tappPulseTarget) {
        setSpeed(0, spindleRPM, false); 
        tappState = TAPP_REV_DIR;
      }
      break;

    case TAPP_REV_DIR:
      if (spindleRPM < 5.0) {
        pinMode(Direction_Pin, OUTPUT);
        digitalWrite(Direction_Pin, LOW); 
        tappPulseStartCount = totalPulseCount;
        tappPulseTarget = 5; 
        tappState = TAPP_BWD;
      }
      break;

    case TAPP_BWD:
      setSpeed((float)manualTappRPM, spindleRPM, true);
      if ((int)(totalPulseCount - tappPulseStartCount) >= tappPulseTarget) {
        setSpeed(0, spindleRPM, false); 
        tappState = TAPP_WAIT_2;
      }
      break;

    case TAPP_WAIT_2:
      if (spindleRPM < 5.0) {
        tappStateTimer = millis();
        tappState = TAPP_OIL;
      }
      break;

    case TAPP_OIL:
      if (millis() - tappStateTimer <= TAPPING_COOLING_DUR) {
        digitalWrite(Pump_Pin, HIGH);
      } else {
        digitalWrite(Pump_Pin, LOW);
        tappState = TAPP_FWD; 
      }
      break;

    default:
      break;
  }
}

/* =========================================================================
   6. COOLING MANAGEMENT SYSTEM
   ========================================================================= */
void updateCooling() {
  static unsigned long pumpOnTime = 0;
  static bool pumpActive = false;
  unsigned long currentMillis = millis();

  if (digitalRead(Pedal_Pin) == LOW) {
    digitalWrite(Pump_Pin, HIGH);
    pumpActive = false; 
    return;
  }

  if (currentMode == MODE_TAPP) return;

  if (coolingMode == 0) {
    digitalWrite(Pump_Pin, LOW);
    pumpActive = false;
    return;
  }

  float power = getPower();
  unsigned long baseDuration = COOLING_DUR_1;
  if (coolingMode == 2) baseDuration = COOLING_DUR_2;
  else if (coolingMode == 3) baseDuration = COOLING_DUR_3;
  else if (coolingMode == 4) baseDuration = COOLING_DUR_4;

  float scalingFactor = coolingScaling[coolingMode - 1];
  unsigned long activeDuration = (unsigned long)(baseDuration * scalingFactor * (1.0f + (power / 50.0f)));

  if (!pumpActive) {
    digitalWrite(Pump_Pin, HIGH); 
    pumpActive = true;
    pumpOnTime = currentMillis;
  } else {
    if (currentMillis - pumpOnTime >= activeDuration) {
      digitalWrite(Pump_Pin, LOW);
      pumpActive = false;
    }
  }
}

/* =========================================================================
   7. DISPLAY & BUTTON UTILITIES
   ========================================================================= */
void renderDisplayLine1() {
  lcd.setCursor(0, 0);
  
  // Mode Prefix
  if (currentMode == MODE_DRILL) lcd.print(F("D:"));
  else if (currentMode == MODE_CHAMPFER) lcd.print(F("C:"));
  else lcd.print(F("T:"));

  // Hole Size (2 digits)
  int size = (currentMode == MODE_TAPP) ? tappSizes[sizeIndex] : drillSizes[sizeIndex];
  if (size < 10) lcd.print('0');
  lcd.print(size);
  lcd.print(' ');

  // Target RPM
  int targetRPM = (int)getActiveTargetRPM();
  lcd.print(F("RPM:"));
  if (targetRPM < 1000) lcd.print(' ');
  lcd.print(targetRPM);
  lcd.print(' ');

  // Cooling Option
  lcd.print(F("C"));
  lcd.print(coolingMode);
}

void renderDisplayLine2() {
  lcd.setCursor(0, 1);
  
  // 1. Left Metric (Voltage, Power, or Measured RPM as U)
  if (line2Metric == 0) {
    lcd.print(systemVoltage, 1);
    lcd.print(F("V "));
  } else if (line2Metric == 1) {
    int pwr = (int)getPower();
    if (pwr < 100) lcd.print(' ');
    lcd.print(pwr);
    lcd.print(F("W "));
  } else {
    int rpmInt = (int)spindleRPM;
    if (rpmInt < 1000) lcd.print(' ');
    lcd.print(rpmInt);
    lcd.print(F("U "));
  }

  // 2. Torque (yyNm)
  int torqueInt = (int)calculateTorque();
  if (torqueInt < 10) lcd.print(' ');
  lcd.print(torqueInt);
  lcd.print(F("Nm "));

  // 3. Temperature (ttt°C)
  int tempInt = (int)motorTemp;
  if (tempInt < 100) lcd.print(' ');
  lcd.print(tempInt);
  lcd.print((char)223); // Degree symbol
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
      } 
      else {
        if (!longPressRegistered) {
          action = 1; // Short press
        }
      }
    }

    if (buttonState == LOW && !longPressRegistered) {
      if ((millis() - buttonDownTimestamp) >= longPressTime) {
        longPressRegistered = true;
        action = 2; // Long press
      }
    }
  }

  lastButtonState = reading;
  return action;
}