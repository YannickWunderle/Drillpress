#include <hd44780.h>
#include <hd44780ioClass/hd44780_pinIO.h> // Arduino pin i/o class header

#define Temp_Pin A2
#define Cruise_Pin 1
#define Direction_Pin 2
#define Throttle_Pin 11
#define Speed_Pin A4
#define Current_Pin A3
#define Pump_Pin 3
#define LED_Pin 12
#define Button_Pin 13
#define Pedal_Pin A5
#define Voltage_Pin A1
#define Keypad_Pin A0

#if defined (ARDUINO_ARCH_ESP8266)
const int rs=D8, en=D9, db4=D4, db5=D5, db6=D6, db7=D7; // for esp8266 devices
#else
const int rs=8, en=9, db4=4, db5=5, db6=6, db7=7;       // for all other devices
#endif
hd44780_pinIO lcd(rs, en, db4, db5, db6, db7);

const int LCD_COLS = 16;
const int LCD_ROWS = 2;

// Keypad Button identifiers
enum KeyState { KEY_NONE, KEY_SELECT, KEY_LEFT, KEY_UP, KEY_DOWN, KEY_RIGHT };

// Button tracking variables (for millis-based debouncing & long press)
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; 
const unsigned long longPressTime = 1000; // 1 second threshold for long press
int lastButtonState = HIGH;
int buttonState = HIGH;
bool longPressRegistered = false;

void setup() {
  Serial.begin(9600);
  lcd.begin(LCD_COLS, LCD_ROWS);
  analogReference(INTERNAL); // Set internal 1.1V reference

  // Configure button and pedal pins with internal pull-up resistors
  pinMode(Button_Pin, INPUT_PULLUP);
  pinMode(Pedal_Pin, INPUT_PULLUP);
  pinMode(Speed_Pin, INPUT);
  pinMode(Temp_Pin, INPUT);
  pinMode(Current_Pin, INPUT);
  pinMode(Voltage_Pin, INPUT);
  pinMode(Keypad_Pin, INPUT);

  // Configure direction pin as floating input (INPUT mode)
  pinMode(Direction_Pin, INPUT);

  // Configure control and indicator outputs
  pinMode(Pump_Pin, OUTPUT);
  pinMode(LED_Pin, OUTPUT);
  pinMode(Throttle_Pin, OUTPUT);
  digitalWrite(Pump_Pin, LOW);
  digitalWrite(LED_Pin, LOW);
  digitalWrite(Throttle_Pin, LOW);
}

void loop() {
  // 1. Read Keypad
  KeyState currentKey = readKeypad();
  
  // 2. Read Motor Temperature
  float motorTemp = readMotorTemperature();

  // 3. Check Button Press (returns 0: None, 1: Short Press, 2: Long Press)
  int btnAction = checkButton();

  // Debug output via Serial every 500ms (using millis instead of delay)
  static unsigned long lastDebugPrint = 0;
  if (millis() - lastDebugPrint >= 500) {
    lastDebugPrint = millis();
    Serial.print(F("Key: ")); Serial.print(currentKey);
    Serial.print(F(" | Temp: ")); Serial.print(motorTemp); Serial.print(F(" C"));
    if (btnAction == 1) Serial.print(F(" | [Button: SHORT PRESS]"));
    if (btnAction == 2) Serial.print(F(" | [Button: LONG PRESS]"));
    Serial.println();
  }

  // Sample LCD output loop
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(motorTemp, 1);
  lcd.print((char)223); // Degree symbol
  lcd.print("C   ");
  
  lcd.setCursor(0, 1);
  lcd.print("Key: ");
  lcd.print(currentKey);
  lcd.print("     ");
}

/* =========================================================================
   1. KEYPAD READER FUNCTION
   Maps raw ADC values to defined keys using a +/- 15 tolerance window.
   ========================================================================= */
KeyState readKeypad() {
  int rawVal = analogRead(Keypad_Pin);

  if (rawVal >= 452 && rawVal <= 482) return KEY_NONE;   // Target ~467
  if (rawVal >= 291 && rawVal <= 321) return KEY_SELECT; // Target ~306
  if (rawVal >= 187 && rawVal <= 217) return KEY_LEFT;   // Target ~202
  if (rawVal >= 32  && rawVal <= 62)  return KEY_UP;     // Target ~47
  if (rawVal >= 113 && rawVal <= 143) return KEY_DOWN;   // Target ~128
  if (rawVal >= 0   && rawVal <= 15)  return KEY_RIGHT;  // Target ~0

  return KEY_NONE; // Fallback default
}

/* =========================================================================
   2. MOTOR TEMPERATURE FUNCTION (KTY84-130 + 6k Pull-up to 1.1V Ref)
   Calculates temperature using ADC counts, a 6k pull-up resistor, 
   and the calibration anchor point (414 counts at 17°C).
   ========================================================================= */
float readMotorTemperature() {
  int rawADC = analogRead(Temp_Pin);
  
  // Prevent division by zero if ADC hits absolute max/min bounds
  if (rawADC <= 0) return -40.0;
  if (rawADC >= 1023) return 300.0;

  // Convert ADC reading to Voltage (using 1.1V internal reference)
  float voltage = (rawADC / 1023.0) * 1.1;

  // Calculate NTC/PTC sensor resistance via Voltage Divider formula:
  // V_out = V_ref * (R_sensor / (R_pullup + R_sensor))
  // Solving for R_sensor: R_sensor = R_pullup * (V_out / (V_ref - V_out))
  float rPullup = 6000.0; // 6k ohm resistor
  float rSensor = rPullup * (voltage / (1.1 - voltage));

  // KTY84-130 characteristics: positive temperature coefficient (PTC).
  // Using the calibration anchor: at 17°C, ADC = 414 counts -> ~3970 ohms.
  // Approximation coefficient for standard KTY84 silicon slope (~0.77% per °C)
  float r17 = rPullup * ( (414.0 / 1023.0) / (1.0 - (414.0 / 1023.0)) ); 
  
  // Linearized delta approximation relative to 17°C baseline 
  // (KTY84 average temperature coefficient around room temp is roughly 9.5 ohms/°C)
  float tempC = 17.0 + ((rSensor - r17) / 9.5);

  return tempC;
}

/* =========================================================================
   3. DEBOUNCED BUTTON FUNCTION (Short & Long Press using millis)
   Returns: 0 = No action, 1 = Short Press, 2 = Long Press
   ========================================================================= */
int checkButton() {
  int reading = digitalRead(Button_Pin);
  int action = 0; // Default: no action

  // Check if button state changed (due to noise or press)
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  // If the stable period has passed past the debounce threshold
  if ((millis() - lastDebounceTime) > debounceDelay) {
    
    // If state actually changed
    if (reading != buttonState) {
      buttonState = reading;

      // Button just pressed (LOW because of INPUT_PULLUP)
      if (buttonState == LOW) {
        longPressRegistered = false;
      } 
      // Button just released
      else {
        if (!longPressRegistered) {
          action = 1; // Short press detected on release
        }
      }
    }

    // Check for long press while button is held down continuously
    if (buttonState == LOW && !longPressRegistered) {
      if ((millis() - lastDebounceTime) >= longPressTime) {
        longPressRegistered = true;
        action = 2; // Long press detected
      }
    }
  }

  lastButtonState = reading;
  return action;
}