#define USE_OLED // Comment this line to use LCD instead of OLED

#ifdef USE_OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#else
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "BigNumbers_I2C.h"
#endif

#include <avr/wdt.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <PID_v1.h>

// Pin Definitions
#define TEMP_SENSOR_PIN A0
#define IRON_PIN 10
#define WS2812_PIN 12
#define BUZZ_PIN 2
#define ENCODER_CLK_PIN 3
#define ENCODER_DT_PIN 4
#define BUTTON_PIN 5
#define LED_OFF_PIN 8
#define EEPROM_TEMP_ADDRESS 0
#define EEPROM_UNIT_ADDRESS 2
#define EEPROM_SLEEP_TEMP_ADDRESS 3
#define EEPROM_SLEEP_TIME_ADDRESS 5
#define EEPROM_OFF_TIME_ADDRESS 6

// Constants
const int MIN_TEMP = 28;
const int MAX_TEMP = 500;
const int MIN_ADC = 0;
const int MAX_ADC = 290;
const int MAX_PWM = 255;
const int AVG_COUNTS = 20;
const int LCD_INTERVAL = 80;
const int MIN_KNOB = 100;
const int MAX_KNOB = 500;
const unsigned long AUTO_SHUTOFF_TIME = 10 * 60000; // 10 minutes in milliseconds

// Overshoot variable
const float MAX_OVERSHOOT = 10.0; // Maximum allowed overshoot in °C

// Adaptive PID variable
int adaptationLoopCount = 0;  // Loop counter for PID adaptation
const int ADAPT_LOOP_THRESHOLD = 225;  // Number of loops after which PID adapts
float sumError = 0;
int errorCount = 0;

// PID Variables
double Setpoint, Input, Output;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Ramping Variables
const unsigned long RAMP_DURATION = 20000; // 20 seconds for ramping
bool isRamping = false;
unsigned long rampStartTime = 0;
int originalSetpoint = 0;

// Sleep and Boost Settings
volatile int sleepTempSetting = 150;
volatile int sleepTimeSetting = 3; // minutes (0 = Disabled)
volatile int offTimeSetting = 10;  // minutes
volatile bool isSleeping = false;

bool isBoostActive = false;
unsigned long boostStartTime = 0;
const unsigned long BOOST_DURATION = 45000; // 45 seconds
const int BOOST_TEMP_OFFSET = 50; // +50°C

// Button Dispatcher variables
volatile int clickCount = 0;
volatile unsigned long lastClickTime = 0;

// Configuration Menu variables
volatile bool inMenu = false;
volatile bool editMode = false;
volatile int menuIndex = 0;

// LED Colors
const uint32_t COLOR_OFF = 0x000000;      // Black (OFF)
const uint32_t COLOR_HEATING = 0xFF0000;  // Red (Heating)
const uint32_t COLOR_READY = 0x00FF00;    // Green (Ready)
const uint32_t COLOR_COOLING = 0x0000FF;  // Blue (Cooling)
const uint32_t COLOR_WARNING = 0xFFFF00;  // Yellow (Warning)
const uint32_t COLOR_RAMPING = 0xFF00FF;  // Purple (Ramping)
const uint32_t COLOR_BOOST = 0x00FFFF;    // Cyan (Boost)

// Global Variables
#ifdef USE_OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#else
LiquidCrystal_I2C lcd(0x27, 16, 2);
BigNumbers_I2C bigNum(&lcd);
#endif

Adafruit_NeoPixel pixel(1, WS2812_PIN, NEO_GRB + NEO_KHZ800);

// Volatile state variables updated in interrupts
volatile int knob = 100;
volatile unsigned long lastActivityTime = 0;
volatile bool settingsChanged = false;
volatile unsigned long lastSettingsChangeTime = 0;

// Non-volatile state variables
int pwm = 0;
int tempRaw = 0;
int counter = 0;
int currentTempAvg = 0;
unsigned long previousMillis = 0;
float currentTemp = 0.0;
float store = 0.0;

bool ledOffState = true;
int lastButtonState = HIGH;

// Safety & Filter variables
bool sensorError = false;
bool thermalRunawayError = false;
unsigned long autoShutoffMsgStartTime = 0;
volatile bool isFahrenheit = false;

double filteredTemp = 0.0;
const float FILTER_ALPHA = 0.15; // EMA filter constant

unsigned long lastThermalCheckTime = 0;
float lastThermalTemp = 0.0;

// Function declarations to resolve forward references
void checkClicks();
void handleBoost();
void saveMenuSettings();

// Function Overloads for Buzzer
void beep() {
  tone(BUZZ_PIN, 1000, 100);
}

void beep(unsigned int duration) {
  tone(BUZZ_PIN, 1000, duration);
}

void setup() {
  initializeWatchdog();
  initializePins();
  initializeDisplay();
  initializeWS2812();
  loadSavedTemperature();
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), encoderISR, FALLING);
  lastActivityTime = millis();

  // Read initial temperature and seed filter
  readTemperature();
  filteredTemp = currentTemp;

  // Initialize PID
  Input = filteredTemp;
  Setpoint = knob;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, MAX_PWM);
}

void loop() {
  wdt_reset();

  readTemperature();
  updateTemperatureAverage();
  handleButtonPress();
  checkClicks();
  handleBoost();

  if (sensorError || thermalRunawayError) {
    pwm = 0;
    analogWrite(IRON_PIN, 0);
    ledOffState = true;
    isRamping = false;
    isSleeping = false;
    isBoostActive = false;
    inMenu = false;
    editMode = false;
  } else {
    updatePID();
    checkSafety();
  }

  controlIronAndLED();
  updateDisplay();
  checkAutoShutoff();
  handleRamping();
  adaptPIDParameters();

  // Save settings asynchronously after user stops rotating to prevent EEPROM wear
  if (settingsChanged && (millis() - lastSettingsChangeTime > 2000)) {
    saveTemperature();
    settingsChanged = false;
  }
}

void initializeWatchdog() {
  wdt_disable(); // Disable watchdog while setting it up
  wdt_enable(WDTO_4S); // Enable watchdog with 4-second timeout
}

void initializePins() {
  pinMode(TEMP_SENSOR_PIN, INPUT);
  pinMode(IRON_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_OFF_PIN, OUTPUT);
  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(ENCODER_CLK_PIN, INPUT_PULLUP);
  pinMode(ENCODER_DT_PIN, INPUT_PULLUP);
  digitalWrite(LED_OFF_PIN, HIGH);
}

void initializeDisplay() {
#ifdef USE_OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("Initializing..."));
  display.display();
#else
  lcd.init();
  lcd.backlight();
  lcd.clear();
  bigNum.begin();
  lcd.setCursor(0, 0);
  lcd.print(F("SET"));
#endif
}

void initializeWS2812() {
  pixel.begin();
  pixel.setBrightness(50); // Set to 50% brightness
  pixel.show(); // Initialize all pixels to 'off'
}

void loadSavedTemperature() {
  int savedTemp = (EEPROM.read(EEPROM_TEMP_ADDRESS) << 8) | EEPROM.read(EEPROM_TEMP_ADDRESS + 1);
  if (savedTemp >= MIN_KNOB && savedTemp <= MAX_KNOB) {
    knob = savedTemp;
  }
  
  // Load unit setting
  byte unit = EEPROM.read(EEPROM_UNIT_ADDRESS);
  isFahrenheit = (unit == 1);

  // Load Sleep Temp
  int savedSleepTemp = (EEPROM.read(EEPROM_SLEEP_TEMP_ADDRESS) << 8) | EEPROM.read(EEPROM_SLEEP_TEMP_ADDRESS + 1);
  if (savedSleepTemp >= 100 && savedSleepTemp <= 200) {
    sleepTempSetting = savedSleepTemp;
  } else {
    sleepTempSetting = 150;
  }

  // Load Sleep Time
  byte savedSleepTime = EEPROM.read(EEPROM_SLEEP_TIME_ADDRESS);
  if (savedSleepTime <= 10) {
    sleepTimeSetting = savedSleepTime;
  } else {
    sleepTimeSetting = 3;
  }

  // Load Off Time
  byte savedOffTime = EEPROM.read(EEPROM_OFF_TIME_ADDRESS);
  if (savedOffTime >= 1 && savedOffTime <= 20) {
    offTimeSetting = savedOffTime;
  } else {
    offTimeSetting = 10;
  }
}

void saveTemperature() {
  EEPROM.update(EEPROM_TEMP_ADDRESS, knob >> 8);
  EEPROM.update(EEPROM_TEMP_ADDRESS + 1, knob & 0xFF);
}

void saveMenuSettings() {
  EEPROM.update(EEPROM_UNIT_ADDRESS, isFahrenheit ? 1 : 0);
  
  EEPROM.update(EEPROM_SLEEP_TEMP_ADDRESS, sleepTempSetting >> 8);
  EEPROM.update(EEPROM_SLEEP_TEMP_ADDRESS + 1, sleepTempSetting & 0xFF);
  
  EEPROM.update(EEPROM_SLEEP_TIME_ADDRESS, sleepTimeSetting);
  EEPROM.update(EEPROM_OFF_TIME_ADDRESS, offTimeSetting);
}

void encoderISR() {
  if (sensorError || thermalRunawayError) return;
  
  if (inMenu) {
    int dir = (digitalRead(ENCODER_DT_PIN) == HIGH) ? 1 : -1;
    if (editMode) {
      switch (menuIndex) {
        case 0: // Sleep Temp
          sleepTempSetting = constrain(sleepTempSetting + (dir * 5), 100, 200);
          break;
        case 1: // Sleep Time (minutes, 0 = Disabled)
          sleepTimeSetting = constrain(sleepTimeSetting + dir, 0, 10);
          break;
        case 2: // Off Time (minutes)
          offTimeSetting = constrain(offTimeSetting + dir, 1, 20);
          // Ensure shutoff time is strictly greater than sleep time
          if (sleepTimeSetting > 0 && offTimeSetting <= sleepTimeSetting) {
            offTimeSetting = sleepTimeSetting + 1;
          }
          break;
        case 3: // Temp Unit
          isFahrenheit = !isFahrenheit;
          break;
      }
    } else {
      menuIndex += dir;
      if (menuIndex > 3) menuIndex = 0;
      else if (menuIndex < 0) menuIndex = 3;
    }
    lastActivityTime = millis();
    return;
  }
  
  if (ledOffState) return;

  if (isSleeping) {
    isSleeping = false;
    lastActivityTime = millis();
    return; // Wake up first without changing temperature
  }
  
  if (digitalRead(ENCODER_DT_PIN) == HIGH) {
    knob = constrain(knob + 10, MIN_KNOB, MAX_KNOB);
  } else {
    knob = constrain(knob - 10, MIN_KNOB, MAX_KNOB);
  }
  settingsChanged = true;
  lastSettingsChangeTime = millis();
  lastActivityTime = millis();
}

void readTemperature() {
  tempRaw = analogRead(TEMP_SENSOR_PIN);
  
  // Sensor error check: open circuit/short circuit detection
  if (tempRaw <= 2 || tempRaw >= 1021) {
    sensorError = true;
    pwm = 0;
    ledOffState = true;
    isRamping = false;
  } else {
    sensorError = false;
  }

  currentTemp = map(tempRaw, MIN_ADC, MAX_ADC, MIN_TEMP, MAX_TEMP);
  
  // Exponential Moving Average filter to smooth input
  filteredTemp = (FILTER_ALPHA * currentTemp) + ((1.0 - FILTER_ALPHA) * filteredTemp);
}

void updateTemperatureAverage() {
  if (counter < AVG_COUNTS) {
    store += currentTemp;
    counter++;
  } else {
    currentTempAvg = (store / AVG_COUNTS) - 1;
    store = 0;
    counter = 0;
  }
}

void checkSafety() {
  // Overheat protection
  if (!ledOffState && currentTemp > (MAX_TEMP + 10)) {
    thermalRunawayError = true;
    ledOffState = true;
    pwm = 0;
    digitalWrite(LED_OFF_PIN, HIGH);
  }

  // Thermal runaway baseline initialization
  if (ledOffState || thermalRunawayError || sensorError) {
    lastThermalCheckTime = millis();
    lastThermalTemp = currentTemp;
    return;
  }

  // Thermal runaway protection: check if power is high but temperature fails to rise
  if (pwm > 150 && (Setpoint - Input > 15)) {
    if (millis() - lastThermalCheckTime > 15000) { // 15 seconds evaluation window
      if (currentTemp < lastThermalTemp + 3.0) {
        thermalRunawayError = true;
        ledOffState = true;
        pwm = 0;
        digitalWrite(LED_OFF_PIN, HIGH);
      } else {
        lastThermalCheckTime = millis();
        lastThermalTemp = currentTemp;
      }
    }
  } else {
    lastThermalCheckTime = millis();
    lastThermalTemp = currentTemp;
  }
}

void adaptPIDParameters() {
  if (ledOffState || isRamping || sensorError || thermalRunawayError || isSleeping || isBoostActive) {
    sumError = 0;
    errorCount = 0;
    adaptationLoopCount = 0;
    return;
  }

  // Adapt only when close to target temperature (steady state)
  if (abs(Setpoint - Input) < 15) {
    sumError += (Setpoint - Input);
    errorCount++;
    adaptationLoopCount++;
  }

  if (adaptationLoopCount >= ADAPT_LOOP_THRESHOLD) {
    if (errorCount > 0) {
      float avgError = sumError / errorCount;
      if (abs(avgError) > 5) {
        Kp *= 1.1;
        Ki *= 1.1;
      } else if (abs(avgError) < 2) {
        Kp *= 0.95;
        Ki *= 0.95;
      }
      Kp = constrain(Kp, 0.5, 10.0);
      Ki = constrain(Ki, 0.1, 20.0);
      myPID.SetTunings(Kp, Ki, Kd);
    }
    sumError = 0;
    errorCount = 0;
    adaptationLoopCount = 0;
  }
}

void updatePID() {
  Input = filteredTemp;
  if (isRamping) {
    Setpoint = calculateRampSetpoint();
  } else if (isBoostActive) {
    Setpoint = constrain(knob + BOOST_TEMP_OFFSET, MIN_KNOB, MAX_KNOB);
  } else if (isSleeping && sleepTimeSetting > 0) {
    Setpoint = sleepTempSetting;
  } else {
    Setpoint = knob;
  }
  myPID.Compute();
  pwm = Output;
}

void controlIronAndLED() {
  if (sensorError || thermalRunawayError) {
    pwm = 0;
    analogWrite(IRON_PIN, 0);
    
    // Safety error alarm and flash pattern
    static unsigned long lastErrorToggle = 0;
    static bool errorToggleState = false;
    if (millis() - lastErrorToggle > 250) {
      errorToggleState = !errorToggleState;
      lastErrorToggle = millis();
      if (errorToggleState) {
        setLEDColor(COLOR_HEATING);
        tone(BUZZ_PIN, 1200, 100);
#ifndef USE_OLED
        lcd.backlight();
#endif
      } else {
        setLEDColor(COLOR_OFF);
        noTone(BUZZ_PIN);
#ifndef USE_OLED
        lcd.noBacklight();
#endif
      }
    }
    digitalWrite(LED_OFF_PIN, HIGH);
    return;
  }

  // Synchronize LED OFF pin with system state
  digitalWrite(LED_OFF_PIN, ledOffState ? HIGH : LOW);

  if (ledOffState && !inMenu) {
    pwm = 0;
    setLEDColor(COLOR_OFF);

#ifndef USE_OLED
    // Manage LCD backlight in OFF state (keep backlight on while shutoff message is showing)
    if (autoShutoffMsgStartTime == 0 || (millis() - autoShutoffMsgStartTime >= 2000)) {
      lcd.noBacklight();
    } else {
      lcd.backlight();
    }
#endif
  } else if (inMenu) {
    pwm = 0;
    setLEDColor(COLOR_OFF);
#ifndef USE_OLED
    lcd.backlight();
#endif
  } else if (isSleeping) {
    pulseSleepLED();
#ifndef USE_OLED
    lcd.backlight();
#endif
  } else {
    // We are active
#ifndef USE_OLED
    lcd.backlight();
#endif

    // Overshoot prevention: cut power if temperature goes above Setpoint + overshoot limit
    if (currentTemp > (Setpoint + MAX_OVERSHOOT)) {
      pwm = 0;
    }

    if (isBoostActive) {
      setLEDColor(COLOR_BOOST);
    } else if (isRamping) {
      setLEDColor(COLOR_RAMPING);
    } else if (currentTemp < Setpoint - 10) {
      setLEDColor(COLOR_HEATING);
    } else if (currentTemp >= Setpoint - 10 && currentTemp <= Setpoint + 10) {
      setLEDColor(COLOR_READY);
    } else if (currentTemp > Setpoint + 10) {
      setLEDColor(COLOR_COOLING);
    }
  }

  // Warning for very high temperatures
  if (currentTemp > MAX_TEMP - 50) {
    setLEDColor(COLOR_WARNING);
  }

  analogWrite(IRON_PIN, pwm);
}

void setLEDColor(uint32_t color) {
  pixel.setPixelColor(0, color);
  pixel.show();
}

void updateDisplay() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= LCD_INTERVAL) {
    previousMillis = currentMillis;

#ifdef USE_OLED
    if (inMenu) {
      display.clearDisplay();
      
      // Settings header title bar
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println(F("SETTINGS"));
      display.drawFastHLine(0, 9, 128, SSD1306_WHITE);
      
      // Large setting name
      display.setTextSize(2);
      display.setCursor(0, 15);
      switch (menuIndex) {
        case 0: display.print(F("Sleep Temp")); break;
        case 1: display.print(F("Sleep Time")); break;
        case 2: display.print(F("Off Time")); break;
        case 3: display.print(F("Temp Unit")); break;
      }
      
      // Large setting value
      display.setTextSize(3);
      display.setCursor(8, 38);
      
      if (editMode) {
        display.print(F("["));
      } else {
        display.print(F(" "));
      }
      
      switch (menuIndex) {
        case 0:
          {
            int temp = isFahrenheit ? (int)(sleepTempSetting * 1.8 + 32) : sleepTempSetting;
            display.print(temp);
            display.print(isFahrenheit ? F("F") : F("C"));
          }
          break;
        case 1:
          if (sleepTimeSetting == 0) display.print(F("OFF"));
          else { display.print(sleepTimeSetting); display.print(F("m")); }
          break;
        case 2:
          display.print(offTimeSetting);
          display.print(F("m"));
          break;
        case 3:
          display.print(isFahrenheit ? F("F") : F("C"));
          break;
      }
      
      if (editMode) {
        display.print(F("]"));
      }
      
      display.display();
      return;
    }

    display.clearDisplay();
    
    if (sensorError) {
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.println(F("SENSOR"));
      display.println(F("ERROR"));
      display.setTextSize(1);
      display.println(F("Check sensor wire"));
      display.display();
      return;
    }
    
    if (thermalRunawayError) {
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.println(F("THERMAL"));
      display.println(F("RUNAWAY"));
      display.setTextSize(1);
      display.println(F("System stopped"));
      display.display();
      return;
    }

    if (autoShutoffMsgStartTime != 0 && millis() - autoShutoffMsgStartTime < 2000) {
      display.setTextSize(2);
      display.setCursor(0,0);
      display.println(F("Auto Shut-off"));
      display.println(F("Activated"));
      display.display();
      return;
    }

    display.setTextSize(2);
    display.setCursor(0,0);
    if (isRamping) {
      display.print(F("RAMP "));
      int remainingTime = (RAMP_DURATION - (currentMillis - rampStartTime)) / 1000;
      if (remainingTime < 0) remainingTime = 0;
      display.setCursor(74, 0);
      display.print(remainingTime);
      display.print(F("s"));
    } else if (isBoostActive) {
      display.print(F("BOOST"));
      int remainingTime = (BOOST_DURATION - (currentMillis - boostStartTime)) / 1000;
      if (remainingTime < 0) remainingTime = 0;
      display.setCursor(74, 0);
      display.print(remainingTime);
      display.print(F("s"));
    } else if (isSleeping) {
      display.print(F("SLEEP"));
      display.setCursor(74, 0);
      int targetTemp = isFahrenheit ? (int)(sleepTempSetting * 1.8 + 32) : sleepTempSetting;
      display.print(targetTemp);
      display.setTextSize(1);
      display.print((char)247);
      display.print(isFahrenheit ? F("F") : F("C"));
    } else {
      display.print(ledOffState ? F("OFF  ") : F("SET  "));
      display.setCursor(74, 0);
      int targetTemp = isFahrenheit ? (int)(knob * 1.8 + 32) : knob;
      display.print(ledOffState ? F("---") : String(targetTemp));
      display.setTextSize(1);
      display.print((char)247);
      display.print(isFahrenheit ? F("F") : F("C"));
    }

    display.setTextSize(4);
    display.setCursor(0, 28);
    int actualTempDisp = isFahrenheit ? (int)(currentTempAvg * 1.8 + 32) : currentTempAvg;
    display.print(actualTempDisp);
    display.setTextSize(3);
    display.print((char)247);
    display.print(isFahrenheit ? F("F") : F("C"));

    // Draw power bar at the bottom when active
    if (!ledOffState && !sensorError && !thermalRunawayError) {
      display.drawRect(0, 58, 128, 6, SSD1306_WHITE);
      int barWidth = map(pwm, 0, MAX_PWM, 0, 126);
      display.fillRect(1, 59, barWidth, 4, SSD1306_WHITE);
    }

    display.display();
#else
    static bool lastWasError = false;
    
    if (inMenu) {
      if (!lastWasError) { lcd.clear(); lastWasError = true; }
      lcd.setCursor(0, 0);
      switch (menuIndex) {
        case 0:
          lcd.print(F("1. Sleep Temp   "));
          lcd.setCursor(0, 1);
          {
            int temp = isFahrenheit ? (int)(sleepTempSetting * 1.8 + 32) : sleepTempSetting;
            if (editMode) {
              lcd.print(F("Edit: ["));
              lcd.print(temp);
              lcd.print(isFahrenheit ? F("F]   ") : F("C]   "));
            } else {
              lcd.print(F("Value: "));
              lcd.print(temp);
              lcd.print(isFahrenheit ? F("F    ") : F("C    "));
            }
          }
          break;
        case 1:
          lcd.print(F("2. Sleep Time   "));
          lcd.setCursor(0, 1);
          if (sleepTimeSetting == 0) {
            if (editMode) lcd.print(F("Edit: [Disabled]"));
            else lcd.print(F("Value: Disabled "));
          } else {
            if (editMode) {
              lcd.print(F("Edit: ["));
              lcd.print(sleepTimeSetting);
              lcd.print(F("m]     "));
            } else {
              lcd.print(F("Value: "));
              lcd.print(sleepTimeSetting);
              lcd.print(F("m      "));
            }
          }
          break;
        case 2:
          lcd.print(F("3. Off Time     "));
          lcd.setCursor(0, 1);
          if (editMode) {
            lcd.print(F("Edit: ["));
            lcd.print(offTimeSetting);
            lcd.print(F("m]     "));
          } else {
            lcd.print(F("Value: "));
            lcd.print(offTimeSetting);
            lcd.print(F("m      "));
          }
          break;
        case 3:
          lcd.print(F("4. Temp Unit    "));
          lcd.setCursor(0, 1);
          if (editMode) {
            lcd.print(isFahrenheit ? F("Edit: [F]       ") : F("Edit: [C]       "));
          } else {
            lcd.print(isFahrenheit ? F("Value: Fahrenheit") : F("Value: Celsius   "));
          }
          break;
      }
      return;
    }

    if (sensorError) {
      if (!lastWasError) { lcd.clear(); lastWasError = true; }
      lcd.setCursor(0, 0);
      lcd.print(F("SENSOR ERROR    "));
      lcd.setCursor(0, 1);
      lcd.print(F("Check connection"));
      return;
    }
    
    if (thermalRunawayError) {
      if (!lastWasError) { lcd.clear(); lastWasError = true; }
      lcd.setCursor(0, 0);
      lcd.print(F("THERMAL ERROR   "));
      lcd.setCursor(0, 1);
      lcd.print(F("System Stopped  "));
      return;
    }

    if (lastWasError) {
      lcd.clear();
      lastWasError = false;
    }

    if (autoShutoffMsgStartTime != 0 && millis() - autoShutoffMsgStartTime < 2000) {
      lcd.setCursor(0, 0);
      lcd.print(F("Auto Shut-off   "));
      lcd.setCursor(0, 1);
      lcd.print(F("Activated       "));
      return;
    }

    lcd.setCursor(0, 0);
    if (isRamping) {
      lcd.print(F("RAMP "));
      int remainingTime = (RAMP_DURATION - (currentMillis - rampStartTime)) / 1000;
      if (remainingTime < 0) remainingTime = 0;
      lcd.setCursor(0, 1);
      lcd.print(remainingTime);
      lcd.print(F("s   "));
    } else {
      if (ledOffState) {
        lcd.print(F("OFF "));
        lcd.setCursor(0, 1);
        lcd.print(F("--- "));
      } else if (isSleeping) {
        lcd.print(F("SLP "));
        lcd.setCursor(0, 1);
        int targetTemp = isFahrenheit ? (int)(sleepTempSetting * 1.8 + 32) : sleepTempSetting;
        lcd.print(F("S"));
        lcd.print(targetTemp);
        if (targetTemp < 100) lcd.print(F("  "));
        else if (targetTemp < 1000) lcd.print(F(" "));
      } else if (isBoostActive) {
        lcd.print(F("BST "));
        lcd.setCursor(0, 1);
        int remainingTime = (BOOST_DURATION - (currentMillis - boostStartTime)) / 1000;
        if (remainingTime < 0) remainingTime = 0;
        lcd.print(F("B"));
        lcd.print(remainingTime);
        lcd.print(F("s "));
      } else {
        int targetTemp = isFahrenheit ? (int)(knob * 1.8 + 32) : knob;
        lcd.print(F("S"));
        lcd.print(targetTemp);
        if (targetTemp < 100) lcd.print(F("  "));
        else if (targetTemp < 1000) lcd.print(F(" "));
        
        lcd.setCursor(0, 1);
        int powerPercent = (pwm * 100) / MAX_PWM;
        lcd.print(F("P"));
        lcd.print(powerPercent);
        lcd.print(F("%"));
        if (powerPercent < 10) lcd.print(F("   "));
        else if (powerPercent < 100) lcd.print(F("  "));
        else lcd.print(F(" "));
      }
    }

    int actualTempDisp = isFahrenheit ? (int)(currentTempAvg * 1.8 + 32) : currentTempAvg;
    bigNum.displayLargeInt(actualTempDisp, 6, 0, 3, false);
    lcd.setCursor(15, 0);
    lcd.print((char)223); // Degree symbol
    lcd.setCursor(15, 1);
    lcd.print(isFahrenheit ? "F" : "C");
#endif
  }
}

void handleButtonPress() {
  int reading = digitalRead(BUTTON_PIN);
  static unsigned long lastDebounceTime = 0;
  static int lastStableState = HIGH;
  static unsigned long buttonPressTime = 0;
  static bool longPressTriggered = false;

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > 50) {
    if (reading != lastStableState) {
      lastStableState = reading;
      if (lastStableState == LOW) {
        buttonPressTime = millis();
        longPressTriggered = false;
      } else {
        // Button released
        if (!longPressTriggered) {
          // Short press: Register click
          clickCount++;
          lastClickTime = millis();
        }
        lastActivityTime = millis();
      }
    }
  }
  lastButtonState = reading;

  // Active check for long press while button is held down
  if (lastStableState == LOW && !longPressTriggered) {
    if (millis() - buttonPressTime >= 1500) {
      longPressTriggered = true;
      // Toggle menu activation
      inMenu = !inMenu;
      if (inMenu) {
        editMode = false;
        menuIndex = 0;
        pwm = 0;
        analogWrite(IRON_PIN, 0);
        isSleeping = false;
        isBoostActive = false;
        isRamping = false;
      } else {
        editMode = false; // Always clear edit mode on exit
        saveMenuSettings();
      }
      beep(300); // Long beep indicator
      lastActivityTime = millis();
    }
  }
}

void startRamping() {
  isRamping = true;
  rampStartTime = millis();
  originalSetpoint = knob;
  beep(150);
}

int calculateRampSetpoint() {
  unsigned long elapsedTime = millis() - rampStartTime;
  if (elapsedTime > RAMP_DURATION) {
    isRamping = false;
    return originalSetpoint;
  }

  int rampTarget = MAX_TEMP;
  float progress = (float)elapsedTime / RAMP_DURATION;
  return originalSetpoint + (rampTarget - originalSetpoint) * progress;
}

void handleRamping() {
  if (isRamping && millis() - rampStartTime > RAMP_DURATION) {
    isRamping = false;
    beep(300); // Long beep to alert user that ramping completed
  }
}

void checkAutoShutoff() {
  if (ledOffState) return;

  unsigned long idleTime = millis() - lastActivityTime;

  // Inactivity Sleep trigger (only if not in menu)
  if (sleepTimeSetting > 0 && !inMenu) {
    unsigned long sleepTimeoutVal = (unsigned long)sleepTimeSetting * 60000;
    if (idleTime > sleepTimeoutVal) {
      if (!isSleeping && !isRamping && !isBoostActive) {
        isSleeping = true;
        beep(150);
      }
    } else {
      isSleeping = false;
    }
  } else {
    isSleeping = false;
  }

  // Auto shutoff after offTimeSetting minutes total inactivity (applies in menu too)
  unsigned long shutoffTimeoutVal = (unsigned long)offTimeSetting * 60000;
  if (idleTime > shutoffTimeoutVal) {
    ledOffState = true;
    isSleeping = false;
    isBoostActive = false;
    inMenu = false;
    editMode = false;
    digitalWrite(LED_OFF_PIN, HIGH);
    setLEDColor(COLOR_OFF);
    beep(200);
    autoShutoffMsgStartTime = millis();
    lastActivityTime = millis(); // Reset activity timer
  }
}

void pulseSleepLED() {
  static unsigned long lastPulseTime = 0;
  static int brightness = 0;
  static bool fadingUp = true;
  
  if (millis() - lastPulseTime > 15) {
    lastPulseTime = millis();
    if (fadingUp) {
      brightness += 2;
      if (brightness >= 80) fadingUp = false;
    } else {
      brightness -= 2;
      if (brightness <= 5) fadingUp = true;
    }
    pixel.setPixelColor(0, pixel.Color(0, 0, brightness)); // Pulse Blue
    pixel.show();
  }
}

void handleBoost() {
  if (isBoostActive && (millis() - boostStartTime > BOOST_DURATION)) {
    isBoostActive = false;
    beep(300);
  }
}



void checkClicks() {
  if (clickCount > 0 && (millis() - lastClickTime > 250)) {
    if (inMenu) {
      if (clickCount >= 1) {
        editMode = !editMode;
        beep(100);
      }
      clickCount = 0;
      return;
    }

    if (clickCount == 1) {
      if (isSleeping) {
        isSleeping = false;
        lastActivityTime = millis();
        beep(100);
      } else if (sensorError || thermalRunawayError) {
        sensorError = false;
        thermalRunawayError = false;
        lastThermalCheckTime = millis();
        lastThermalTemp = currentTemp;
        beep(200);
      } else if (ledOffState) {
        ledOffState = false;
        digitalWrite(LED_OFF_PIN, LOW);
        beep(100);
      } else if (isBoostActive) {
        isBoostActive = false;
        beep(50);
      } else if (!isRamping) {
        startRamping();
      } else {
        ledOffState = true;
        isRamping = false;
        digitalWrite(LED_OFF_PIN, HIGH);
        beep(50);
      }
    } else if (clickCount >= 2) {
      if (!ledOffState && !sensorError && !thermalRunawayError && !isSleeping) {
        isBoostActive = !isBoostActive;
        boostStartTime = millis();
        beep(250);
      }
    }
    clickCount = 0;
  }
}
