// =============================================================
//  Server Room Fan Controller
//  Board  : Arduino Mega 2560
//  Sensor : TMP36  (analog)
//  Sensor : HC-SR04 (digital)
//  Display: I2C LCD  – LED 8050 R7  (address 0x27, change if needed)
//  Fan    : 5 V PWM fan
// =============================================================
//  Required libraries (install via Library Manager):
//    • LiquidCrystal_I2C  by Frank de Brabander
// =============================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ─── PIN CONFIGURATION ────────────────────────────────────────
#define PIN_TMP36      A0   // TMP36 analog output
#define PIN_TRIG        3   // HC-SR04 trigger
#define PIN_ECHO        4   // HC-SR04 echo
#define PIN_FAN         9   // PWM-capable pin (Mega: 2,3,4,5,6,7,8,9,10,11,12,13,44,45,46)

// ─── LCD CONFIGURATION ────────────────────────────────────────
// Change 0x27 to 0x3F if the display stays blank after upload
#define LCD_I2C_ADDR   0x27
#define LCD_COLS        16
#define LCD_ROWS         2

// ─── TEMPERATURE THRESHOLDS (°C) ──────────────────────────────
#define TEMP_FAN_ON     30.0   // Fan starts above this
#define TEMP_FAN_OFF    28.0   // Hysteresis: fan stops below this (must be < TEMP_FAN_ON)
#define TEMP_MIN_MAP    20.0   // Temp at which PWM = FAN_PWM_MIN
#define TEMP_MAX_MAP    40.0   // Temp at which PWM = 255

// ─── FAN PWM LIMITS ───────────────────────────────────────────
#define FAN_PWM_MIN    100     // Minimum to reliably spin up (raise if fan stutters)
#define FAN_PWM_MAX    255

// ─── PERSON DETECTION ─────────────────────────────────────────
#define PERSON_DIST_CM  50     // Fan off when someone is closer than this
#define SONAR_MAX_CM   500     // HC-SR04 reliable range limit
#define SONAR_TIMEOUT_US 40000UL  // ~4 m round-trip; no response → failsafe

// ─── TIMING INTERVALS (ms) ────────────────────────────────────
#define INTERVAL_TEMP   500    // How often to read temperature
#define INTERVAL_SONAR  200    // How often to ping the sonar
#define INTERVAL_LCD   1000    // How often to refresh the display

// ─── MOVING AVERAGE ───────────────────────────────────────────
#define AVG_SAMPLES     10     // Number of temperature readings to smooth

// =============================================================
//  DATA STRUCTURES
// =============================================================

struct SensorData {
  float   tempC;          // Smoothed temperature in °C
  int     distanceCm;     // Last measured distance (-1 = error/timeout)
  bool    personNearby;   // true when distance < PERSON_DIST_CM
};

struct FanState {
  bool  running;          // Is the fan currently supposed to run?
  int   pwmValue;         // Current PWM value written to the pin (0-255)
  int   percentSpeed;     // PWM as 0-100 % (for display)
};

SensorData sensor = { 25.0, 200, false };
FanState   fan    = { false, 0, 0 };

// ─── Internal: moving average buffer ──────────────────────────
float   tempBuffer[AVG_SAMPLES];
uint8_t tempIndex     = 0;
bool    bufferFull    = false;

// ─── Internal: non-blocking timers ────────────────────────────
unsigned long lastTempMs  = 0;
unsigned long lastSonarMs = 0;
unsigned long lastLcdMs   = 0;

// ─── Internal: hysteresis state ───────────────────────────────
bool fanEnabledByTemp = false;  // latched on/off with hysteresis

// =============================================================
//  LCD OBJECT
// =============================================================
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

// =============================================================
//  FUNCTION DECLARATIONS
// =============================================================
float   readTemperatureC();
float   smoothedTemperature(float newReading);
int     readDistanceCm();
int     calculatePWM(float tempC);
void    updateFan(SensorData &s, FanState &f);
void    updateDisplay(const SensorData &s, const FanState &f);
void    printPadded(int val, int width);

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(9600);
  Serial.println(F("=== Server Room Fan Controller ==="));

  // Pin modes
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_FAN,  OUTPUT);
  analogWrite(PIN_FAN, 0);   // Fan off at startup

  // Pre-fill moving average buffer with a first real reading
  float firstRead = readTemperatureC();
  for (uint8_t i = 0; i < AVG_SAMPLES; i++) {
    tempBuffer[i] = firstRead;
  }
  sensor.tempC = firstRead;

  // LCD init
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Server Room Fan"));
  lcd.setCursor(0, 1);
  lcd.print(F("  Initializing.."));
  delay(1500);
  lcd.clear();
}

// =============================================================
//  LOOP  –  fully non-blocking (millis-based)
// =============================================================
void loop() {
  unsigned long now = millis();

  // ── 1. Temperature reading ──────────────────────────────────
  if (now - lastTempMs >= INTERVAL_TEMP) {
    lastTempMs = now;
    float raw    = readTemperatureC();
    sensor.tempC = smoothedTemperature(raw);
  }

  // ── 2. Sonar / person detection ────────────────────────────
  if (now - lastSonarMs >= INTERVAL_SONAR) {
    lastSonarMs = now;
    sensor.distanceCm  = readDistanceCm();
    sensor.personNearby = (sensor.distanceCm > 0 &&
                           sensor.distanceCm < PERSON_DIST_CM);
  }

  // ── 3. Fan control logic ────────────────────────────────────
  updateFan(sensor, fan);

  // ── 4. Display update ───────────────────────────────────────
  if (now - lastLcdMs >= INTERVAL_LCD) {
    lastLcdMs = now;
    updateDisplay(sensor, fan);
  }
}

// =============================================================
//  TEMPERATURE  –  TMP36
// =============================================================
// TMP36 formula: Temp(°C) = (mV - 500) / 10
// At 5 V reference: mV = analogRead * (5000.0 / 1024.0)
float readTemperatureC() {
  int   raw = analogRead(PIN_TMP36);
  float mV  = raw * (5000.0 / 1024.0);
  float tempC = (mV - 500.0) / 10.0;
  return tempC;
}

// Rolling moving average (returns smoothed value)
float smoothedTemperature(float newReading) {
  tempBuffer[tempIndex] = newReading;
  tempIndex = (tempIndex + 1) % AVG_SAMPLES;
  if (tempIndex == 0) bufferFull = true;

  uint8_t count = bufferFull ? AVG_SAMPLES : tempIndex;
  float   sum   = 0.0;
  for (uint8_t i = 0; i < count; i++) sum += tempBuffer[i];
  return sum / count;
}

// =============================================================
//  DISTANCE  –  HC-SR04
// =============================================================
// Returns distance in cm, or -1 on timeout / out-of-range
int readDistanceCm() {
  // Send 10 µs trigger pulse
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // Measure echo pulse duration with timeout
  long duration = pulseIn(PIN_ECHO, HIGH, SONAR_TIMEOUT_US);

  if (duration == 0) {
    // Timeout → sensor disconnected or object too far
    return -1;
  }

  int cm = (int)(duration / 58);   // speed of sound: ~340 m/s → /58 gives cm
  if (cm > SONAR_MAX_CM) return -1;
  return cm;
}

// =============================================================
//  FAN CONTROL
//  Combines temperature (PWM-mapped) + hysteresis + person check
//  + failsafe: sensor error → full speed to protect servers
// =============================================================
int calculatePWM(float tempC) {
  // map() works with integers; scale ×10 to keep one decimal of precision
  long pwm = map(
    (long)(tempC  * 10),
    (long)(TEMP_MIN_MAP * 10),
    (long)(TEMP_MAX_MAP * 10),
    FAN_PWM_MIN,
    FAN_PWM_MAX
  );
  return (int)constrain(pwm, FAN_PWM_MIN, FAN_PWM_MAX);
}

void updateFan(SensorData &s, FanState &f) {

  // ── Failsafe: sonar timeout (-1) → full speed ───────────────
  if (s.distanceCm == -1) {
    f.running      = true;
    f.pwmValue     = FAN_PWM_MAX;
    f.percentSpeed = 100;
    analogWrite(PIN_FAN, f.pwmValue);
    Serial.println(F("[FAILSAFE] Sonar error – fan at 100%"));
    return;
  }

  // ── Person nearby → force fan off ──────────────────────────
  if (s.personNearby) {
    f.running      = false;
    f.pwmValue     = 0;
    f.percentSpeed = 0;
    analogWrite(PIN_FAN, 0);
    return;
  }

  // ── Hysteresis for temperature threshold ────────────────────
  if (!fanEnabledByTemp && s.tempC >= TEMP_FAN_ON)  fanEnabledByTemp = true;
  if ( fanEnabledByTemp && s.tempC <  TEMP_FAN_OFF) fanEnabledByTemp = false;

  if (fanEnabledByTemp) {
    f.running      = true;
    f.pwmValue     = calculatePWM(s.tempC);
    f.percentSpeed = map(f.pwmValue, 0, FAN_PWM_MAX, 0, 100);
    analogWrite(PIN_FAN, f.pwmValue);
  } else {
    f.running      = false;
    f.pwmValue     = 0;
    f.percentSpeed = 0;
    analogWrite(PIN_FAN, 0);
  }
}

// =============================================================
//  LCD DISPLAY  –  16×2 layout
//
//  Row 0:  "T:  25.3C  F: 65%"
//  Row 1:  "Dist: 120cm  OK "   or "PERSON DETECTED!"
//           (failsafe) "SENSOR ERROR!  "
// =============================================================
void updateDisplay(const SensorData &s, const FanState &f) {
  // ── Row 0: temperature + fan speed ─────────────────────────
  lcd.setCursor(0, 0);
  lcd.print(F("T:"));
  // Temperature with one decimal, right-padded to 5 chars
  lcd.print(s.tempC, 1);
  lcd.print(F("C  F:"));
  printPadded(f.percentSpeed, 3);
  lcd.print(F("%"));

  // ── Row 1: distance / status ────────────────────────────────
  lcd.setCursor(0, 1);
  if (s.distanceCm == -1) {
    lcd.print(F("SENSOR ERROR!   "));
  } else if (s.personNearby) {
    lcd.print(F("PERSON DETECTED!"));
  } else {
    lcd.print(F("Dist:"));
    printPadded(s.distanceCm, 4);
    lcd.print(F("cm      "));
  }

  // ── Serial debug (open Serial Monitor at 9600 baud) ─────────
  Serial.print(F("Temp: "));    Serial.print(s.tempC, 1);
  Serial.print(F("°C  Fan: ")); Serial.print(f.percentSpeed);
  Serial.print(F("%  Dist: ")); Serial.print(s.distanceCm);
  Serial.print(F("cm  Person: ")); Serial.println(s.personNearby ? F("YES") : F("NO"));
}

// Helper: print integer right-aligned in a fixed-width field,
//         padding with spaces on the left to overwrite stale digits
void printPadded(int val, int width) {
  String s = String(val);
  for (int i = s.length(); i < width; i++) lcd.print(' ');
  lcd.print(s);
}
