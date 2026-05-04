// =============================================================
//  Server Room Fan Controller
//  Board  : Arduino Mega 2560
//  Sensor : DHT11  (temp + humidity, digital)
//  Sensor : HC-SR04 (digital)
//  Display: I2C LCD  – LED 8050 R7  (address 0x27, change if needed)
//  Fan    : 5 V PWM fan
// =============================================================
//  Required libraries (install via Library Manager):
//    • DHT sensor library  by Adafruit
//    • Adafruit Unified Sensor  by Adafruit  (dependency of above)
//    • LiquidCrystal_I2C  by Frank de Brabander
// =============================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// ─── PIN CONFIGURATION ────────────────────────────────────────
#define PIN_DHT         2   // DHT11 data pin (any digital pin)
#define PIN_TRIG        3   // HC-SR04 trigger
#define PIN_ECHO        4   // HC-SR04 echo
#define PIN_FAN         9   // PWM-capable pin

// ─── DHT SENSOR TYPE ──────────────────────────────────────────
#define DHT_TYPE       DHT11   // Change to DHT22 if you upgrade later

// ─── LCD CONFIGURATION ────────────────────────────────────────
// Change 0x27 to 0x3F if the display stays blank after upload
#define LCD_I2C_ADDR   0x27
#define LCD_COLS        16
#define LCD_ROWS         2

// ─── TEMPERATURE THRESHOLDS (°C) ──────────────────────────────
#define TEMP_FAN_ON     30.0   // Fan starts above this
#define TEMP_FAN_OFF    28.0   // Hysteresis: fan stops below this
#define TEMP_MIN_MAP    20.0   // Temp at which PWM = FAN_PWM_MIN
#define TEMP_MAX_MAP    40.0   // Temp at which PWM = 255

// ─── FAN PWM LIMITS ───────────────────────────────────────────
#define FAN_PWM_MIN    100     // Raise if fan stutters at startup
#define FAN_PWM_MAX    255

// ─── PERSON DETECTION ─────────────────────────────────────────
#define PERSON_DIST_CM  50     // Fan off when someone is closer than this
#define SONAR_MAX_CM   500
#define SONAR_TIMEOUT_US 40000UL  // ~6.8 m round-trip ceiling

// ─── TIMING INTERVALS (ms) ────────────────────────────────────
#define INTERVAL_DHT   2000    // DHT11 minimum read interval is 1-2 s
#define INTERVAL_SONAR  200
#define INTERVAL_LCD   1000

// ─── MOVING AVERAGE ───────────────────────────────────────────
#define AVG_SAMPLES     5      // DHT11 is slow, so 5 samples is enough

// =============================================================
//  DATA STRUCTURES
// =============================================================

struct SensorData {
  float   tempC;          // Smoothed temperature in °C
  float   humidity;       // Last humidity reading in %
  int     distanceCm;     // Last measured distance (-1 = error)
  bool    personNearby;
  bool    dhtError;       // true if DHT11 returns NaN
};

struct FanState {
  bool  running;
  int   pwmValue;
  int   percentSpeed;
};

SensorData sensor = { 25.0, 50.0, 200, false, false };
FanState   fan    = { false, 0, 0 };

// ─── Internal: moving average buffer ──────────────────────────
float   tempBuffer[AVG_SAMPLES];
uint8_t tempIndex  = 0;
bool    bufferFull = false;

// ─── Internal: non-blocking timers ────────────────────────────
unsigned long lastDhtMs   = 0;
unsigned long lastSonarMs = 0;
unsigned long lastLcdMs   = 0;

// ─── Internal: hysteresis state ───────────────────────────────
bool fanEnabledByTemp = false;

// =============================================================
//  OBJECTS
// =============================================================
DHT dht(PIN_DHT, DHT_TYPE);
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

// =============================================================
//  FUNCTION DECLARATIONS
// =============================================================
bool  readDHT(float &tempC, float &humidity);
float smoothedTemperature(float newReading);
int   readDistanceCm();
int   calculatePWM(float tempC);
void  updateFan(SensorData &s, FanState &f);
void  updateDisplay(const SensorData &s, const FanState &f);
void  printPadded(int val, int width);

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(9600);
  Serial.println(F("=== Server Room Fan Controller ==="));

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_FAN,  OUTPUT);
  analogWrite(PIN_FAN, 0);

  dht.begin();

  // Pre-fill moving average – DHT11 needs a moment after power-on
  delay(2000);
  float t, h;
  if (readDHT(t, h)) {
    for (uint8_t i = 0; i < AVG_SAMPLES; i++) tempBuffer[i] = t;
    sensor.tempC    = t;
    sensor.humidity = h;
  }

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
//  LOOP
// =============================================================
void loop() {
  unsigned long now = millis();

  // ── 1. DHT11 reading (max 1 read per 2 s) ──────────────────
  if (now - lastDhtMs >= INTERVAL_DHT) {
    lastDhtMs = now;
    float t, h;
    if (readDHT(t, h)) {
      sensor.dhtError  = false;
      sensor.tempC     = smoothedTemperature(t);
      sensor.humidity  = h;
    } else {
      sensor.dhtError  = true;   // triggers failsafe in updateFan
      Serial.println(F("[ERROR] DHT11 read failed"));
    }
  }

  // ── 2. Sonar / person detection ────────────────────────────
  if (now - lastSonarMs >= INTERVAL_SONAR) {
    lastSonarMs = now;
    sensor.distanceCm  = readDistanceCm();
    sensor.personNearby = (sensor.distanceCm > 0 &&
                           sensor.distanceCm < PERSON_DIST_CM);
  }

  // ── 3. Fan control ─────────────────────────────────────────
  updateFan(sensor, fan);

  // ── 4. Display ─────────────────────────────────────────────
  if (now - lastLcdMs >= INTERVAL_LCD) {
    lastLcdMs = now;
    updateDisplay(sensor, fan);
  }
}

// =============================================================
//  DHT11 READ
//  Returns false if reading is NaN (disconnected or bad signal)
// =============================================================
bool readDHT(float &tempC, float &humidity) {
  float t = dht.readTemperature();   // °C
  float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) return false;
  tempC    = t;
  humidity = h;
  return true;
}

// =============================================================
//  MOVING AVERAGE
// =============================================================
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
//  HC-SR04
// =============================================================
int readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duration = pulseIn(PIN_ECHO, HIGH, SONAR_TIMEOUT_US);
  if (duration == 0) return -1;

  int cm = (int)(duration / 58);
  if (cm > SONAR_MAX_CM) return -1;
  return cm;
}

// =============================================================
//  FAN CONTROL
// =============================================================
int calculatePWM(float tempC) {
  long pwm = map(
    (long)(tempC * 10),
    (long)(TEMP_MIN_MAP * 10),
    (long)(TEMP_MAX_MAP * 10),
    FAN_PWM_MIN,
    FAN_PWM_MAX
  );
  return (int)constrain(pwm, FAN_PWM_MIN, FAN_PWM_MAX);
}

void updateFan(SensorData &s, FanState &f) {

  // ── Failsafe: DHT11 error OR sonar error → full speed ───────
  if (s.dhtError || s.distanceCm == -1) {
    f.running      = true;
    f.pwmValue     = FAN_PWM_MAX;
    f.percentSpeed = 100;
    analogWrite(PIN_FAN, f.pwmValue);
    Serial.println(F("[FAILSAFE] Sensor error – fan at 100%"));
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

  // ── Hysteresis ──────────────────────────────────────────────
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
//  Row 0:  "T:25.3C  H: 60%"
//  Row 1:  "F: 65% D:120cm "
//          "PERSON DETECTED!"
//          "SENSOR ERROR!   "
// =============================================================
void updateDisplay(const SensorData &s, const FanState &f) {

  // ── Row 0: temperature + humidity ───────────────────────────
  lcd.setCursor(0, 0);
  if (s.dhtError) {
    lcd.print(F("T: ERR   H: ERR "));
  } else {
    lcd.print(F("T:"));
    lcd.print(s.tempC, 1);
    lcd.print(F("C H:"));
    printPadded((int)s.humidity, 3);
    lcd.print(F("% "));
  }

  // ── Row 1: fan speed + distance / status ────────────────────
  lcd.setCursor(0, 1);
  if (s.dhtError || s.distanceCm == -1) {
    lcd.print(F("SENSOR ERROR!   "));
  } else if (s.personNearby) {
    lcd.print(F("PERSON DETECTED!"));
  } else {
    lcd.print(F("F:"));
    printPadded(f.percentSpeed, 3);
    lcd.print(F("% D:"));
    printPadded(s.distanceCm, 4);
    lcd.print(F("cm"));
  }

  // ── Serial debug ────────────────────────────────────────────
  Serial.print(F("Temp: "));      Serial.print(s.tempC, 1);
  Serial.print(F("C  Hum: "));    Serial.print(s.humidity, 0);
  Serial.print(F("%  Fan: "));    Serial.print(f.percentSpeed);
  Serial.print(F("%  Dist: "));   Serial.print(s.distanceCm);
  Serial.print(F("cm  Person: ")); Serial.println(s.personNearby ? F("YES") : F("NO"));
}

void printPadded(int val, int width) {
  String s = String(val);
  for (int i = s.length(); i < width; i++) lcd.print(' ');
  lcd.print(s);
}
