// =============================================================
//  Server Room Fan Controller (V3 - optimized)
//  Board  : Arduino Mega 2560
//  Sensor : DHT11  (temp + humidity, digital)
//  Sensor : HC-SR04 (digital)
//  Display: I2C LCD (0x27)
//  Fan    : 5 V PWM fan
// =============================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// ─── PIN CONFIGURATION ────────────────────────────────────────
constexpr uint8_t PIN_DHT  = 2;
constexpr uint8_t PIN_TRIG = 3;
constexpr uint8_t PIN_ECHO = 4;
constexpr uint8_t PIN_FAN  = 9;

// ─── SENSOR + LCD CONFIG ──────────────────────────────────────
#define DHT_TYPE DHT11
constexpr uint8_t LCD_I2C_ADDR = 0x27;
constexpr uint8_t LCD_COLS = 16;
constexpr uint8_t LCD_ROWS = 2;

// ─── CONTROL THRESHOLDS ───────────────────────────────────────
constexpr float TEMP_FAN_ON  = 30.0f;
constexpr float TEMP_FAN_OFF = 28.0f;
constexpr float TEMP_MIN_MAP = 20.0f;
constexpr float TEMP_MAX_MAP = 40.0f;

constexpr uint8_t FAN_PWM_MIN = 100;
constexpr uint8_t FAN_PWM_MAX = 255;

constexpr uint16_t PERSON_DIST_CM   = 50;
constexpr uint16_t SONAR_MAX_CM     = 500;
constexpr unsigned long SONAR_TIMEOUT_US = 40000UL;

// ─── TIMING ────────────────────────────────────────────────────
constexpr unsigned long INTERVAL_DHT_MS   = 2000;
constexpr unsigned long INTERVAL_SONAR_MS = 200;
constexpr unsigned long INTERVAL_LCD_MS   = 1000;
constexpr unsigned long INTERVAL_LOG_MS   = 2000;

// ─── FILTER ────────────────────────────────────────────────────
constexpr uint8_t AVG_SAMPLES = 5;

struct SensorData {
  float tempC;
  float humidity;
  int distanceCm;
  bool personNearby;
  bool dhtError;
};

struct FanState {
  bool running;
  uint8_t pwmValue;
  uint8_t percentSpeed;
};

SensorData sensor{25.0f, 50.0f, 200, false, false};
FanState fan{false, 0, 0};

float tempBuffer[AVG_SAMPLES] = {25, 25, 25, 25, 25};
uint8_t tempIndex = 0;
bool bufferFull = false;
bool fanEnabledByTemp = false;

unsigned long lastDhtMs = 0;
unsigned long lastSonarMs = 0;
unsigned long lastLcdMs = 0;
unsigned long lastLogMs = 0;

DHT dht(PIN_DHT, DHT_TYPE);
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

bool readDHT(float &tempC, float &humidity);
float smoothedTemperature(float newReading);
int readDistanceCm();
uint8_t calculatePWM(float tempC);
void updateFan(const SensorData &s, FanState &f);
void updateDisplay(const SensorData &s, const FanState &f);
void printStatusLog(const SensorData &s, const FanState &f);
void buildDisplayRows(const SensorData &s, const FanState &f, char *row0, char *row1, size_t rowSize);
void renderScrolledRow(uint8_t row, const char *text, uint8_t &offset);

void setup() {
  Serial.begin(9600);
  Serial.println(F("=== Server Room Fan Controller V3 ==="));

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_FAN, OUTPUT);
  analogWrite(PIN_FAN, 0);

  dht.begin();

  // No blocking warm-up delay: first valid DHT read will initialize values in loop().
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Server Room Fan"));
  lcd.setCursor(0, 1);
  lcd.print(F("V3 Ready       "));
}

void loop() {
  const unsigned long now = millis();

  if (now - lastDhtMs >= INTERVAL_DHT_MS) {
    lastDhtMs = now;
    float t, h;
    if (readDHT(t, h)) {
      sensor.dhtError = false;
      sensor.humidity = h;

      // Initialize smoothing buffer once from first real DHT sample.
      static bool dhtInitialized = false;
      if (!dhtInitialized) {
        for (uint8_t i = 0; i < AVG_SAMPLES; ++i) tempBuffer[i] = t;
        sensor.tempC = t;
        dhtInitialized = true;
      } else {
        sensor.tempC = smoothedTemperature(t);
      }
    } else {
      sensor.dhtError = true;
    }
  }

  if (now - lastSonarMs >= INTERVAL_SONAR_MS) {
    lastSonarMs = now;
    sensor.distanceCm = readDistanceCm();
    sensor.personNearby = (sensor.distanceCm > 0 && sensor.distanceCm < PERSON_DIST_CM);
  }

  updateFan(sensor, fan);

  if (now - lastLcdMs >= INTERVAL_LCD_MS) {
    lastLcdMs = now;
    updateDisplay(sensor, fan);
  }

  if (now - lastLogMs >= INTERVAL_LOG_MS) {
    lastLogMs = now;
    printStatusLog(sensor, fan);
  }
}

bool readDHT(float &tempC, float &humidity) {
  const float t = dht.readTemperature();
  const float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) return false;
  tempC = t;
  humidity = h;
  return true;
}

float smoothedTemperature(float newReading) {
  tempBuffer[tempIndex] = newReading;
  tempIndex = (tempIndex + 1) % AVG_SAMPLES;
  if (tempIndex == 0) bufferFull = true;

  const uint8_t count = bufferFull ? AVG_SAMPLES : tempIndex;
  float sum = 0.0f;
  for (uint8_t i = 0; i < count; ++i) sum += tempBuffer[i];
  return (count == 0) ? newReading : (sum / count);
}

int readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  const unsigned long duration = pulseIn(PIN_ECHO, HIGH, SONAR_TIMEOUT_US);
  if (duration == 0) return -1;

  const int cm = static_cast<int>(duration / 58UL);
  return (cm > SONAR_MAX_CM) ? -1 : cm;
}

uint8_t calculatePWM(float tempC) {
  const long pwm = map(
    static_cast<long>(tempC * 10.0f),
    static_cast<long>(TEMP_MIN_MAP * 10.0f),
    static_cast<long>(TEMP_MAX_MAP * 10.0f),
    FAN_PWM_MIN,
    FAN_PWM_MAX
  );
  return static_cast<uint8_t>(constrain(pwm, FAN_PWM_MIN, FAN_PWM_MAX));
}

void updateFan(const SensorData &s, FanState &f) {
  if (s.dhtError || s.distanceCm == -1) {
    f = {true, FAN_PWM_MAX, 100};
    analogWrite(PIN_FAN, f.pwmValue);
    return;
  }

  if (s.personNearby) {
    f = {false, 0, 0};
    analogWrite(PIN_FAN, 0);
    return;
  }

  if (!fanEnabledByTemp && s.tempC >= TEMP_FAN_ON) fanEnabledByTemp = true;
  if (fanEnabledByTemp && s.tempC < TEMP_FAN_OFF) fanEnabledByTemp = false;

  if (fanEnabledByTemp) {
    f.running = true;
    f.pwmValue = calculatePWM(s.tempC);
    f.percentSpeed = static_cast<uint8_t>(map(f.pwmValue, 0, FAN_PWM_MAX, 0, 100));
    analogWrite(PIN_FAN, f.pwmValue);
  } else {
    f = {false, 0, 0};
    analogWrite(PIN_FAN, 0);
  }
}

void updateDisplay(const SensorData &s, const FanState &f) {
  // Build wider status rows and scroll them right-to-left for cleaner readability.
  static uint8_t row0Offset = 0;
  static uint8_t row1Offset = 0;
  char row0[48];
  char row1[48];

  buildDisplayRows(s, f, row0, row1, sizeof(row0));
  renderScrolledRow(0, row0, row0Offset);
  renderScrolledRow(1, row1, row1Offset);
}

void buildDisplayRows(const SensorData &s, const FanState &f, char *row0, char *row1, size_t rowSize) {
  if (s.dhtError) {
    snprintf(row0, rowSize, "TEMP:ERR  HUM:ERR");
  } else {
    snprintf(row0, rowSize, "TEMP:%4.1fC  HUM:%2d%%  ", s.tempC, static_cast<int>(s.humidity));
  }

  if (s.dhtError || s.distanceCm == -1) {
    snprintf(row1, rowSize, "SENSOR ERROR  CHECK DHT/SONAR  ");
  } else if (s.personNearby) {
    snprintf(row1, rowSize, "PERSON DETECTED  FAN OFF  DIST:%dcm  ", s.distanceCm);
  } else {
    snprintf(row1, rowSize, "FAN:%3d%%  DIST:%3dcm  STATUS:OK  ", f.percentSpeed, s.distanceCm);
  }
}

void renderScrolledRow(uint8_t row, const char *text, uint8_t &offset) {
  const uint8_t width = LCD_COLS;
  const size_t len = strlen(text);

  lcd.setCursor(0, row);
  if (len <= width) {
    lcd.print(text);
    for (uint8_t i = len; i < width; ++i) lcd.print(' ');
    offset = 0;
    return;
  }

  // Seamless right-to-left scrolling by appending gap then wrapping.
  char scrollBuf[96];
  snprintf(scrollBuf, sizeof(scrollBuf), "%s   %s", text, text);
  const size_t cycleLen = len + 3;

  for (uint8_t i = 0; i < width; ++i) {
    lcd.print(scrollBuf[offset + i]);
  }

  offset = (offset + 1) % cycleLen;
}

void printStatusLog(const SensorData &s, const FanState &f) {
  Serial.print(F("Temp:")); Serial.print(s.tempC, 1);
  Serial.print(F("C Hum:")); Serial.print(s.humidity, 0);
  Serial.print(F("% Fan:")); Serial.print(f.percentSpeed);
  Serial.print(F("% Dist:")); Serial.print(s.distanceCm);
  Serial.print(F("cm Person:")); Serial.println(s.personNearby ? F("YES") : F("NO"));
}

