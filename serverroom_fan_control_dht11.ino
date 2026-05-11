// =============================================================
//  Server Room Fan Controller – Variante mit DHT11
//
//  Ziel des Sketches:
//    * Temperatur und Luftfeuchtigkeit im Serverraum messen.
//    * Per Ultraschallsensor erkennen, ob eine Person nahe am Lüfter steht.
//    * Den 5-V-Lüfter temperaturabhängig per PWM regeln.
//    * Messwerte und Status auf einem 16x2-I2C-LCD sowie seriell ausgeben.
//
//  Board  : Arduino Mega 2560
//  Sensor : DHT11  (digitale Temperatur- und Feuchtemessung)
//  Sensor : HC-SR04 (digitale Abstandsmessung)
//  Display: I2C LCD  – LED 8050 R7  (Adresse 0x27, bei Bedarf ändern)
//  Fan    : 5 V PWM-Lüfter
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
// Alle Hardware-Anschlüsse werden zentral definiert, damit die
// Verkabelung später ohne Suche im Programm angepasst werden kann.
#define PIN_DHT         2   // DHT11-Datenleitung; ein beliebiger Digitalpin genügt.
#define PIN_TRIG        3   // HC-SR04-Trigger: Arduino sendet hier den Ultraschall-Impuls.
#define PIN_ECHO        4   // HC-SR04-Echo: Pulsdauer entspricht der gemessenen Entfernung.
#define PIN_FAN         9   // PWM-fähiger Ausgang zur Drehzahlsteuerung des Lüfters.

// ─── DHT SENSOR TYPE ──────────────────────────────────────────
// Die Adafruit-DHT-Bibliothek benötigt den Sensortyp beim Erzeugen
// des Objekts. Bei einem späteren Wechsel auf DHT22 reicht diese Zeile.
#define DHT_TYPE       DHT11

// ─── LCD CONFIGURATION ────────────────────────────────────────
// Adresse und Geometrie des I2C-LCDs. Bleibt das Display leer, ist
// häufig 0x3F statt 0x27 die richtige I2C-Adresse.
#define LCD_I2C_ADDR   0x27
#define LCD_COLS        16
#define LCD_ROWS         2

// ─── TEMPERATURE THRESHOLDS (°C) ──────────────────────────────
// Die Ein- und Ausschaltpunkte sind absichtlich verschieden. Diese
// Hysterese verhindert, dass der Lüfter bei ca. 30 °C ständig taktet.
#define TEMP_FAN_ON     30.0   // Ab dieser Temperatur wird die Lüfterfreigabe gesetzt.
#define TEMP_FAN_OFF    28.0   // Unter diesem Wert wird die Lüfterfreigabe wieder gelöscht.
#define TEMP_MIN_MAP    20.0   // Temperatur, die auf FAN_PWM_MIN abgebildet wird.
#define TEMP_MAX_MAP    40.0   // Temperatur, die auf FAN_PWM_MAX/255 abgebildet wird.

// ─── FAN PWM LIMITS ───────────────────────────────────────────
// PWM-Werte liegen bei Arduino analogWrite() zwischen 0 und 255.
// Viele Lüfter laufen bei sehr kleinen PWM-Werten nicht zuverlässig an.
#define FAN_PWM_MIN    100     // Mindestwert, sobald der Lüfter eingeschaltet ist.
#define FAN_PWM_MAX    255     // Maximalwert: entspricht 100 % Einschaltdauer.

// ─── PERSON DETECTION ─────────────────────────────────────────
// Der Ultraschallsensor dient als Sicherheits-/Komfortabschaltung:
// steht jemand sehr nah am Gerät, wird der Lüfter unabhängig von der
// Temperatur ausgeschaltet. Ungültige Messungen lösen den Failsafe aus.
#define PERSON_DIST_CM  50     // Unterhalb dieser Entfernung gilt: Person erkannt.
#define SONAR_MAX_CM   500     // Größere Werte werden als ungültig verworfen.
#define SONAR_TIMEOUT_US 40000UL  // Maximale Wartezeit auf Echo; 0 bedeutet Timeout.

// ─── TIMING INTERVALS (ms) ────────────────────────────────────
// Die Hauptschleife arbeitet mit millis()-Zeitstempeln statt langen
// delay()-Pausen. So bleibt die Lüfterregelung reaktionsfähig.
#define INTERVAL_DHT   2000    // DHT11 darf nur etwa alle 1-2 Sekunden gelesen werden.
#define INTERVAL_SONAR  200    // Abstandsmessung wird deutlich häufiger aktualisiert.
#define INTERVAL_LCD   1000    // LCD-Refresh: langsam genug, um Flackern zu vermeiden.

// ─── MOVING AVERAGE ───────────────────────────────────────────
// Der gleitende Mittelwert glättet sprunghafte Sensorwerte, ohne die
// Regelung komplett träge zu machen. Beim DHT11 reichen wenige Samples.
#define AVG_SAMPLES     5

// =============================================================
//  DATA STRUCTURES
// =============================================================

struct SensorData {
  float   tempC;          // Geglättete Temperatur in °C; Grundlage für die PWM-Regelung.
  float   humidity;       // Zuletzt gültig gemessene relative Luftfeuchtigkeit in %.
  int     distanceCm;     // Letzte Entfernung in cm; -1 kennzeichnet Timeout/Fehler.
  bool    personNearby;   // true, wenn distanceCm innerhalb PERSON_DIST_CM liegt.
  bool    dhtError;       // true, wenn der DHT11 NaN liefert oder nicht erreichbar ist.
};

struct FanState {
  bool  running;          // Logischer Laufzustand: true = Lüfter soll drehen.
  int   pwmValue;         // Tatsächlich ausgegebener PWM-Wert von 0 bis 255.
  int   percentSpeed;     // PWM-Wert als Prozentangabe für Display/Debug-Ausgabe.
};

SensorData sensor = { 25.0, 50.0, 200, false, false };
FanState   fan    = { false, 0, 0 };

// ─── Internal: moving average buffer ──────────────────────────
// Ringpuffer: jeder neue Temperaturwert überschreibt den ältesten Wert.
float   tempBuffer[AVG_SAMPLES];
uint8_t tempIndex  = 0;
bool    bufferFull = false;

// ─── Internal: non-blocking timers ────────────────────────────
// Diese Zeitstempel merken, wann ein Teilbereich zuletzt ausgeführt wurde.
unsigned long lastDhtMs   = 0;
unsigned long lastSonarMs = 0;
unsigned long lastLcdMs   = 0;

// ─── Internal: hysteresis state ───────────────────────────────
// Merkt, ob die Temperaturregelung aktuell freigegeben ist. Ohne diesen
// Zustand wäre keine saubere Hysterese zwischen ON und OFF möglich.
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
  // setup() läuft genau einmal nach Reset/Upload. Hier wird die
  // Hardware vorbereitet und es werden sinnvolle Startwerte gesetzt.
  Serial.begin(9600);
  Serial.println(F("=== Server Room Fan Controller ==="));

  // Richtung der I/O-Pins festlegen. Der Lüfter wird direkt auf 0
  // gesetzt, damit er beim Start nicht unbeabsichtigt kurz anläuft.
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_FAN,  OUTPUT);
  analogWrite(PIN_FAN, 0);

  dht.begin();  // DHT-Bibliothek initialisieren, bevor Messwerte gelesen werden.

  // Vor dem ersten DHT11-Lesen kurz warten: der Sensor braucht nach
  // dem Einschalten etwas Zeit. Der Mittelwertpuffer wird anschließend
  // mit einem realen Wert gefüllt, damit die erste Anzeige stabil ist.
  delay(2000);
  float t, h;
  if (readDHT(t, h)) {
    for (uint8_t i = 0; i < AVG_SAMPLES; i++) tempBuffer[i] = t;
    sensor.tempC    = t;
    sensor.humidity = h;
  }

  // LCD starten und eine kurze Initialisierungsmeldung anzeigen.
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
//  LOOP – zyklische, weitgehend nicht-blockierende Steuerung
// =============================================================
void loop() {
  // Aktuelle Laufzeit einmal speichern. Alle Zeitvergleiche nutzen
  // denselben Wert, dadurch bleiben die Teilaufgaben konsistent.
  unsigned long now = millis();

  // ── 1. DHT11-Messung (max. eine Messung alle 2 s) ─────────────
  // Nur bei abgelaufenem Intervall lesen. Bei gültigem Wert wird die
  // Temperatur geglättet; bei Fehler bleibt der letzte Wert erhalten
  // und updateFan() schaltet in den Sicherheitsmodus.
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

  // ── 2. Ultraschall / Personenerkennung ───────────────────────
  // Eine positive Entfernung unterhalb PERSON_DIST_CM sperrt den Lüfter.
  // -1 bedeutet Messfehler/Timeout und wird später als Failsafe behandelt.
  if (now - lastSonarMs >= INTERVAL_SONAR) {
    lastSonarMs = now;
    sensor.distanceCm  = readDistanceCm();
    sensor.personNearby = (sensor.distanceCm > 0 &&
                           sensor.distanceCm < PERSON_DIST_CM);
  }

  // ── 3. Lüfterregelung ────────────────────────────────────────
  // Diese Funktion fasst Sensorfehler, Personenerkennung, Hysterese und
  // PWM-Berechnung zu einer klaren Stellgröße für den Lüfter zusammen.
  updateFan(sensor, fan);

  // ── 4. Display/Debug-Ausgabe ─────────────────────────────────
  // Das LCD muss nicht bei jedem loop()-Durchlauf aktualisiert werden.
  if (now - lastLcdMs >= INTERVAL_LCD) {
    lastLcdMs = now;
    updateDisplay(sensor, fan);
  }
}

// =============================================================
//  DHT11 READ
//  Kapselt den Sensorzugriff. Die Funktion liefert true nur dann,
//  wenn Temperatur und Luftfeuchte gültige Zahlen sind. So muss der
//  restliche Code nicht direkt mit NaN-Werten umgehen.
// =============================================================
bool readDHT(float &tempC, float &humidity) {
  float t = dht.readTemperature();   // Standardmäßig in °C.
  float h = dht.readHumidity();      // Relative Luftfeuchtigkeit in %.
  if (isnan(t) || isnan(h)) return false;  // Bibliothek nutzt NaN als Fehlercode.
  tempC    = t;
  humidity = h;
  return true;
}

// =============================================================
//  MOVING AVERAGE
//  Speichert neue Messwerte in einem Ringpuffer und bildet daraus den
//  Durchschnitt. Dadurch reagiert der Lüfter nicht auf einzelne Ausreißer.
// =============================================================
float smoothedTemperature(float newReading) {
  tempBuffer[tempIndex] = newReading;          // Neuen Wert an aktueller Position ablegen.
  tempIndex = (tempIndex + 1) % AVG_SAMPLES;   // Index zyklisch weiterschalten.
  if (tempIndex == 0) bufferFull = true;       // Nach einem Umlauf sind alle Plätze belegt.

  uint8_t count = bufferFull ? AVG_SAMPLES : tempIndex;  // Beim Start nur gefüllte Felder mitteln.
  float   sum   = 0.0;
  for (uint8_t i = 0; i < count; i++) sum += tempBuffer[i];
  return sum / count;
}

// =============================================================
//  HC-SR04
//  Erzeugt den Triggerimpuls, misst die Echo-Pulsdauer und rechnet sie
//  in Zentimeter um. Rückgabewert -1 bedeutet: Messung unbrauchbar.
// =============================================================
int readDistanceCm() {
  // Sauberer Trigger: kurz LOW, dann 10 µs HIGH, danach wieder LOW.
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // pulseIn() blockiert maximal SONAR_TIMEOUT_US. Dauer 0 = kein Echo.
  long duration = pulseIn(PIN_ECHO, HIGH, SONAR_TIMEOUT_US);
  if (duration == 0) return -1;

  // Faustformel HC-SR04: Mikrosekunden / 58 ≈ Entfernung in cm.
  int cm = (int)(duration / 58);
  if (cm > SONAR_MAX_CM) return -1;  // unrealistische Werte verwerfen
  return cm;
}

// =============================================================
//  FAN CONTROL
//  calculatePWM() bildet die Temperatur linear auf den zulässigen
//  PWM-Bereich ab. updateFan() entscheidet anschließend, ob dieser
//  Wert ausgegeben oder wegen Person/Fehler auf 0/100 % gesetzt wird.
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
  // Priorität der Regeln:
  //   1. Sensorfehler -> maximal kühlen (Schutz des Serverraums).
  //   2. Person in der Nähe -> Lüfter aus (Sicherheit/Komfort).
  //   3. Temperaturregelung mit Hysterese und PWM-Kennlinie.

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

  // ── Hysterese ────────────────────────────────────────────────
  // Einschalten erst ab TEMP_FAN_ON, Ausschalten erst unter TEMP_FAN_OFF.
  // Dadurch bleibt der Zustand zwischen 28 °C und 30 °C stabil.
  if (!fanEnabledByTemp && s.tempC >= TEMP_FAN_ON)  fanEnabledByTemp = true;
  if ( fanEnabledByTemp && s.tempC <  TEMP_FAN_OFF) fanEnabledByTemp = false;

  if (fanEnabledByTemp) {
    f.running      = true;
    f.pwmValue     = calculatePWM(s.tempC);  // wärmer = höherer PWM-Wert
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
//  LCD DISPLAY  –  16x2 layout
//
//  Row 0:  "T:25.3C  H: 60%"
//  Row 1:  "F: 65% D:120cm "
//          "PERSON DETECTED!"
//          "SENSOR ERROR!   "
// =============================================================
void updateDisplay(const SensorData &s, const FanState &f) {

  // ── Zeile 0: Temperatur + Luftfeuchtigkeit ─────────────────
  // Bei DHT-Fehlern wird ausdrücklich ERR angezeigt, damit alte Werte
  // nicht versehentlich als aktuelle Messung interpretiert werden.
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

  // ── Zeile 1: Lüfterdrehzahl + Entfernung oder Statusmeldung ─
  // Statusmeldungen haben Vorrang vor Zahlenwerten, weil sie für den
  // Betreiber wichtiger sind als die Detailanzeige.
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

  // ── Serielle Debug-Ausgabe ──────────────────────────────────
  // Diese Werte helfen beim Abgleich mit realen Sensorwerten im
  // Serial Monitor bei 9600 Baud.
  Serial.print(F("Temp: "));      Serial.print(s.tempC, 1);
  Serial.print(F("C  Hum: "));    Serial.print(s.humidity, 0);
  Serial.print(F("%  Fan: "));    Serial.print(f.percentSpeed);
  Serial.print(F("%  Dist: "));   Serial.print(s.distanceCm);
  Serial.print(F("cm  Person: ")); Serial.println(s.personNearby ? F("YES") : F("NO"));
}

// Gibt Zahlen rechtsbündig mit führenden Leerzeichen aus. Dadurch
// werden bei kürzeren neuen Werten alte LCD-Ziffern zuverlässig gelöscht.
void printPadded(int val, int width) {
  String s = String(val);
  for (int i = s.length(); i < width; i++) lcd.print(' ');
  lcd.print(s);
}
