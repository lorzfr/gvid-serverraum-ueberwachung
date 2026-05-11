// =============================================================
//  Server Room Fan Controller – Variante mit TMP36
//
//  Ziel des Sketches:
//    * Temperatur analog über einen TMP36 messen.
//    * Per Ultraschallsensor erkennen, ob eine Person nahe am Lüfter steht.
//    * Den 5-V-Lüfter temperaturabhängig per PWM regeln.
//    * Messwerte und Status auf einem 16x2-I2C-LCD sowie seriell ausgeben.
//
//  Board  : Arduino Mega 2560
//  Sensor : TMP36  (analoger Temperatursensor)
//  Sensor : HC-SR04 (digitale Abstandsmessung)
//  Display: I2C LCD  – LED 8050 R7  (Adresse 0x27, bei Bedarf ändern)
//  Fan    : 5 V PWM-Lüfter
// =============================================================
//  Required libraries (install via Library Manager):
//    • LiquidCrystal_I2C  by Frank de Brabander
// =============================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ─── PIN CONFIGURATION ────────────────────────────────────────
// Alle Hardware-Anschlüsse werden zentral definiert, damit die
// Verkabelung später ohne Suche im Programm angepasst werden kann.
#define PIN_TMP36      A0   // Analoger TMP36-Ausgang; analogRead() liefert 0..1023.
#define PIN_TRIG        3   // HC-SR04-Trigger: Arduino sendet hier den Ultraschall-Impuls.
#define PIN_ECHO        4   // HC-SR04-Echo: Pulsdauer entspricht der gemessenen Entfernung.
#define PIN_FAN         9   // PWM-fähiger Ausgang zur Drehzahlsteuerung des Lüfters.

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
#define INTERVAL_TEMP   500    // Analogtemperatur regelmäßig, aber nicht unnötig oft lesen.
#define INTERVAL_SONAR  200    // Abstandsmessung wird deutlich häufiger aktualisiert.
#define INTERVAL_LCD   1000    // LCD-Refresh: langsam genug, um Flackern zu vermeiden.

// ─── MOVING AVERAGE ───────────────────────────────────────────
// Der TMP36 ist analog und kann etwas rauschen. Zehn Samples ergeben
// eine ruhige Anzeige, ohne die Regelung zu stark zu verzögern.
#define AVG_SAMPLES     10

// =============================================================
//  DATA STRUCTURES
// =============================================================

struct SensorData {
  float   tempC;          // Geglättete Temperatur in °C; Grundlage für die PWM-Regelung.
  int     distanceCm;     // Letzte Entfernung in cm; -1 kennzeichnet Timeout/Fehler.
  bool    personNearby;   // true, wenn distanceCm innerhalb PERSON_DIST_CM liegt.
};

struct FanState {
  bool  running;          // Logischer Laufzustand: true = Lüfter soll drehen.
  int   pwmValue;         // Tatsächlich ausgegebener PWM-Wert von 0 bis 255.
  int   percentSpeed;     // PWM-Wert als Prozentangabe für Display/Debug-Ausgabe.
};

SensorData sensor = { 25.0, 200, false };
FanState   fan    = { false, 0, 0 };

// ─── Internal: moving average buffer ──────────────────────────
// Ringpuffer: jeder neue Temperaturwert überschreibt den ältesten Wert.
float   tempBuffer[AVG_SAMPLES];
uint8_t tempIndex     = 0;
bool    bufferFull    = false;

// ─── Internal: non-blocking timers ────────────────────────────
// Diese Zeitstempel merken, wann ein Teilbereich zuletzt ausgeführt wurde.
unsigned long lastTempMs  = 0;
unsigned long lastSonarMs = 0;
unsigned long lastLcdMs   = 0;

// ─── Internal: hysteresis state ───────────────────────────────
// Merkt, ob die Temperaturregelung aktuell freigegeben ist. Ohne diesen
// Zustand wäre keine saubere Hysterese zwischen ON und OFF möglich.
bool fanEnabledByTemp = false;

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

  // Mittelwertpuffer mit einem realen Startwert füllen. Dadurch zeigt
  // die erste Berechnung keinen künstlichen Durchschnitt aus Nullen.
  float firstRead = readTemperatureC();
  for (uint8_t i = 0; i < AVG_SAMPLES; i++) {
    tempBuffer[i] = firstRead;
  }
  sensor.tempC = firstRead;

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

  // ── 1. Temperaturmessung ────────────────────────────────────
  // Nur bei abgelaufenem Intervall lesen und anschließend glätten.
  // Die analoge Messung ist schnell, das Intervall reduziert aber Rauschen.
  if (now - lastTempMs >= INTERVAL_TEMP) {
    lastTempMs = now;
    float raw    = readTemperatureC();
    sensor.tempC = smoothedTemperature(raw);
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
//  TEMPERATURE  –  TMP36
// =============================================================
// Der TMP36 liefert 750 mV bei 25 °C und ändert sich um 10 mV/°C.
// Formel: Temp(°C) = (mV - 500) / 10. Bei 5-V-Referenz gilt:
// mV = analogRead * (5000.0 / 1024.0).
float readTemperatureC() {
  int   raw = analogRead(PIN_TMP36);      // 10-bit ADC-Wert: 0..1023
  float mV  = raw * (5000.0 / 1024.0);   // ADC-Wert in Millivolt umrechnen.
  float tempC = (mV - 500.0) / 10.0;     // TMP36-Offset und Steigung anwenden.
  return tempC;
}

// Speichert neue Messwerte in einem Ringpuffer und bildet daraus den
// Durchschnitt. Dadurch reagiert der Lüfter nicht auf einzelne Ausreißer.
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
//  DISTANCE  –  HC-SR04
// =============================================================
// Erzeugt den Triggerimpuls, misst die Echo-Pulsdauer und rechnet sie
// in Zentimeter um. Rückgabewert -1 bedeutet: Messung unbrauchbar.
int readDistanceCm() {
  // Sauberer Trigger: kurz LOW, dann 10 µs HIGH, danach wieder LOW.
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // pulseIn() blockiert maximal SONAR_TIMEOUT_US. Dauer 0 = kein Echo.
  long duration = pulseIn(PIN_ECHO, HIGH, SONAR_TIMEOUT_US);

  if (duration == 0) {
    // Timeout: Sensor getrennt, Echo zu schwach oder Objekt zu weit entfernt.
    return -1;
  }

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
  // map() arbeitet ganzzahlig; Skalierung x10 erhält eine Nachkommastelle.
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
  // Priorität der Regeln:
  //   1. Sensorfehler -> maximal kühlen (Schutz des Serverraums).
  //   2. Person in der Nähe -> Lüfter aus (Sicherheit/Komfort).
  //   3. Temperaturregelung mit Hysterese und PWM-Kennlinie.

  // ── Failsafe: Sonar-Timeout (-1) -> volle Drehzahl ──────────
  if (s.distanceCm == -1) {
    f.running      = true;
    f.pwmValue     = FAN_PWM_MAX;
    f.percentSpeed = 100;
    analogWrite(PIN_FAN, f.pwmValue);
    Serial.println(F("[FAILSAFE] Sonar error – fan at 100%"));
    return;
  }

  // ── Person in der Nähe -> Lüfter zwangsweise aus ────────────
  if (s.personNearby) {
    f.running      = false;
    f.pwmValue     = 0;
    f.percentSpeed = 0;
    analogWrite(PIN_FAN, 0);
    return;
  }

  // ── Hysterese für Temperaturschwelle ────────────────────────
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
//  Row 0:  "T:  25.3C  F: 65%"
//  Row 1:  "Dist: 120cm  OK "   or "PERSON DETECTED!"
//           (failsafe) "SENSOR ERROR!  "
// =============================================================
void updateDisplay(const SensorData &s, const FanState &f) {
  // ── Zeile 0: Temperatur + Lüfterleistung ───────────────────
  lcd.setCursor(0, 0);
  lcd.print(F("T:"));
  // Temperatur mit einer Nachkommastelle; danach folgt die aktuelle Lüfterleistung.
  lcd.print(s.tempC, 1);
  lcd.print(F("C  F:"));
  printPadded(f.percentSpeed, 3);
  lcd.print(F("%"));

  // ── Zeile 1: Entfernung oder Statusmeldung ─────────────────
  // Statusmeldungen haben Vorrang vor Zahlenwerten, weil sie für den
  // Betreiber wichtiger sind als die Detailanzeige.
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

  // ── Serielle Debug-Ausgabe ──────────────────────────────────
  // Diese Werte helfen beim Abgleich mit realen Sensorwerten im
  // Serial Monitor bei 9600 Baud.
  Serial.print(F("Temp: "));    Serial.print(s.tempC, 1);
  Serial.print(F("°C  Fan: ")); Serial.print(f.percentSpeed);
  Serial.print(F("%  Dist: ")); Serial.print(s.distanceCm);
  Serial.print(F("cm  Person: ")); Serial.println(s.personNearby ? F("YES") : F("NO"));
}

// Gibt Zahlen rechtsbündig mit führenden Leerzeichen aus. Dadurch
// werden bei kürzeren neuen Werten alte LCD-Ziffern zuverlässig gelöscht.
void printPadded(int val, int width) {
  String s = String(val);
  for (int i = s.length(); i < width; i++) lcd.print(' ');
  lcd.print(s);
}
