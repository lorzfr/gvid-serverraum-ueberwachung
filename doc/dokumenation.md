# Deckblatt / Projektübersicht

**Titel des Projekts:** Serverraumüberwachung mit temperaturabhängiger Lüftersteuerung (TMP36- und DHT11-Version)  
**Projektbeschreibung:** Dieses Projekt steuert einen 5V-PWM-Lüfter für einen Serverraum auf Basis der gemessenen Temperatur. Zusätzlich erkennt ein HC-SR04-Abstandssensor Personen in der Nähe und stoppt den Lüfter aus Sicherheitsgründen; bei Sensorfehlern greift ein Failsafe und schaltet den Lüfter auf 100%.  
**Ziele und Anforderungen:**
- Temperaturüberwachung in Echtzeit.
- Automatische Lüfterregelung mit Hysterese gegen Flattern.
- Sicherheitslogik: Lüfter aus bei Personenerkennung.
- Robuste Fehlerstrategie (Failsafe bei Sensorfehler).
- Darstellung der Betriebswerte auf I2C-LCD (16x2).

**Verwendete Technologien:**
- **Hardware:** Arduino Mega 2560, TMP36 oder DHT11, HC-SR04, I2C-LCD (0x27), 5V PWM-Lüfter.
- **Software:** Arduino C/C++, Libraries `LiquidCrystal_I2C`, bei DHT-Version zusätzlich `DHT` (Adafruit).
- **Schaltplan:** Fritzing-Datei im Repository: `serverraumüberwachung.fzz`.

**Teammitglieder / Aufgabenverteilung:**
- TODO: Namen ergänzen.
- TODO: Aufgaben pro Person ergänzen (Hardware, Code, Test, Dokumentation).

**Datum:** 04.05.2026  
**Betreuer/Lehrer:** TODO: Name ergänzen.

---

# Inhaltsverzeichnis
1. Systemarchitektur & Hardware
2. Code-Dokumentation (TMP36 und DHT11)
3. Testphase und Fehlerbehandlung
4. Benutzeranleitung
5. Zusammenfassung und Reflexion

---

# 1) Systemarchitektur & Hardware

## 1.1 Gesamtaufbau
Das System besteht aus vier Funktionsblöcken:
1. **Temperatursensor** (wahlweise TMP36 analog oder DHT11 digital)
2. **Abstandssensor HC-SR04** zur Personenerkennung
3. **Aktor Lüfter** (PWM an Pin 9)
4. **Ausgabe** auf I2C-LCD (16x2)

Die Hauptlogik läuft nicht-blockierend mit `millis()`-Intervallen:
- Sensoren zyklisch lesen,
- Lüfterstatus sofort neu berechnen,
- Display periodisch aktualisieren.

## 1.2 Pinbelegung
### TMP36-Version
- `A0`: TMP36 Temperaturausgang
- `3`: HC-SR04 Trigger
- `4`: HC-SR04 Echo
- `9`: Lüfter PWM

### DHT11-Version
- `2`: DHT11 Datenpin
- `3`: HC-SR04 Trigger
- `4`: HC-SR04 Echo
- `9`: Lüfter PWM

## 1.3 Schaltplan
- Der Schaltplan ist als Fritzing-Projekt vorhanden: `serverraumüberwachung.fzz`.
- Für die finale Abgabe bitte zusätzlich einen exportierten Schaltplan (PNG/PDF) einfügen.

## 1.4 Hardwareliste
- 1x Arduino Mega 2560
- 1x TMP36 **oder** 1x DHT11
- 1x HC-SR04 Ultraschallsensor
- 1x I2C LCD 16x2
- 1x 5V PWM-Lüfter
- Verbindungskabel, ggf. Breadboard

---

# 2) Code-Dokumentation (TMP36 und DHT11)

## 2.1 Gemeinsame Logik in beiden Versionen

## 2.1.1 Regelprinzip
- **Temperaturgrenzen:**
  - `TEMP_FAN_ON = 30.0°C`
  - `TEMP_FAN_OFF = 28.0°C` (Hysterese)
- **PWM-Mapping:** linear von `TEMP_MIN_MAP = 20.0°C` bis `TEMP_MAX_MAP = 40.0°C` auf den Bereich `FAN_PWM_MIN` bis `255`.
- **Personenschutz:** Wenn Distanz `< PERSON_DIST_CM` (50 cm), wird der Lüfter abgeschaltet.
- **Failsafe:** Bei Sensorausfall läuft Lüfter mit 100%, um Überhitzung zu vermeiden.

## 2.1.2 Zustandsdaten
Beide Programme nutzen strukturierte Zustände:
- `SensorData`: Messwerte und Fehlerzustände
- `FanState`: Lüfterstatus, PWM, Prozentwert

Vorteil: klare Trennung von Messung, Entscheidung und Anzeige.

## 2.1.3 Zeitsteuerung
Die Hauptschleife nutzt mehrere Intervalle:
- Temperatur: 500 ms (TMP36) bzw. 2000 ms (DHT11)
- HC-SR04: 200 ms
- LCD: 1000 ms

Dadurch bleibt das System reaktionsschnell und blockiert nicht unnötig.

## 2.2 TMP36-Version – Analyse
Datei: `serverroom_fan_control-TMP36.ino`

### Besonderheiten
- Temperatur wird analog eingelesen und per Formel umgerechnet:  
  `Temp(°C) = (mV - 500) / 10`
- Ein gleitender Mittelwert mit 10 Samples reduziert Messrauschen.
- Failsafe wird bei HC-SR04-Fehler (`distanceCm == -1`) ausgelöst.

### Wichtige Funktionen
- `readTemperatureC()` – analoger TMP36-Read inkl. Umrechnung.
- `smoothedTemperature()` – gleitender Mittelwert.
- `readDistanceCm()` – Ultraschallmessung mit Timeout.
- `updateFan()` – Prioritätslogik: Failsafe > Person erkannt > Temperaturregelung.
- `updateDisplay()` – 16x2 Darstellung von Temperatur, Lüfterleistung, Distanz und Status.

## 2.3 DHT11-Version – Analyse
Datei: `serverroom_fan_control_dht11.ino`

### Besonderheiten
- Temperatur und Luftfeuchte werden digital per DHT11 gelesen.
- DHT11 darf nur langsam gelesen werden, deshalb `INTERVAL_DHT = 2000 ms`.
- Gleitender Mittelwert mit 5 Samples (passend zur langsameren DHT11-Rate).
- Zusätzlicher Fehlerzustand `dhtError`; Failsafe bei DHT- **oder** Sonarfehler.

### Wichtige Funktionen
- `readDHT(float &tempC, float &humidity)` – liest Sensorwerte und prüft auf `NaN`.
- `smoothedTemperature()` – Mittelwertbildung.
- `updateFan()` – Failsafe bei `dhtError || distanceCm == -1`.
- `updateDisplay()` – zeigt Temperatur, Feuchte, Lüfter und Distanz; im Fehlerfall „ERR“.

## 2.4 Vergleich TMP36 vs. DHT11

| Kriterium | TMP36-Version | DHT11-Version |
|---|---|---|
| Signalart | Analog | Digital |
| Messgrößen | Temperatur | Temperatur + Luftfeuchte |
| Update-Frequenz | schneller (500 ms) | langsamer (2000 ms) |
| Fehlerfälle | Sonarfehler abgesichert | DHT- und Sonarfehler abgesichert |
| Bibliotheken | nur LCD | LCD + DHT |

### Fazit Vergleich
- **TMP36** eignet sich für schnelle, einfache Temperaturregelung.
- **DHT11** bietet zusätzliche Umgebungsdaten (Luftfeuchte), ist aber träger.

## 2.5 Programmablaufplan (PAP)
Für die Abgabe in draw.io (DIN 66001) sollte der Ablauf so dargestellt werden:
1. Start / Setup
2. Temperatur lesen (TMP36 oder DHT11)
3. Abstand lesen
4. Sensorfehler? → Ja: Lüfter 100%
5. Person erkannt? → Ja: Lüfter aus
6. Sonst: Hysterese + PWM aus Temperatur berechnen
7. Display aktualisieren
8. Zurück zu Schleifenanfang

---

# 3) Testphase und Fehlerbehandlung

## 3.1 Fehlerprotokoll (typische Fälle)
1. **LCD bleibt leer**
   - Ursache: falsche I2C-Adresse.
   - Lösung: `0x27` auf `0x3F` ändern.

2. **DHT11 liefert `NaN`**
   - Ursache: zu schnelle Abfrage / Leitungsproblem.
   - Lösung: 2-Sekunden-Intervall einhalten, Verkabelung prüfen.

3. **HC-SR04 keine Werte**
   - Ursache: Echo-Timeout oder Reichweite überschritten.
   - Lösung: Timeout/Verkabelung prüfen; System setzt automatisch Failsafe.

4. **Lüfter startet nicht zuverlässig**
   - Ursache: PWM-Startwert zu niedrig.
   - Lösung: `FAN_PWM_MIN` erhöhen.

## 3.2 Testreihen
- **Schwellwerttest:** Sensor erwärmen (z. B. Föhn), prüfen ob Lüfter ab ~30°C startet.
- **Hysterese-Test:** Temperatur langsam senken, prüfen ob Lüfter erst unter 28°C ausgeht.
- **Personenschutz-Test:** Hand vor HC-SR04 (<50 cm), Lüfter muss stoppen.
- **Failsafe-Test Sonar:** Echo-Leitung lösen/simulieren, Lüfter muss 100% laufen.
- **Failsafe-Test DHT11:** DHT11 trennen, Lüfter muss 100% laufen.
- **Langzeittest:** 30–60 Minuten Betrieb, stabile Anzeige und keine Hänger.

---

# 4) Benutzeranleitung

## 4.1 Start
1. Hardware gemäß Schaltplan anschließen.
2. Passenden Sketch hochladen:
   - `serverroom_fan_control-TMP36.ino` oder
   - `serverroom_fan_control_dht11.ino`
3. Arduino mit Strom versorgen.
4. Nach Initialisierung erscheinen Messwerte auf dem LCD.

## 4.2 Bedienung
- Es gibt keine aktiven Bedienelemente; das System arbeitet automatisch.
- Optional können Grenzwerte im Code angepasst werden:
  - `TEMP_FAN_ON`, `TEMP_FAN_OFF`
  - `PERSON_DIST_CM`
  - `FAN_PWM_MIN`

## 4.3 Displayinterpretation
- **Zeile 1:** Temperatur (und bei DHT11 auch Luftfeuchte)
- **Zeile 2:** Lüfterleistung in % und Distanz
- Sonderanzeigen:
  - `PERSON DETECTED!` → Lüfter aus
  - `SENSOR ERROR!` bzw. ERR-Werte → Failsafe aktiv

---

# 5) Zusammenfassung und Reflexion

## 5.1 Ergebnisse
- Das System erfüllt die Kernfunktion: temperaturabhängige Lüftersteuerung.
- Sicherheits- und Failsafe-Mechanismen erhöhen die Betriebssicherheit.
- Die modulare Struktur (Lesen, Regeln, Anzeigen) verbessert Wartbarkeit und Erweiterbarkeit.

## 5.2 Offene Punkte / Grenzen
- Ohne Kalibrierung können absolute Temperaturwerte je nach Sensor leicht abweichen.
- DHT11 ist relativ ungenau und langsam.
- HC-SR04 kann in engen/geometrisch ungünstigen Umgebungen Fehlmessungen erzeugen.

## 5.3 Ausblick (Version 2.0)
- Austausch DHT11 gegen DHT22 oder BME280 für genauere Messungen.
- Logging auf SD-Karte (Temperaturverlauf, Fehlerereignisse).
- Alarmfunktion (Buzzer/LED/Nachricht) bei Grenzwertüberschreitung.
- Web-Dashboard (z. B. per ESP32) für Fernüberwachung.
- Manuelle Betriebsmodi (Auto/Manuell/Service).

---

# Anhang

## A) Verwendete Quellcodedateien
- `serverroom_fan_control-TMP36.ino`
- `serverroom_fan_control_dht11.ino`

## B) Hinweis zur Abgabe als PDF
Vor der finalen Abgabe:
1. Teammitglieder und Betreuer ergänzen.
2. Inhaltsverzeichnis mit Seitenzahlen in der finalen Layout-Version erzeugen.
3. Schaltplan (Fritzing-Export) und PAP (draw.io-Export) als Abbildungen einfügen.
4. Dokument als PDF exportieren.
