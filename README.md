# Serverraumüberwachung mit automatischer Lüftersteuerung

Dieses Repository enthält ein Arduino-Projekt zur Überwachung eines Serverraums mit temperaturabhängiger Lüftersteuerung, Abstandserkennung und LCD-Anzeige.

## Projektziel

Ziel ist eine robuste, leicht nachvollziehbare Steuerung, die:
- Temperatur kontinuierlich überwacht,
- den Lüfter per PWM regelt,
- bei Personenerkennung in Lüfternähe den Lüfter stoppt,
- bei Sensorfehlern automatisch in einen Failsafe-Modus geht.

## Aktueller Projektstand (Dateien im Repository)

### Arduino-Sketches
1. `serverroom_fan_control-TMP36.ino`
   - Sensorik: **TMP36** (Temperatur, analog) + **HC-SR04** (Abstand)
   - Anzeige: **I2C LCD 16x2**
   - Logik: Temperatur-Hysterese, PWM-Mapping, Sonar-Failsafe

2. `serverroom_fan_control_dht11.ino`
   - Sensorik: **DHT11** (Temperatur + Luftfeuchte) + **HC-SR04** (Abstand)
   - Anzeige: **I2C LCD 16x2**
   - Logik: Temperatur-Hysterese, PWM-Mapping, DHT/Sonar-Failsafe

> Hinweis: In älteren Dokumentationen werden teilweise andere Dateinamen genannt. Maßgeblich sind die oben genannten tatsächlich vorhandenen Sketch-Dateien.

## Verwendete Hardware

- Arduino Mega 2560
- 5V PWM-Lüfter
- TMP36 **oder** DHT11
- HC-SR04 Ultraschallsensor
- I2C LCD (16x2, häufig Adresse `0x27`, alternativ `0x3F`)
- Jumper-Kabel / Breadboard

## Benötigte Libraries

Über den Arduino Library Manager installieren:
- `LiquidCrystal_I2C` (Frank de Brabander)
- `DHT sensor library` (Adafruit) *(nur für DHT11-Sketch)*
- `Adafruit Unified Sensor` *(Abhängigkeit der DHT-Library)*

## Kernlogik

- Nicht-blockierende Zeitsteuerung mit `millis()`
- Temperaturglättung per Moving Average
- Hysterese mit `TEMP_FAN_ON` und `TEMP_FAN_OFF`
- Personenerkennung über Distanzschwelle (`PERSON_DIST_CM`)
- Failsafe:
  - TMP36-Sketch: bei Sonarfehler Lüfter auf 100 %
  - DHT11-Sketch: bei DHT- **oder** Sonarfehler Lüfter auf 100 %

## Schnellstart

1. Hardware aufbauen und verkabeln.
2. Passenden Sketch öffnen:
   - `serverroom_fan_control-TMP36.ino` **oder**
   - `serverroom_fan_control_dht11.ino`
3. Bibliotheken installieren.
4. Sketch auf den Arduino Mega 2560 hochladen.
5. LCD prüfen (bei leerem Display I2C-Adresse `0x27`/`0x3F` testen).

## Projektstruktur

```text
.
├── README.md
├── LICENSE
├── Dokumentation-als-PDF.txt
├── doc/
│   └── dokumenation.md
├── serverroom_fan_control-TMP36.ino
├── serverroom_fan_control_dht11.ino
├── serverraumüberwachung.fzz
├── IMG_2060.jpeg
├── IMG_2066.jpeg
└── IMG_2067.jpeg
```

## Dokumentation & Medien

- Ausführliche Projektdokumentation: `doc/dokumenation.md`
- Hinweis zur PDF-Abgabe: `Dokumentation-als-PDF.txt`
- Fritzing-Datei: `serverraumüberwachung.fzz`
- Aufbaufotos:
  - `IMG_2060.jpeg`
  - `IMG_2066.jpeg`
  - `IMG_2067.jpeg`

## Bilder

![Aufbau 1](./IMG_2060.jpeg)
![Aufbau 2](./IMG_2066.jpeg)
![Aufbau 3](./IMG_2067.jpeg)
