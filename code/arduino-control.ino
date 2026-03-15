#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// ===========================================================================
// PROJEKT: Statistik-Teststrecke mit Kugelbahn
// ===========================================================================
// Hardware:
// - Arduino Uno
// - LCD 20x4 (I2C)
// - FSR-Sensor
// - 2x IR-Lichtschranken
// - Servo-Motor
//
// Funktion:
// - Misst die Zeit zwischen zwei Punkten
// - Berechnet die Geschwindigkeit
// - Misst die Aufprallkraft am Ende der Bahn
//
// Ausgabe:
// - LCD-Display (4 Nachkommastellen)
// - Serieller Monitor
// ===========================================================================

// ===========================================================================
// OBJEKTE
// ===========================================================================
Servo myServo;                     // Servo-Objekt zur Steuerung des Motors
LiquidCrystal_I2C lcd(0x27, 20, 4); // LCD-Adresse: 0x27, 20 Spalten, 4 Zeilen

// ===========================================================================
// PHYSIKALISCHE KONSTANTEN UND EINSTELLUNGEN
// ===========================================================================
const float DISTANCE_METERS = 0.81;      // Abstand zwischen den beiden Sensoren in Metern
const unsigned long minRaceTime = 200;   // Minimale Laufzeit zur Entprellung (ms)

// ===========================================================================
// KALIBRIERUNGSWERTE (BENUTZERDEFINIERT)
// Diese Werte wurden experimentell ermittelt, damit 100 g genau 1 N entsprechen
// ===========================================================================
const float ZERO_OFFSET = 2158.63;       // Nullpunkt-Korrektur
const float CALIBRATION_FACTOR = 2628.51; // Umrechnungsfaktor

// ===========================================================================
// BENUTZERDEFINIERTES ZEICHEN (Custom Char)
// Erstellt ein 'v' mit Querstrich darüber (Symbol für Durchschnittsgeschwindigkeit)
// ===========================================================================
byte v_quer[8] = {
  0b11111,
  0b00000,
  0b00000,
  0b10001,
  0b10001,
  0b01010,
  0b01010,
  0b00100
};

// ===========================================================================
// PIN-ZUWEISUNGEN (HARDWARE-KONFIGURATION)
// ===========================================================================
#define LEDPIN      13   // Optionale Status-LED
#define SENSOR1PIN  4    // Erster IR-Sensor (Start)
#define SENSOR2PIN  5    // Zweiter IR-Sensor (Ziel)
#define Button1PIN  2    // Taster 1 (Servo Position 1)
#define Button2PIN  7    // Taster 2 (Servo Position 2)
#define Button3PIN  8    // Taster 3 (Servo Position 3)
#define Button4PIN  12   // Taster 4 (System Reset)
#define FSR_PIN     A1   // Analoger Eingang für den Kraftsensor (FSR)

// ===========================================================================
// SYSTEMVARIABLEN
// ===========================================================================
const float R_DIV = 10000.0;   // Widerstand im Spannungsteiler (10 kOhm)

int dynamicThreshold = 0;      // Variabler Schwellenwert zur Rauschunterdrückung

int sensor1State = 0;
int lastSensor1State = 0;

int sensor2State = 0;
int lastSensor2State = 0;

int Button1State;
int Button2State;
int Button3State;
int Button4State;

// ===========================================================================
// ZEITMESSUNG
// ===========================================================================
bool timingActive = false;     // Läuft die Messung aktuell?
unsigned long startTime = 0;   // Startzeitpunkt der Messung

// ===========================================================================
// SETUP: INITIALISIERUNG DES SYSTEMS
// ===========================================================================
void setup() {
  // --- Pin-Konfiguration ---
  pinMode(LEDPIN, OUTPUT);

  pinMode(SENSOR1PIN, INPUT_PULLUP);
  pinMode(SENSOR2PIN, INPUT_PULLUP);

  pinMode(Button1PIN, INPUT_PULLUP);
  pinMode(Button2PIN, INPUT_PULLUP);
  pinMode(Button3PIN, INPUT_PULLUP);
  pinMode(Button4PIN, INPUT_PULLUP);

  pinMode(FSR_PIN, INPUT);

  // --- Serielle Kommunikation ---
  Serial.begin(9600);

  // --- LCD initialisieren ---
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, v_quer);

  // --- Startbildschirm ---
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("   System Bereit");
  lcd.setCursor(0, 2);
  lcd.print("   Warte auf Start");

  // --- Servo initialisieren ---
  myServo.attach(9);
  myServo.write(0);

  // --- Tabellenkopf für Seriellen Monitor (CSV-Format) ---
  Serial.println("Time(s), Speed(m/s), Force(N)");
}

// ===========================================================================
// LOOP: HAUPTPROGRAMMSCHLEIFE
// ===========================================================================
void loop() {
  // -------------------------------------------------------------------------
  // 1. Zustände aller Sensoren und Taster einlesen
  // -------------------------------------------------------------------------
  sensor1State = digitalRead(SENSOR1PIN);
  sensor2State = digitalRead(SENSOR2PIN);

  Button1State = digitalRead(Button1PIN);
  Button2State = digitalRead(Button2PIN);
  Button3State = digitalRead(Button3PIN);
  Button4State = digitalRead(Button4PIN);

  // -------------------------------------------------------------------------
  // 2. Servo-Steuerung für definierte Winkelpositionen
  // -------------------------------------------------------------------------
  if (Button1State == LOW) {
    myServo.write(0);
    delay(150);

    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Pos. 1: 50 Grad");
  }

  if (Button2State == LOW) {
    myServo.write(60);
    delay(150);

    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Pos. 2: 40 Grad");
  }

  if (Button3State == LOW) {
    myServo.write(120);
    delay(150);

    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Pos. 3: 35 Grad");
  }

  if (Button4State == LOW) {
    // Reset-Funktion: Servo zurücksetzen und Display bereinigen
    myServo.write(0);
    delay(150);

    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("   System Bereit");
    lcd.setCursor(0, 2);
    lcd.print("   Warte Auf Start");
  }

  // -------------------------------------------------------------------------
  // 3. Start der Zeitmessung (Trigger durch Sensor 1)
  // -------------------------------------------------------------------------
  if (sensor1State == LOW && lastSensor1State == HIGH && !timingActive) {
    startTime = millis();
    timingActive = true;

    digitalWrite(LEDPIN, HIGH);

    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print(">> Messung laeuft >>");
  }

  // -------------------------------------------------------------------------
  // 4. Stopp der Zeitmessung und Datenerfassung (Trigger durch Sensor 2)
  // -------------------------------------------------------------------------
  if (sensor2State == LOW && lastSensor2State == HIGH && timingActive) {
    // Sicherheitsabfrage: Mindestlaufzeit zur Vermeidung von Fehltriggern
    if ((millis() - startTime) > minRaceTime) {
      // ---------------------------------------------------------------------
      // A. Berechnung der Kinematik
      // ---------------------------------------------------------------------
      unsigned long stopTime = millis();
      unsigned long diffMs = stopTime - startTime;

      float diffSec = diffMs / 1000.0;
      float speedMPS = DISTANCE_METERS / diffSec;

      timingActive = false;
      digitalWrite(LEDPIN, LOW);

      // ---------------------------------------------------------------------
      // B. Vorbereitung der Kraftmessung
      // ---------------------------------------------------------------------
      int noiseLevel = analogRead(FSR_PIN);
      dynamicThreshold = noiseLevel + 30;

      if (dynamicThreshold < 100) {
        dynamicThreshold = 100;
      }

      // ---------------------------------------------------------------------
      // C. Daten sammeln (Mehrfachmessung)
      // Ziel: Erfassung des Aufpralls mit hoher Präzision
      // ---------------------------------------------------------------------
      unsigned long waitStart = millis();

      long totalRawSum = 0;
      int sampleCount = 0;
      int maxRawReading = 0;
      bool impactDetected = false;

      // Warten maximal 3 Sekunden auf einen Aufprall
      while (millis() - waitStart < 3000) {
        int currentReading = analogRead(FSR_PIN);

        // Wenn Schwellenwert überschritten wird -> Aufprall erkannt
        if (currentReading > dynamicThreshold) {
          impactDetected = true;
          unsigned long peakTimer = millis();

          // Erfassungsfenster: 50 ms lang Daten sammeln
          while (millis() - peakTimer < 50) {
            int r = analogRead(FSR_PIN);

            if (r > maxRawReading) {
              maxRawReading = r;
            }

            totalRawSum += r;
            sampleCount++;
          }

          break;
        }
      }

      // ---------------------------------------------------------------------
      // D. Physikalische Berechnung der Kraft (Newton)
      // ---------------------------------------------------------------------
      float forceNewtons = 0.0;

      if (impactDetected && sampleCount > 0) {
        float averageRaw = (float)totalRawSum / sampleCount;
        float finalRawValue = (maxRawReading + averageRaw) / 2.0;

        // Umrechnung Analogwert -> Spannung
        float voltageAtPin = finalRawValue * (5.0 / 1023.0);

        if (voltageAtPin > 0) {
          // Berechnung des FSR-Widerstands und Leitwerts
          float fsrResistance =
              (5000.0 - (voltageAtPin * 1000)) * R_DIV / (voltageAtPin * 1000);

          float fsrConductance = 1000000.0 / fsrResistance;

          // ---------------------------------------------------------------
          // Benutzerspezifische Kalibrierung
          // Formel: (Leitwert - Nullpunkt) / Kalibrierungsfaktor
          // ---------------------------------------------------------------
          float netCond = fsrConductance - ZERO_OFFSET;

          if (netCond < 0) {
            netCond = 0;
          }

          forceNewtons = netCond / CALIBRATION_FACTOR;
        }
      }

      // ---------------------------------------------------------------------
      // E. Ausgabe der Ergebnisse auf dem LCD
      // ---------------------------------------------------------------------
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("--- Ergebnisse ---");

      // Zeile 2: Zeit
      lcd.setCursor(0, 1);
      lcd.print("t = ");
      lcd.print(diffSec, 4);
      lcd.print(" s");

      // Zeile 3: Geschwindigkeit
      lcd.setCursor(0, 2);
      lcd.write(0);  // v-quer Symbol
      lcd.print(" = ");
      lcd.print(speedMPS, 4);
      lcd.print(" m/s");

      // Zeile 4: Kraft
      lcd.setCursor(0, 3);
      lcd.print("F = ");

      if (impactDetected) {
        lcd.print(forceNewtons, 4);
        lcd.print(" N");
      } else {
        lcd.print("--- (Kein Hit)");
      }

      // ---------------------------------------------------------------------
      // F. Ausgabe auf dem Seriellen Monitor (CSV-Format)
      // ---------------------------------------------------------------------
      Serial.print(diffSec, 4);
      Serial.print(", ");

      Serial.print(speedMPS, 4);
      Serial.print(", ");

      Serial.println(forceNewtons, 4);
    }
  }

  // -------------------------------------------------------------------------
  // 5. Zustände für nächsten Schleifendurchlauf speichern
  // -------------------------------------------------------------------------
  lastSensor1State = sensor1State;
  lastSensor2State = sensor2State;
}
