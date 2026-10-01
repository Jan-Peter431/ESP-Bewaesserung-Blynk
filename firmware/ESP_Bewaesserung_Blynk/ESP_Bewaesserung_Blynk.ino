#define BLYNK_TEMPLATE_ID "TMPL4YJh3kbXd"
#define BLYNK_TEMPLATE_NAME "ESP32 Bewaesserung"
#include "secrets.h"
#define BLYNK_PRINT Serial

#include <Arduino.h>
#include <BlynkSimpleEsp32.h>
#include <time.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MAX1704X.h>
#include <esp_sleep.h>
#include <Adafruit_NeoPixel.h>


// ------------------------------------------------------------
// WLAN-DATEN
// ------------------------------------------------------------

char ssid[] = WIFI_SSID;
char pass[] = WIFI_PASSWORD;

/************************************************************

************************************************************/
#define FW_VERSION "2026-08-28_MS8 final"


/************************************************************
  Pinbelegung ESP32
************************************************************/

// Bodenfeuchtesensoren, analog
const int PIN_MOISTURE_1 = 4;
const int PIN_MOISTURE_2 = 5;
const int PIN_MOISTURE_3 = 6;
// DS18B20 Temperatur DATA
const int PIN_DS18B20 = 7;
// LED Ring
const int PIN_LED_RING = 8;
// MAX17048 BAT Monitor
const int PIN_I2C_SCL = 13;
const int PIN_I2C_SDA = 14;
// Sensor Powerschalter 5V über EN
const int PIN_SENSOR_POWER = 15;
// Ultraschall-Wasserstandssensor UART
const int PIN_WATER_RX = 16;   // ESP32 empfängt Sensor-TX
// Solarinterupt
const int PIN_SOLAR_CTRL = 17;
// Pumpe über MOSFETpp6 
const int PIN_PUMP = 18;



/************************************************************
  Blynk Virtual Pins
************************************************************/
// Messwerte Feuchte / Temperatur / Zeit
#define VPIN_MOISTURE_1              V1
#define VPIN_MOISTURE_2              V2
#define VPIN_MOISTURE_3              V3
#define VPIN_MOISTURE_AVG            V4
#define VPIN_RAW_MOISTURE_1          V5
#define VPIN_RAW_MOISTURE_2          V6
#define VPIN_RAW_MOISTURE_3          V7
#define VPIN_TEMPERATURE             V8
#define VPIN_UNIX_TIME               V9

// Bedienung / Betriebsparameter
#define VPIN_MODE                    V10
#define VPIN_MANUAL_PUMP             V11
#define VPIN_FIXED_DURATION_MIN      V12
#define VPIN_FIXED_FREQUENCY         V13
#define VPIN_START_FIRST_HOUR        V14
#define VPIN_START_SECOND_HOUR       V15
#define VPIN_AUTO_MIN_MOISTURE       V16
#define VPIN_AUTO_MAX_MOISTURE       V17
#define VPIN_DATE_TIME_TEXT          V18
#define VPIN_BLYNK_SEND_INTERVAL_MS  V19

// Wasser / Pumpe / Sicherheit
#define VPIN_PUMP_COOLING_DURATION   V20
#define VPIN_WATER_RAW               V21
#define VPIN_WATER_LITERS            V22
#define VPIN_WATER_PUMP_RELEASE      V23
#define VPIN_WATER_SUPPLY_ERROR      V24
#define VPIN_WATER_ERROR_RESET       V25
#define VPIN_MAX_RUNTIME_MIN         V26

#define VPIN_PUMP_STATE              V30

// System
#define VPIN_SYSTEM_MODE_TEXT        V31
#define VPIN_SYSTEM_MODE_NUMBER      V32
#define VPIN_SYSTEM_STATE_NUMBER     V33
#define VPIN_SYSTEM_STATE_TEXT       V34
#define VPIN_OPERATION_MODE          V35

// Messaging
#define VPIN_MESSAGING_ACTIVE        V37
#define VPIN_MESSAGING_WATER_WARNING_PCT V38

// Kalibrierung Wasser
#define VPIN_WATER_FULL_L            V75
#define VPIN_WATER_EMPTY_RAW         V76
#define VPIN_WATER_FULL_RAW          V77
#define VPIN_SET_WATER_EMPTY_RAW     V78
#define VPIN_SET_WATER_FULL_RAW      V79

// Kalibrierung Feuchte
#define VPIN_M1_DRY                  V80
#define VPIN_M1_WET                  V81
#define VPIN_M2_DRY                  V82
#define VPIN_M2_WET                  V83
#define VPIN_M3_DRY                  V84
#define VPIN_M3_WET                  V85
#define VPIN_SET_M1_DRY              V86
#define VPIN_SET_M1_WET              V87
#define VPIN_SET_M2_DRY              V88
#define VPIN_SET_M2_WET              V89

#define VPIN_SET_M3_DRY              V90
#define VPIN_SET_M3_WET              V91
#define VPIN_BATTERY_VOLTAGE         V92
#define VPIN_BATTERY_PERCENT         V93
#define VPIN_BATTERY_MONITOR_OK      V94
#define VPIN_DEV_MODE                V95
#define VPIN_WAKE_INTERVAL_MIN       V96
#define VPIN_NIGHT_SLEEP_START       V97
#define VPIN_NIGHT_SLEEP_END         V98
#define VPIN_MOISTURE_CONTROL_MODE   V99

/************************************************************
  Konstanten / Typen
************************************************************/

//LED Anzahl
const int LED_RING_COUNT = 12;

enum UserMode {
  MODE_MANUAL = 0,
  MODE_FIXED  = 1,
  MODE_AUTO   = 2
};

enum AutoPhase {
  AUTO_IDLE,
  AUTO_WATERING,
  AUTO_PAUSING
};

enum MoistureControlMode {
  MOISTURE_FS1 = 0,
  MOISTURE_FS2 = 1,
  MOISTURE_FS3 = 2,
  MOISTURE_AVG = 3
};

enum OperationMode {
  OPERATION_INDOOR  = 0,
  OPERATION_OUTDOOR = 1
};

/************************************************************
  Telegram Bot
************************************************************/

const char* TELEGRAM_BOT_TOKEN = TELEGRAM_BOT_TOKEN_VALUE;
const char* TELEGRAM_CHAT_ID   = TELEGRAM_CHAT_ID_VALUE;

bool messagingActive = false;
bool waterLowTelegramSent = false;
bool devMode = false;


int messagingWaterWarningPct = 20;
const int MESSAGING_WATER_RESET_HYSTERESIS_PCT = 5;

const float BATTERY_ALARM_PCT = 5.0f;         // Unter 5% Batterieladestand Alarm senden
const float BATTERY_ALARM_RESET_PCT = 8.0f;   // Hysterese bis 8%

uint8_t batteryReadErrorCount = 0;
const uint8_t BATTERY_MAX_READ_ERRORS = 3;

// Überlebt Deep Sleep
RTC_DATA_ATTR bool batteryLowTelegramSent = false;

/************************************************************
  Globale Variablen
************************************************************/

BlynkTimer timer;

MoistureControlMode moistureControlMode = MOISTURE_AVG;

// ----------------------------------------------------------
// Betriebsart V35
// 0 = Indoor / Netz
// 1 = Outdoor / Solar + Batterie
// ----------------------------------------------------------

OperationMode operationMode = OPERATION_OUTDOOR;

// Erst true, nachdem V35 von Blynk empfangen wurde.
// Solange false -> kein Deep Sleep und kein Leuchtring.
bool operationModeSynced = false;

OneWire oneWire(PIN_DS18B20);
DallasTemperature tempSensor(&oneWire);
HardwareSerial WaterSerial(1);

Adafruit_MAX17048 batteryMonitor;

bool batteryMonitorOk = false;

float batteryVoltage = NAN;
float batteryPercent = NAN;

UserMode currentMode = MODE_MANUAL;
AutoPhase autoPhase = AUTO_IDLE;

bool pumpOn = false;
bool manualPumpRequest = false;

// Messwerte roh
int rawMoisture1 = 0;
int rawMoisture2 = 0;
int rawMoisture3 = 0;
int rawWaterLevel = 0;
bool waterLevelValid = false;
const unsigned long WATER_SENSOR_STALE_MS = 10000UL;

// Messwerte berechnet
float moisture1Pct = 0.0;
float moisture2Pct = 0.0;
float moisture3Pct = 0.0;
float moistureAvgPct = 0.0;
float waterLiters = 0.0;
float temperatureC = NAN;

// Kalibrierwerte Feuchte
// Achtung: Bei kapazitiven Sensoren ist trocken meistens hoher Rohwert,
// nass meistens niedriger Rohwert.
int m1DryRaw = 3000;
int m1WetRaw = 1200;

int m2DryRaw = 3000;
int m2WetRaw = 1200;

int m3DryRaw = 3000;
int m3WetRaw = 1200;

// Kalibrierwerte Wasserstand
int waterEmptyRaw = 800;
int waterFullRaw = 100;
float waterFullLiters = 60.0;

// Parameter Fixed-Automatik
float fixedDurationMin = 5.0;
int fixedFrequencyMode = 1;        // 0 = 2x täglich, 1 = täglich, 2 = alle 2 Tage, 3 = wöchentlich
int startfirstHour = 6;
int startsecondHour = 18;

bool fixedCycleActive = false;
unsigned long fixedCycleStartMillis = 0;

int lastFixedfirstDayKey = -1;
int lastFixedsecondDayKey = -1;
int lastFixedEvery2DayKey = -1;
int lastFixedWeeklyDayKey = -1;

const int FIXED_START_WINDOW_MIN = 5;

unsigned long blynkSendIntervalMs = 1000;
unsigned long lastBlynkSendMillis = 0;

// Parameter Vollautomatik
float autoStartMoisturePct = 35.0;
float autoStopMoisturePct = 55.0;
float pumpCoolingDurationMin = 30.0;

// Sicherheit
float maxRuntimeMin = 20;

// Zeitsteuerung
unsigned long pumpStartMillis = 0;
unsigned long autoPhaseStartMillis = 0;

// Status
String systemStateText = "System startet";
String errorText = "";
const String WATER_WARNING_TEXT = "Warnung: Wasserstand unter Mindestgrenze";
const String WATER_SENSOR_ERROR_TEXT = "Fehler: Kein gueltiger Wasserstandswert";


// Solar interupt
bool solarResetThisBoot = false;
RTC_DATA_ATTR uint32_t solarResetCounter = 0;

const uint8_t LED_RING_BRIGHTNESS = 40;

Adafruit_NeoPixel ring(
  LED_RING_COUNT,
  PIN_LED_RING,
  NEO_GRB + NEO_KHZ800
);

bool ringInitialized = false;

bool isNightTime(int hour);

void ringOff();
void ringSetColor(uint8_t r, uint8_t g, uint8_t b);
void updateRingStatus();

void updateWidgetAvailability();

//Systemzeit prüfen
bool systemTimeValid()
{
  return time(nullptr) >= 1700000000;
}

bool isNightNow()
{
  time_t now = time(nullptr);

  if (now < 1700000000) {
    return false;
  }

  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  return isNightTime(timeinfo.tm_hour);
}

// ------------------------------------------------------------
// Schlafsteuerung
// ------------------------------------------------------------

int wakeIntervalMin = 60;
int nightSleepStartHour = 23;
int nightSleepEndHour = 5;

// Sensoren nach dem Einschalten stabilisieren lassen
const unsigned long SENSOR_STABILIZE_MS = 2000;

// Synchronisationsstatus Blynk
bool devModeSynced = false;
bool wakeIntervalSynced = false;
bool nightStartSynced = false;
bool nightEndSynced = false;

bool sleepConfigReady()
{
  return devModeSynced &&
         wakeIntervalSynced &&
         nightStartSynced &&
         nightEndSynced;
}

bool isNightTime(int hour)
{
    if (nightSleepStartHour == nightSleepEndHour) {
        return false;   // zunächst: Nachtmodus deaktiviert
    }

    if (nightSleepStartHour < nightSleepEndHour) {
        // z.B. 01:00 bis 05:00
        return hour >= nightSleepStartHour &&
               hour < nightSleepEndHour;
    }
    else {
        // z.B. 23:00 bis 05:00
        return hour >= nightSleepStartHour ||
               hour < nightSleepEndHour;
    }
}

// Nach jedem Blynk-Connect mindestens 30 Sekunden wach bleiben.
// In dieser Zeit ist auch manuelle Bedienung möglich.
const unsigned long WAKE_ONLINE_WINDOW_MS = 30000UL;

unsigned long blynkConnectedMillis = 0;

// Mindestens ein kompletter Mess-/Steuerzyklus muss erfolgt sein,
// bevor geschlafen werden darf.
bool firstControlCycleDone = false;

/************************************************************
  LED Ring Funktionen
************************************************************/

void ringSetColor(uint8_t r, uint8_t g, uint8_t b)
{
  if (!ringInitialized) {
    return;
  }

  uint32_t color = ring.Color(r, g, b);

  for (int i = 0; i < LED_RING_COUNT; i++) {
    ring.setPixelColor(i, color);
  }

  ring.show();
}


void ringOff()
{
  if (!ringInitialized) {
    return;
  }

  ring.clear();
  ring.show();
}


void updateRingStatus()
{
  if (!ringInitialized) {
    return;
  }

  // V35 noch nicht bekannt
  if (!operationModeSynced) {
    ringOff();
    return;
  }

  // OUTDOOR: Ring grundsätzlich AUS
  if (operationMode == OPERATION_OUTDOOR) {
    ringOff();
    return;
  }

  // --------------------------------------------------------
  // Ab hier ausschließlich INDOOR
  // Priorität:
  // Fehler > Wasserwarnung > Pumpe > Start/Warten > OK
  // --------------------------------------------------------

  // Allgemeiner Fehler = ROT
  if (errorText.length() > 0 &&
      errorText != WATER_WARNING_TEXT)
  {
    ringSetColor(255, 0, 0);
    return;
  }

  // Wasser niedrig = GELB
  if (errorText == WATER_WARNING_TEXT)
  {
    ringSetColor(255, 160, 0);
    return;
  }

  // Pumpe läuft = BLAU
  if (pumpOn)
  {
    ringSetColor(0, 0, 255);
    return;
  }

  // System startet / wartet = ORANGE
  if (systemStateText == "System startet" ||
      systemStateText.indexOf("Warte") >= 0 ||
      systemStateText.indexOf("warte") >= 0)
  {
    ringSetColor(255, 80, 0);
    return;
  }

  // Normalbetrieb = GRÜN
  ringSetColor(0, 255, 0);
}




/************************************************************
  Hilfsfunktionen
************************************************************/

float getControlMoisturePct()
{
  switch (moistureControlMode)
  {
    case MOISTURE_FS1:
      return moisture1Pct;

    case MOISTURE_FS2:
      return moisture2Pct;

    case MOISTURE_FS3:
      return moisture3Pct;

    case MOISTURE_AVG:
    default:
      return moistureAvgPct;
  }
}

String moistureControlModeToText()
{
  switch (moistureControlMode)
  {
    case MOISTURE_FS1:
      return "FS1";

    case MOISTURE_FS2:
      return "FS2";

    case MOISTURE_FS3:
      return "FS3";

    case MOISTURE_AVG:
      return "Mittelwert";

    default:
      return "Unbekannt";
  }
}

bool sendTelegramMessage(const String& message)
{
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[TELEGRAM] Nicht gesendet: WLAN nicht verbunden");
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;

  String url = "https://api.telegram.org/bot";
  url += TELEGRAM_BOT_TOKEN;
  url += "/sendMessage";

  if (!https.begin(client, url)) {
    Serial.println("[TELEGRAM] HTTPS begin fehlgeschlagen");
    return false;
  }

  https.addHeader(
    "Content-Type",
    "application/x-www-form-urlencoded"
  );

  String payload =
      "chat_id=" + String(TELEGRAM_CHAT_ID) +
      "&text=" + message;

  int httpCode = https.POST(payload);

  String response = https.getString();

  Serial.print("[TELEGRAM] HTTP Code: ");
  Serial.println(httpCode);

  Serial.print("[TELEGRAM] Antwort: ");
  Serial.println(response);

  bool success =
      (httpCode == 200) &&
      (response.indexOf("\"ok\":true") >= 0);

  https.end();

  if (success) {
    Serial.println("[TELEGRAM] Nachricht erfolgreich gesendet");
  }
  else {
    Serial.println("[TELEGRAM] Versand FEHLGESCHLAGEN");
  }

  return success;
}

void prepareSensorPowerForDeepSleep()
{
  // Sensor-Boost abschalten
  pinMode(PIN_SENSOR_POWER, OUTPUT);
  digitalWrite(PIN_SENSOR_POWER, LOW);

  // Kurz warten, damit EN sicher LOW ist
  delay(10);

  // Aktuellen Zustand von GPIO15 festhalten
  gpio_hold_en((gpio_num_t)PIN_SENSOR_POWER);

  // Hold auch während Deep Sleep aktivieren
  gpio_deep_sleep_hold_en();
}

void checkBatteryAlarm()
{
  if (!batteryMonitorOk || isnan(batteryPercent)) {
    return;
  }

  // Alarm wieder freigeben, wenn Akku deutlich erholt ist
  if (batteryPercent >= BATTERY_ALARM_RESET_PCT)
  {
    if (batteryLowTelegramSent) {
      Serial.println("[BAT] Batteriealarm wieder freigegeben");
    }

    batteryLowTelegramSent = false;
  }

  // Messaging ausgeschaltet?
  if (!messagingActive) {
    return;
  }

  // Alarm wurde bereits erfolgreich gesendet?
  if (batteryLowTelegramSent) {
    return;
  }

  // Batterie kritisch?
  if (batteryPercent <= BATTERY_ALARM_PCT)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println(
        "[BAT] Batteriealarm: WLAN nicht verbunden"
      );

      return;
    }

    String message =
      "WARNUNG ESP32 Bewaesserung: Batterie kritisch! "
      "Ladezustand nur noch " +
      String(batteryPercent, 1) +
      " %. Batteriespannung: " +
      String(batteryVoltage, 3) +
      " V.";

    bool sent = sendTelegramMessage(message);

    if (sent)
    {
      batteryLowTelegramSent = true;

      Serial.println(
        "[BAT] Batteriealarm erfolgreich gesendet"
      );
    }
    else
    {
      Serial.println(
        "[BAT] Batteriealarm nicht gesendet -> neuer Versuch folgt"
      );
    }
  }
}


void publishModeToBlynk() {
  if (!Blynk.connected()) {
    return;
  }

  Blynk.virtualWrite(VPIN_SYSTEM_MODE_TEXT, modeToText(currentMode));
  Blynk.virtualWrite(VPIN_SYSTEM_MODE_NUMBER, (int)currentMode);
}

void publishWaterCalibrationValues() {
  if (!Blynk.connected()) {
    return;
  }

  Blynk.virtualWrite(VPIN_WATER_FULL_L, waterFullLiters);
  Blynk.virtualWrite(VPIN_WATER_EMPTY_RAW, waterEmptyRaw);
  Blynk.virtualWrite(VPIN_WATER_FULL_RAW, waterFullRaw);
}

void updateWidgetAvailability() {
  if (!Blynk.connected()) {
    return;
  }

  bool manualMode = currentMode == MODE_MANUAL;
  bool fixedMode  = currentMode == MODE_FIXED;
  bool autoMode   = currentMode == MODE_AUTO;

  bool indoorMode =
    operationModeSynced &&
    operationMode == OPERATION_INDOOR;

  bool waterOk = hasWater();
  bool manualPumpAllowed = manualMode && waterOk;

  Blynk.setProperty(VPIN_MANUAL_PUMP, "isDisabled", !manualPumpAllowed);

  if (!manualPumpAllowed) {
    manualPumpRequest = false;
    Blynk.virtualWrite(VPIN_MANUAL_PUMP, 0);
  }

  // ----------------------------------------------------------
// Deep-Sleep Aktivierung / Deaktivierung
//
// Indoor:
// V95 ausgegraut, da Deep Sleep grundsätzlich deaktiviert.
//
// Outdoor:
// V95 wieder normal bedienbar.
// ----------------------------------------------------------
Blynk.setProperty(
  VPIN_DEV_MODE,
  "isDisabled",
  indoorMode
);

Blynk.setProperty(
  VPIN_WAKE_INTERVAL_MIN,
  "isDisabled",
  indoorMode
);

Blynk.setProperty(
  VPIN_NIGHT_SLEEP_START,
  "isDisabled",
  indoorMode
);

Blynk.setProperty(
  VPIN_NIGHT_SLEEP_END,
  "isDisabled",
  indoorMode
);

  // Gießdauer gilt für Fixed und Vollautomatik
  Blynk.setProperty(VPIN_FIXED_DURATION_MIN, "isDisabled", !(fixedMode || autoMode));

  // Fixed-spezifisch
  Blynk.setProperty(VPIN_FIXED_FREQUENCY, "isDisabled", !fixedMode);
  Blynk.setProperty(VPIN_START_FIRST_HOUR, "isDisabled", !fixedMode);

  // Zweite Gießzeit nur bei Fixed + 2x täglich aktiv
  bool secondTimeAllowed = fixedMode && (fixedFrequencyMode == 0);
  Blynk.setProperty(VPIN_START_SECOND_HOUR, "isDisabled", !secondTimeAllowed);

  // Vollauto-spezifisch
  Blynk.setProperty(VPIN_AUTO_MIN_MOISTURE, "isDisabled", !autoMode);
  Blynk.setProperty(VPIN_AUTO_MAX_MOISTURE, "isDisabled", !autoMode);
  Blynk.setProperty(VPIN_PUMP_COOLING_DURATION, "isDisabled", !autoMode);

  // Grundeinstellungen immer bedienbar
    Blynk.setProperty(VPIN_MAX_RUNTIME_MIN, "isDisabled", false);

/*
  Serial.println("Widget-Verfügbarkeit aktualisiert:");
  Serial.print("  Modus: ");
  Serial.println(modeToText(currentMode));
  Serial.print("  Wasser OK: ");
  Serial.println(waterOk ? "JA" : "NEIN");
  Serial.print("  Manuelle Pumpe erlaubt: ");
  Serial.println(manualPumpAllowed ? "JA" : "NEIN");
  */
}

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

float mapRawToPercent(int raw, int dryRaw, int wetRaw) {
  if (dryRaw == wetRaw) {
    return 0.0;
  }

  // trocken = 0 %, nass = 100 %
  float pct = ((float)(dryRaw - raw) * 100.0) / ((float)(dryRaw - wetRaw));
  return clampFloat(pct, 0.0, 100.0);
}

bool hasWater()
{
  // Ohne gültige Wasserstandsmessung keine Pumpenfreigabe.
  if (!waterLevelValid) {
    return false;
  }

  // Ohne gültiges Maximalvolumen keine sichere Prozentberechnung.
  if (waterFullLiters <= 0.1f) {
    return false;
  }

  // Aktuellen Wasserstand in Prozent berechnen.
  float waterLevelPct =
      (waterLiters * 100.0f) / waterFullLiters;

  // Wasser ist ausreichend, solange der Füllstand
  // ÜBER der eingestellten Warnschwelle V38 liegt.
  return waterLevelPct > (float)messagingWaterWarningPct;
}

String modeToText(UserMode mode) {
  switch (mode) {
    case MODE_MANUAL: return "Manuell";
    case MODE_FIXED:  return "Fix-Automatik";
    case MODE_AUTO:   return "Vollautomatik";
    default:          return "Unbekannt";
  }
}

void setSystemState(String text)
{
  systemStateText = text;

  if (Blynk.connected()) {
    Blynk.virtualWrite(
      VPIN_SYSTEM_STATE_TEXT,
      systemStateText
    );

    Blynk.virtualWrite(
      VPIN_SYSTEM_STATE_NUMBER,
      0
    );
  }

  updateRingStatus();
}


void setError(String text)
{
  errorText = text;

  if (Blynk.connected() && text.length() > 0) {
    Blynk.virtualWrite(
      VPIN_SYSTEM_STATE_TEXT,
      text
    );

    Blynk.virtualWrite(
      VPIN_SYSTEM_STATE_NUMBER,
      100
    );
  }

  updateRingStatus();
}


void clearError()
{
  errorText = "";
  setSystemState("System bereit");
}

float getWaterLevelPercent() {
  if (waterFullLiters <= 0.1) {
    return 0.0;
  }

  float percent = (waterLiters * 100.0) / waterFullLiters;
  return clampFloat(percent, 0.0, 100.0);
}

void updateWaterWarningStatus()
{
  // --------------------------------------------------------
  // Fall 1: Wasserstandssensor liefert keinen gültigen Wert
  // --------------------------------------------------------
  if (!waterLevelValid)
  {
    if (errorText != WATER_SENSOR_ERROR_TEXT)
    {
      setError(WATER_SENSOR_ERROR_TEXT);
    }

    updateWidgetAvailability();
    return;
  }

  float waterLevelPct = getWaterLevelPercent();

  // --------------------------------------------------------
  // Fall 2: Wasserstand unter oder gleich Warnschwelle V38
  // --------------------------------------------------------
  if (waterLevelPct <= (float)messagingWaterWarningPct)
  {
    if (errorText != WATER_WARNING_TEXT)
    {
      setError(WATER_WARNING_TEXT);
    }

    updateWidgetAvailability();
  }
  else
  {
    if (errorText == WATER_WARNING_TEXT ||
        errorText == WATER_SENSOR_ERROR_TEXT)
    {
      clearError();
    }

    updateWidgetAvailability();
  }

  // --------------------------------------------------------
  // Telegram-Wasserwarnung
  // --------------------------------------------------------
  if (messagingActive)
  {
    if (!waterLowTelegramSent &&
        waterLevelPct <= (float)messagingWaterWarningPct)
    {
      waterLowTelegramSent = true;

      sendTelegramMessage(
        "ESP32 Bewaesserung: Wasserstand niedrig. Aktuell ca. " +
        String(waterLevelPct, 1) +
        " %. Warnschwelle: " +
        String(messagingWaterWarningPct) +
        " %."
      );
    }

    if (waterLowTelegramSent &&
        waterLevelPct >
          (float)messagingWaterWarningPct +
          MESSAGING_WATER_RESET_HYSTERESIS_PCT)
    {
      waterLowTelegramSent = false;

      sendTelegramMessage(
        "ESP32 Bewaesserung: Wasserstand wieder ausreichend. Aktuell ca. " +
        String(waterLevelPct, 1) +
        " %."
      );
    }
  }
  else
  {
    waterLowTelegramSent = false;
  }
}

void sensorPowerOn()
{
  // Alte UART-Daten VOR dem Einschalten entfernen
  while (WaterSerial.available() > 0) {
    WaterSerial.read();
  }

  digitalWrite(PIN_SENSOR_POWER, HIGH);

  Serial.println("[POWER] Sensorversorgung EIN");

  delay(SENSOR_STABILIZE_MS);

}


// ----------------------------------------------------------
// 5 V Versorgung der Sensoren abschalten
// ----------------------------------------------------------

void sensorPowerOff()
{
  digitalWrite(PIN_SENSOR_POWER, LOW);

  Serial.println("[POWER] Sensorversorgung AUS");
}

bool canEnterDeepSleep()
{
  // ----------------------------------------------------------
  // Betriebsart V35 muss zuerst bekannt sein.
  //
  // Solange V35 nach Boot / Reconnect noch nicht von Blynk
  // synchronisiert wurde, darf der ESP32 NICHT schlafen.
  // ----------------------------------------------------------
  if (!operationModeSynced) {
    return false;
  }

  // ----------------------------------------------------------
  // INDOOR / NETZBETRIEB:
  // Deep Sleep grundsätzlich gesperrt.
  // ----------------------------------------------------------
  if (operationMode == OPERATION_INDOOR) {
    return false;
  }

  // ----------------------------------------------------------
  // OUTDOOR:
  // Entwicklungsmodus sperrt Deep Sleep weiterhin wie bisher.
  // ----------------------------------------------------------
  if (devMode) {
    return false;
  }

  // Schlafparameter müssen aus Blynk angekommen sein
  if (!sleepConfigReady()) {
    return false;
  }

  // Ohne gültige Uhrzeit nicht schlafen.
// Sonst kann der Nachtmodus nicht sicher berechnet werden.
if (!systemTimeValid()) {
  return false;
}

  // Mindestens ein kompletter Steuerzyklus
  if (!firstControlCycleDone) {
    return false;
  }

  // Nach Verbindung zunächst Bedien-/Onlinefenster offen halten
  if (blynkConnectedMillis == 0) {
    return false;
  }

  if (millis() - blynkConnectedMillis < WAKE_ONLINE_WINDOW_MS) {
    return false;
  }

  // Niemals während Pumpenbetrieb schlafen
  if (pumpOn) {
    return false;
  }

  if (fixedCycleActive) {
    return false;
  }

  // Vollautomatik läuft gerade
  if (autoPhase != AUTO_IDLE) {
    return false;
  }

  // ----------------------------------------------------------
  // VORÜBERGEHEND:
  // Fixed-Modus noch wach lassen.
  //
  // Erst wenn wir die Fixed-Giesszeiten V14/V15 in die
  // Wake-Berechnung einbezogen haben, geben wir Deep Sleep
  // auch für MODE_FIXED frei.
  // ----------------------------------------------------------
  if (currentMode == MODE_FIXED) {
    return false;
  }

  return true;
}

time_t getNextHourOccurrence(int hour, time_t referenceTime)
{
  struct tm targetTime;
  localtime_r(&referenceTime, &targetTime);

  targetTime.tm_hour = hour;
  targetTime.tm_min  = 0;
  targetTime.tm_sec  = 0;

  time_t result = mktime(&targetTime);

  // Zeitpunkt heute bereits vorbei?
  if (result <= referenceTime) {
    targetTime.tm_mday += 1;
    result = mktime(&targetTime);
  }

  return result;
}

time_t getNextRegularWake(time_t now)
{
  struct tm localNow;
  localtime_r(&now, &localNow);

  int intervalMin = wakeIntervalMin;

  if (intervalMin < 1) {
    intervalMin = 1;
  }

  if (intervalMin > 1440) {
    intervalMin = 1440;
  }

  int minutesSinceMidnight =
      localNow.tm_hour * 60 +
      localNow.tm_min;

  // Nächstes festes Raster
  int nextMinute =
      ((minutesSinceMidnight / intervalMin) + 1) *
      intervalMin;

  struct tm target = localNow;

  target.tm_hour = 0;
  target.tm_min  = 0;
  target.tm_sec  = 0;

  // nextMinute kann über Mitternacht hinausgehen
  target.tm_mday += nextMinute / 1440;
  target.tm_min   = nextMinute % 1440;

  return mktime(&target);
}

uint32_t calculateSleepSeconds()
{
  time_t now = time(nullptr);

  if (!systemTimeValid()) {
  Serial.println("[SLEEP] Uhrzeit ungueltig -> Deep Sleep blockiert");
  return 0;
 }

  // Uhrzeit noch nicht gültig:
  // als Fallback normales Wake-Intervall verwenden.
  if (now < 1700000000) {
    Serial.println(
      "[SLEEP] Uhrzeit ungueltig -> Intervall-Fallback"
    );

    return (uint32_t)wakeIntervalMin * 60UL;
  }

  struct tm localNow;
  localtime_r(&now, &localNow);

  time_t targetWake;

  bool nightModeEnabled =
      nightSleepStartHour != nightSleepEndHour;

  // ----------------------------------------------------------
  // Wir befinden uns bereits in der Nachtruhe
  // ----------------------------------------------------------

  if (nightModeEnabled &&
      isNightTime(localNow.tm_hour)) {

    targetWake =
        getNextHourOccurrence(
          nightSleepEndHour,
          now
        );
  }

  // ----------------------------------------------------------
  // Normaler Tagesbetrieb
  // ----------------------------------------------------------

  else {

    targetWake = getNextRegularWake(now);

    if (nightModeEnabled) {

      // Wann beginnt die nächste Nachtruhe?
      time_t nextNightStart =
          getNextHourOccurrence(
            nightSleepStartHour,
            now
          );

      // Würde der normale Schlaf über den Beginn
      // der Nachtruhe hinausgehen?
      //
      // Dann nicht mitten in der Nacht aufwachen,
      // sondern direkt bis zum Ende der Nachtruhe schlafen.
      if (nextNightStart <= targetWake) {

        targetWake =
            getNextHourOccurrence(
              nightSleepEndHour,
              nextNightStart
            );
      }
    }
  }

  long sleepSeconds =
      (long)difftime(targetWake, now);

  if (sleepSeconds < 5) {
    sleepSeconds = 5;
  }

  struct tm targetLocal;
  localtime_r(&targetWake, &targetLocal);

  Serial.printf(
    "[SLEEP] Naechster Wake: %02d:%02d:%02d\n",
    targetLocal.tm_hour,
    targetLocal.tm_min,
    targetLocal.tm_sec
  );

  Serial.print("[SLEEP] Schlafdauer: ");
  Serial.print(sleepSeconds);
  Serial.println(" s");

  return (uint32_t)sleepSeconds;
}

void enterDeepSleep(uint32_t sleepSeconds)
{
  if (!canEnterDeepSleep()) {
    return;
  }

  if (sleepSeconds < 5) {
    sleepSeconds = 5;
  }

  Serial.println();
  Serial.println("====================================");
  Serial.println("[SLEEP] Deep Sleep wird vorbereitet");
  Serial.println("====================================");

  // ----------------------------------------------------------
  // Wake-Timer zuerst konfigurieren
  // ----------------------------------------------------------

  uint64_t sleepUs =
      (uint64_t)sleepSeconds * 1000000ULL;

  esp_err_t result =
      esp_sleep_enable_timer_wakeup(sleepUs);

  if (result != ESP_OK) {
    Serial.print(
      "[SLEEP] FEHLER Wake-Timer: "
    );
    Serial.println((int)result);
    return;
  }

  // ----------------------------------------------------------
  // Letzte Messwerte senden
  // ----------------------------------------------------------

  if (Blynk.connected()) {

    sendSensorValuesToBlynk();
    Blynk.virtualWrite(
    VPIN_PUMP_STATE,
    0
    );

    Blynk.virtualWrite(
      VPIN_SYSTEM_STATE_TEXT,
      "Deep Sleep"
    );

    // Blynk noch kurz Zeit zum Senden geben
    unsigned long flushStart = millis();

    while (millis() - flushStart < 300UL) {
      Blynk.run();
      delay(10);
    }
  }

  // Während dieser kurzen Zeit könnte theoretisch noch
  // ein Blynk-Befehl eingetroffen sein.
  if (!canEnterDeepSleep()) {
    Serial.println(
      "[SLEEP] Sleep wurde kurzfristig abgebrochen"
    );
    return;
  }

  // ----------------------------------------------------------
  // Pumpe sicher AUS
  // ----------------------------------------------------------

  digitalWrite(PIN_PUMP, LOW);
  pumpOn = false;

// ----------------------------------------------------------
// MAX17048 in Energiesparmodus
// ----------------------------------------------------------

if (batteryMonitorOk) {
  batteryMonitor.hibernate();
  Serial.println("[POWER] MAX17048 Hibernate");
}

ringOff();

pinMode(PIN_LED_RING, OUTPUT);
digitalWrite(PIN_LED_RING, LOW);

delay(5);

  // ----------------------------------------------------------
  // 5-V-Sensorversorgung AUS
  // ----------------------------------------------------------

  sensorPowerOff();

  delay(50);

  Serial.println("[SLEEP] Sensoren AUS");
  Serial.println("[SLEEP] ESP32 geht jetzt schlafen");
  Serial.flush();

  // WLAN wird im Deep Sleep nicht benötigt
  Blynk.disconnect();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  delay(50);

// ----------------------------------------------------------
// Solar muss während Deep Sleep verbunden bleiben
// ----------------------------------------------------------

pinMode(PIN_SOLAR_CTRL, OUTPUT);
digitalWrite(PIN_SOLAR_CTRL, LOW);

delay(10);

// ----------------------------------------------------------
// Deep Sleep
// ----------------------------------------------------------

prepareSensorPowerForDeepSleep();
esp_deep_sleep_start();

  // Hierhin kommt der Programmablauf nicht mehr.
}

void handleSleepManager()
{
  if (!canEnterDeepSleep()) {
    return;
  }

  uint32_t sleepSeconds = calculateSleepSeconds();

  if (sleepSeconds == 0) {
    return;
  }

  enterDeepSleep(sleepSeconds);
}

/************************************************************
  Solar + beim Aufwachen kurs unterbrechen um 6h Notabschaltung zu umgehen
************************************************************/

void handleSolarWakeReset()
{
  esp_sleep_wakeup_cause_t wakeCause =
      esp_sleep_get_wakeup_cause();

  pinMode(PIN_SOLAR_CTRL, OUTPUT);

  if (wakeCause == ESP_SLEEP_WAKEUP_TIMER)
  {
    solarResetThisBoot = true;
    solarResetCounter++;

    // PC817 EIN -> Solar trennen
    digitalWrite(PIN_SOLAR_CTRL, HIGH);

    delay(1000);

    // PC817 AUS -> Solar wieder verbinden
    digitalWrite(PIN_SOLAR_CTRL, LOW);
  }
  else
  {
    solarResetThisBoot = false;

    // Normaler Boot -> Solar verbunden
    digitalWrite(PIN_SOLAR_CTRL, LOW);
  }
}

/************************************************************
  Pumpe
************************************************************/

void pumpStart(String reason) {
  if (pumpOn) {
    return;
  }

  if (!hasWater()) {
    pumpOn = false;
    digitalWrite(PIN_PUMP, LOW);
    Blynk.virtualWrite(VPIN_PUMP_STATE, 0);
    setError(WATER_WARNING_TEXT);
    return;
  }

  clearError();

  pumpOn = true;
  pumpStartMillis = millis();

  digitalWrite(PIN_PUMP, HIGH);
  Blynk.virtualWrite(VPIN_PUMP_STATE, 1);

  setSystemState("Pumpe läuft: " + reason);
}

void pumpStop(String reason) {
  if (!pumpOn) {
    return;
  }

  pumpOn = false;
  digitalWrite(PIN_PUMP, LOW);
  Blynk.virtualWrite(VPIN_PUMP_STATE, 0);
  setSystemState("Pumpe gestoppt: " + reason);
}

void pumpSafetyCheck() {
  if (!pumpOn) {
    return;
  }

  unsigned long runtimeMs = millis() - pumpStartMillis;
  unsigned long maxRuntimeMs = (unsigned long)(maxRuntimeMin * 60.0 * 1000.0);

if (runtimeMs >= maxRuntimeMs) {
  manualPumpRequest = false;

  if (Blynk.connected()) {
    Blynk.virtualWrite(VPIN_MANUAL_PUMP, 0);
  }

  pumpStop("Maximallaufzeit überschritten");
  setError("Sicherheitsabschaltung: Maximallaufzeit überschritten");

  updateWidgetAvailability();

  Serial.println("Sicherheitsabschaltung: manueller Pumpenwunsch wurde zurueckgesetzt.");
  return;
}

  if (!hasWater()) {
    pumpStop("Wasserstand zu niedrig");
    setError(WATER_WARNING_TEXT);
  }
}

void updateManualPumpSwitchAvailability() {
  if (!Blynk.connected()) {
    return;
  }

  bool manualMode = currentMode == MODE_MANUAL;
  bool waterOk = hasWater();

  bool switchAllowed = manualMode && waterOk;

  Blynk.setProperty(VPIN_MANUAL_PUMP, "isDisabled", !switchAllowed);

  if (!waterOk) {
    // App-Schalter optisch und logisch auf AUS zurücksetzen
    Blynk.virtualWrite(VPIN_MANUAL_PUMP, 0);
  }

  Serial.print("Manueller Pumpenschalter: ");
  Serial.println(switchAllowed ? "freigegeben" : "gesperrt");
}

float mapRawToLiters(int distanceMm) {
  if (waterEmptyRaw == waterFullRaw) {
    return 0.0;
  }

  // Abstandssensor:
  // kleiner Abstand = voll
  // großer Abstand  = leer
  //
  // waterEmptyRaw = Abstand bei MIN/leer
  // waterFullRaw  = Abstand bei MAX/voll
  float liters =
    ((float)(waterEmptyRaw - distanceMm) * waterFullLiters) /
    ((float)(waterEmptyRaw - waterFullRaw));

  return clampFloat(liters, 0.0, waterFullLiters);
}

bool isValidWaterCalibration() {
  return waterEmptyRaw > waterFullRaw;
}

/************************************************************
  Sensoren
************************************************************/

bool waterDistanceFresh = false;
unsigned long lastWaterFrameMillis = 0;
unsigned long waterGoodFrames = 0;
unsigned long waterBadChecksumFrames = 0;
unsigned long waterDroppedBytes = 0;

uint8_t lastWaterFrame[4] = {0, 0, 0, 0};

int readWaterDistanceMm() {
  waterDistanceFresh = false;
  int newestDistanceMm = -1;

  while (WaterSerial.available() > 0) {

    // Auf Frame-Start 0xFF synchronisieren
    if (WaterSerial.peek() != 0xFF) {
      WaterSerial.read();
      waterDroppedBytes++;
      continue;
    }

    // Noch kein kompletter Frame da
    if (WaterSerial.available() < 4) {
      break;
    }

    uint8_t data[4];
    size_t n = WaterSerial.readBytes(data, 4);

    if (n != 4) {
      break;
    }

    uint8_t checksum = (data[0] + data[1] + data[2]) & 0xFF;

    if (checksum != data[3]) {
      waterBadChecksumFrames++;
      continue;
    }

    int distanceMm = ((int)data[1] << 8) | data[2];

    // Gültiger Sensorbereich: ca. 30 bis 4500 mm
    if (distanceMm >= 30 && distanceMm <= 4500) {
      newestDistanceMm = distanceMm;

      lastWaterFrame[0] = data[0];
      lastWaterFrame[1] = data[1];
      lastWaterFrame[2] = data[2];
      lastWaterFrame[3] = data[3];

      waterGoodFrames++;
      waterDistanceFresh = true;
      lastWaterFrameMillis = millis();
    }
  }

  return newestDistanceMm;
}

void readSensors() {
  rawMoisture1 = analogRead(PIN_MOISTURE_1);
  rawMoisture2 = analogRead(PIN_MOISTURE_2);
  rawMoisture3 = analogRead(PIN_MOISTURE_3);

int distanceMm = readWaterDistanceMm();

if (distanceMm > 0) {
  rawWaterLevel = distanceMm;
  waterLiters = mapRawToLiters(rawWaterLevel);
  waterLevelValid = true;
}

// Sensorwert zu alt oder noch nie empfangen?
if (lastWaterFrameMillis == 0 ||
    millis() - lastWaterFrameMillis > WATER_SENSOR_STALE_MS)
{
  waterLevelValid = false;
}

  moisture1Pct = mapRawToPercent(rawMoisture1, m1DryRaw, m1WetRaw);
  moisture2Pct = mapRawToPercent(rawMoisture2, m2DryRaw, m2WetRaw);
  moisture3Pct = mapRawToPercent(rawMoisture3, m3DryRaw, m3WetRaw);

  moistureAvgPct = (moisture1Pct + moisture2Pct + moisture3Pct) / 3.0;

  tempSensor.requestTemperatures();
  float t = tempSensor.getTempCByIndex(0);

  if (t > -100.0 && t < 100.0) {
    temperatureC = t;
  } else {
    temperatureC = NAN;
  }
}

void sendSensorValuesToBlynk() {
  Blynk.virtualWrite(VPIN_MOISTURE_1, moisture1Pct);
  Blynk.virtualWrite(VPIN_MOISTURE_2, moisture2Pct);
  Blynk.virtualWrite(VPIN_MOISTURE_3, moisture3Pct);
  Blynk.virtualWrite(VPIN_MOISTURE_AVG, moistureAvgPct);

  Blynk.virtualWrite(VPIN_RAW_MOISTURE_1, rawMoisture1);
  Blynk.virtualWrite(VPIN_RAW_MOISTURE_2, rawMoisture2);
  Blynk.virtualWrite(VPIN_RAW_MOISTURE_3, rawMoisture3);

  Blynk.virtualWrite(VPIN_WATER_RAW, rawWaterLevel);
  Blynk.virtualWrite(VPIN_WATER_LITERS, waterLiters);
  Blynk.virtualWrite(VPIN_WATER_EMPTY_RAW, waterEmptyRaw);  
  Blynk.virtualWrite(VPIN_WATER_FULL_RAW, waterFullRaw);    
  Blynk.virtualWrite(VPIN_WATER_FULL_L, waterFullLiters);   

  


  if (!isnan(temperatureC)) {
    Blynk.virtualWrite(VPIN_TEMPERATURE, temperatureC);
  }

  Blynk.virtualWrite(VPIN_UNIX_TIME, time(nullptr));

  Blynk.virtualWrite(VPIN_PUMP_STATE, pumpOn ? 1 : 0);

  if (batteryMonitorOk) {
  if (!isnan(batteryVoltage)) {
    Blynk.virtualWrite(VPIN_BATTERY_VOLTAGE, batteryVoltage);
  }

  if (!isnan(batteryPercent)) {
    Blynk.virtualWrite(VPIN_BATTERY_PERCENT, batteryPercent);
  }

  Blynk.virtualWrite(VPIN_BATTERY_MONITOR_OK, 1);
} else {
  Blynk.virtualWrite(VPIN_BATTERY_MONITOR_OK, 0);
}
}

BLYNK_WRITE(VPIN_MESSAGING_ACTIVE) {
  messagingActive = param.asInt() == 1;

  Blynk.virtualWrite(VPIN_MESSAGING_ACTIVE, messagingActive ? 1 : 0);

  Serial.print("APP -> ESP | V37 | Telegram Messaging = ");
  Serial.println(messagingActive ? "AKTIV" : "AUS");

  if (!messagingActive) {
    waterLowTelegramSent = false;
  }
}

BLYNK_WRITE(VPIN_MOISTURE_CONTROL_MODE)
{
  int mode = param.asInt();

  if (mode < 0) {
    mode = 0;
  }

  if (mode > 3) {
    mode = 3;
  }

  moistureControlMode = (MoistureControlMode)mode;

  // Auswahl zur App zurückschreiben
  Blynk.virtualWrite(
    VPIN_MOISTURE_CONTROL_MODE,
    (int)moistureControlMode
  );

  Serial.print("APP -> ESP | V99 | Feuchteregelung = ");
  Serial.println(moistureControlModeToText());

  Serial.print("Aktueller Regel-Feuchtewert = ");
  Serial.print(getControlMoisturePct(), 1);
  Serial.println(" %");
}

BLYNK_WRITE(VPIN_SET_WATER_EMPTY_RAW) {
  if (param.asInt() == 1) {
    readSensors();

    if (rawWaterLevel <= 0) {
      Serial.println("APP -> ESP | V78 | Wasser MIN/LEER speichern verweigert: kein gueltiger Abstandswert");

      Blynk.virtualWrite(VPIN_SET_WATER_EMPTY_RAW, 0);
      setSystemState("Wasser MIN speichern fehlgeschlagen");
      return;
    }

    waterEmptyRaw = rawWaterLevel;

    Blynk.virtualWrite(VPIN_WATER_EMPTY_RAW, waterEmptyRaw);
    Blynk.virtualWrite(VPIN_SET_WATER_EMPTY_RAW, 0);

    if (isValidWaterCalibration()) {
      waterLiters = mapRawToLiters(rawWaterLevel);
    } else {
      waterLiters = 0.0;
      setSystemState("Wasser-Kalibrierung ungueltig");
    }

    Blynk.virtualWrite(VPIN_WATER_LITERS, waterLiters);

    Serial.print("APP -> ESP | V78 | Wasser MIN/LEER Abstand gespeichert = ");
    Serial.print(waterEmptyRaw);
    Serial.println(" mm");

    Serial.print("Aktuelle Wassermenge = ");
    Serial.print(waterLiters, 1);
    Serial.println(" l");

    setSystemState("Wasser MIN/LEER gespeichert: " + String(waterEmptyRaw) + " mm");
  }
}

BLYNK_WRITE(VPIN_MESSAGING_WATER_WARNING_PCT) {
  messagingWaterWarningPct = param.asInt();

  if (messagingWaterWarningPct < 1) {
    messagingWaterWarningPct = 1;
  }

  if (messagingWaterWarningPct > 100) {
    messagingWaterWarningPct = 100;
  }

  Blynk.virtualWrite(VPIN_MESSAGING_WATER_WARNING_PCT, messagingWaterWarningPct);

  Serial.print("APP -> ESP | V38 | Telegram-Warnschwelle Wasserstand = ");
  Serial.print(messagingWaterWarningPct);
  Serial.println(" %");
}

BLYNK_WRITE(VPIN_SET_WATER_FULL_RAW) {
  if (param.asInt() == 1) {
    readSensors();

    if (rawWaterLevel <= 0) {
      Serial.println("APP -> ESP | V79 | Wasser MAX/VOLL speichern verweigert: kein gueltiger Abstandswert");

      Blynk.virtualWrite(VPIN_SET_WATER_FULL_RAW, 0);
      setSystemState("Wasser MAX speichern fehlgeschlagen");
      return;
    }

    waterFullRaw = rawWaterLevel;

    Blynk.virtualWrite(VPIN_WATER_FULL_RAW, waterFullRaw);
    Blynk.virtualWrite(VPIN_SET_WATER_FULL_RAW, 0);

    if (isValidWaterCalibration()) {
      waterLiters = mapRawToLiters(rawWaterLevel);
    } else {
      waterLiters = 0.0;
      setSystemState("Wasser-Kalibrierung ungueltig");
    }

    Blynk.virtualWrite(VPIN_WATER_LITERS, waterLiters);

    Serial.print("APP -> ESP | V79 | Wasser MAX/VOLL Abstand gespeichert = ");
    Serial.print(waterFullRaw);
    Serial.println(" mm");

    Serial.print("Aktuelle Wassermenge = ");
    Serial.print(waterLiters, 1);
    Serial.println(" l");

    setSystemState("Wasser MAX/VOLL gespeichert: " + String(waterFullRaw) + " mm");
  }
}

BLYNK_WRITE(VPIN_WATER_FULL_L) {
  waterFullLiters = param.asFloat();

  if (waterFullLiters < 1.0) {
    waterFullLiters = 1.0;
  }

  if (waterFullLiters > 10000.0) {
    waterFullLiters = 10000.0;
  }

  Blynk.virtualWrite(VPIN_WATER_FULL_L, waterFullLiters);
  Blynk.virtualWrite(VPIN_WATER_LITERS, waterLiters);

  Serial.print("BLYNK -> ESP | V75 | Nutzbares Wasservolumen gesetzt = ");
  Serial.print(waterFullLiters, 1);
  Serial.println(" l");
}

BLYNK_WRITE(VPIN_WATER_EMPTY_RAW) {
  waterEmptyRaw = constrain(param.asInt(), 0, 4500);

  if (isValidWaterCalibration()) {
    waterLiters = mapRawToLiters(rawWaterLevel);
  } else {
    waterLiters = 0.0;
    setSystemState("Wasser-Kalibrierung ungueltig");
  }

  Blynk.virtualWrite(VPIN_WATER_EMPTY_RAW, waterEmptyRaw);
 
  Serial.print("BLYNK -> ESP | V76 | Wasser MIN/LEER Abstand gesetzt = ");
  Serial.print(waterEmptyRaw);
  Serial.println(" mm");
}

BLYNK_WRITE(VPIN_WATER_FULL_RAW) {
  waterFullRaw = constrain(param.asInt(), 0, 4500);

  if (isValidWaterCalibration()) {
    waterLiters = mapRawToLiters(rawWaterLevel);
  } else {
    waterLiters = 0.0;
    setSystemState("Wasser-Kalibrierung ungueltig");
  }

  Blynk.virtualWrite(VPIN_WATER_FULL_RAW, waterFullRaw);
  
  Serial.print("BLYNK -> ESP | V77 | Wasser MAX/VOLL Abstand gesetzt = ");
  Serial.print(waterFullRaw);
  Serial.println(" mm");
}

BLYNK_WRITE(VPIN_DEV_MODE)
{
  devMode = param.asInt() == 1;
  devModeSynced = true;

  Serial.printf(
    "[SLEEP] DEV_MODE = %d -> Deep Sleep %s\n",
    devMode,
    devMode ? "GESPERRT" : "FREIGEGEBEN"
  );
}

BLYNK_WRITE(VPIN_WAKE_INTERVAL_MIN)
{
  wakeIntervalMin = param.asInt();

  if (wakeIntervalMin < 1) {
    wakeIntervalMin = 1;
  }

  if (wakeIntervalMin > 1440) {
    wakeIntervalMin = 1440;
  }

  wakeIntervalSynced = true;

  Serial.print("[SLEEP] Wake-Intervall = ");
  Serial.print(wakeIntervalMin);
  Serial.println(" min");
}


BLYNK_WRITE(VPIN_NIGHT_SLEEP_START)
{
  nightSleepStartHour = constrain(param.asInt(), 0, 23);

  nightStartSynced = true;

  Serial.print("[SLEEP] Nachtruhe Beginn = ");
  Serial.print(nightSleepStartHour);
  Serial.println(":00");
}

BLYNK_WRITE(VPIN_OPERATION_MODE)
{
  int value = param.asInt();

  // Nur 0 = Indoor und 1 = Outdoor zulassen
  if (value != 0 && value != 1) {
    value = 1;   // Sicherheitsfallback = Outdoor
  }

  operationMode = (OperationMode)value;
  operationModeSynced = true;

  Serial.print("APP -> ESP | V35 | Betriebsart = ");

  if (operationMode == OPERATION_INDOOR)
  {
    Serial.println("INDOOR / NETZ");
  }
  else
  {
    Serial.println("OUTDOOR / SOLAR-BATTERIE");
  }

  // Blynk-Widgets passend zur Betriebsart
  updateWidgetAvailability();

  // Aktuellen Zustand zur App zurückschreiben
  Blynk.virtualWrite(
    VPIN_OPERATION_MODE,
    (int)operationMode
  );

  // LED-Ring passend zu Indoor/Outdoor und Systemstatus
  updateRingStatus();
}

BLYNK_WRITE(VPIN_NIGHT_SLEEP_END)
{
  nightSleepEndHour = constrain(param.asInt(), 0, 23);

  nightEndSynced = true;

  Serial.print("[SLEEP] Nachtruhe Ende = ");
  Serial.print(nightSleepEndHour);
  Serial.println(":00");
}

/************************************************************
  Steuerlogik: Manuell
************************************************************/

void handleManualMode() {
  autoPhase = AUTO_IDLE;

  if (manualPumpRequest) {
    if (!pumpOn) {
      pumpStart("Manuell");
    }
  } else {
    if (pumpOn) {
      pumpStop("Manuell ausgeschaltet");
    }
  }
}

/************************************************************
  Steuerlogik: Fix-Automatik
************************************************************/
bool getFixedLocalTime(struct tm &timeinfo) {
  time_t now = time(nullptr);

  // Zeit noch nicht gültig
  if (now < 1700000000) {
    return false;
  }

  localtime_r(&now, &timeinfo);
  return true;
}

int getFixedDayKey(const struct tm &timeinfo) {
  // fortlaufender Tages-Schlüssel, ausreichend für Tagesvergleiche
  return timeinfo.tm_year * 366 + timeinfo.tm_yday;
}

void handleFixedMode() {
  if (currentMode != MODE_FIXED) {
    if (fixedCycleActive) {
      fixedCycleActive = false;
    }
    return;
  }

  struct tm timeinfo;

  if (!getFixedLocalTime(timeinfo)) {
    
    setSystemState("Fixed: warte auf Uhrzeit");
    return;
  }

  // Laufende Fixed-Giessung beenden, wenn die Dauer erreicht ist
  if (fixedCycleActive) {
    unsigned long runtimeMs = millis() - fixedCycleStartMillis;

    float effectiveDurationMin = fixedDurationMin;

    if (effectiveDurationMin < 0.1) {
      effectiveDurationMin = 0.1;
    }

    if (effectiveDurationMin > maxRuntimeMin) {
      effectiveDurationMin = maxRuntimeMin;
    }

    unsigned long durationMs = (unsigned long)(effectiveDurationMin * 60.0 * 1000.0);

    if (runtimeMs >= durationMs) {
      pumpStop("Fixed: Giessdauer beendet");
      fixedCycleActive = false;

      Blynk.virtualWrite(VPIN_PUMP_STATE, 0);

      setSystemState("Fixed: Giessung beendet");
    }

    return;
  }


  // Wenn Pumpe aus einem anderen Grund läuft, hier nichts starten
  if (pumpOn) {
    return;
  }

  int currentHour = timeinfo.tm_hour;
  int currentMinute = timeinfo.tm_min;
  int dayKey = getFixedDayKey(timeinfo);

  // Nur am Anfang der Stunde starten, damit nicht irgendwann mitten in der Stunde gestartet wird
  if (currentMinute >= FIXED_START_WINDOW_MIN) {
    return;
  }

  bool shouldStart = false;

  // V13 = 0 -> 2x täglich: morgens und abends
  if (fixedFrequencyMode == 0) {
    if (currentHour == startfirstHour && lastFixedfirstDayKey != dayKey) {
      shouldStart = true;
      lastFixedfirstDayKey = dayKey;
    }

    if (currentHour == startsecondHour && lastFixedsecondDayKey != dayKey) {
      shouldStart = true;
      lastFixedsecondDayKey = dayKey;
    }
  }

  // V13 = 1 -> täglich: egal wann
  if (fixedFrequencyMode == 1) {
    if (currentHour == startfirstHour && lastFixedfirstDayKey != dayKey) {
      shouldStart = true;
      lastFixedfirstDayKey = dayKey;
    }
  }

  // V13 = 2 -> jeden 2. Tag: egal wann
  if (fixedFrequencyMode == 2) {
    if (currentHour == startfirstHour) {
      if (lastFixedEvery2DayKey < 0 || dayKey - lastFixedEvery2DayKey >= 2) {
        shouldStart = true;
        lastFixedEvery2DayKey = dayKey;
      }
    }
  }

  // V13 = 3 -> wöchentlich: egal wann
  if (fixedFrequencyMode == 3) {
    if (currentHour == startfirstHour) {
      if (lastFixedWeeklyDayKey < 0 || dayKey - lastFixedWeeklyDayKey >= 7) {
        shouldStart = true;
        lastFixedWeeklyDayKey = dayKey;
      }
    }
  }

  if (!shouldStart) {
    return;
  }

  // Wassersperre
  if (!hasWater()) {
    setError(WATER_WARNING_TEXT);
    return;
  }

  pumpStart("Fixed: Giessung gestartet");
  fixedCycleActive = true;
  fixedCycleStartMillis = millis();

  Blynk.virtualWrite(VPIN_PUMP_STATE, 1);

  setSystemState("Fixed: Giessung aktiv");
}

/************************************************************
  Steuerlogik: Vollautomatik
************************************************************/

void handleAutoMode()
{
  manualPumpRequest = false;

  if (Blynk.connected()) {
    Blynk.virtualWrite(VPIN_MANUAL_PUMP, 0);
  }

  unsigned long nowMs = millis();

  // ----------------------------------------------------------
  // Feuchtewert auswählen, nach dem geregelt wird
  // ----------------------------------------------------------

  float controlMoisturePct = getControlMoisturePct();

  float effectiveWateringDurationMin = fixedDurationMin;

  if (effectiveWateringDurationMin < 0.1) {
    effectiveWateringDurationMin = 0.1;
  }

  if (effectiveWateringDurationMin > maxRuntimeMin) {
    effectiveWateringDurationMin = maxRuntimeMin;
  }

  unsigned long pulseMs =
      (unsigned long)(
        effectiveWateringDurationMin *
        60.0 *
        1000.0
      );

  unsigned long pauseMs =
      (unsigned long)(
        pumpCoolingDurationMin *
        60.0 *
        1000.0
      );

  switch (autoPhase)
  {
    // --------------------------------------------------------
    // Warten auf Unterschreitung der MIN-Feuchte
    // --------------------------------------------------------

    case AUTO_IDLE:

      if (controlMoisturePct <= autoStartMoisturePct)
      {
      pumpStart("Vollautomatik Impuls");

      // Nur in den Bewässerungszustand wechseln,
      // wenn die Pumpe tatsächlich gestartet wurde.
      if (pumpOn)
      {
      autoPhase = AUTO_WATERING;
      autoPhaseStartMillis = nowMs;
      }
      else
      {
      // Pumpenstart wurde z. B. wegen Wassermangel blockiert.
      autoPhase = AUTO_IDLE;
      }
    }
    else
    {
    setSystemState("Vollautomatik bereit");
    }

    break;


    // --------------------------------------------------------
    // Bewässerung läuft
    // --------------------------------------------------------

    case AUTO_WATERING:

      if (controlMoisturePct >= autoStopMoisturePct)
      {
        pumpStop("Ziel-Feuchte erreicht");

        autoPhase = AUTO_IDLE;
      }

      else if (
        nowMs - autoPhaseStartMillis >= pulseMs
      )
      {
        pumpStop("Impuls beendet, Wartepause");

        autoPhase = AUTO_PAUSING;
        autoPhaseStartMillis = nowMs;
      }

      break;


    // --------------------------------------------------------
    // Wartepause nach Bewässerungsimpuls
    // --------------------------------------------------------

    case AUTO_PAUSING:
    {
      unsigned long elapsedMs =
          nowMs - autoPhaseStartMillis;

      if (controlMoisturePct >= autoStopMoisturePct)
      {
        autoPhase = AUTO_IDLE;

        setSystemState(
          "Vollautomatik: Ziel-Feuchte erreicht"
        );
      }

    else if (elapsedMs >= pauseMs)
    {
      pumpStart(
      "Vollautomatik weiterer Impuls"
      );

      // Nur weiterbewässern, wenn die Pumpe
     // tatsächlich gestartet wurde.
      if (pumpOn)
    {
      autoPhase = AUTO_WATERING;
      autoPhaseStartMillis = nowMs;
    }
    else
    {
      // Start wurde z. B. wegen Wassermangel blockiert.
      autoPhase = AUTO_IDLE;
    }
  }

      else
      {
        unsigned long remainingMs =
            pauseMs - elapsedMs;

        unsigned long remainingSec =
            (remainingMs + 999UL) / 1000UL;

        setSystemState(
          "Vollautomatik: Wartepause noch " +
          String(remainingSec) +
          " s"
        );
      }

      break;
    }
  }
}

/************************************************************
  Hauptlogik
************************************************************/
void sendSensorValuesToBlynkIfDue() {
  if (!Blynk.connected()) {
    return;
  }

  unsigned long nowMs = millis();

  if (nowMs - lastBlynkSendMillis < blynkSendIntervalMs) {
    return;
  }

  lastBlynkSendMillis = nowMs;

  sendSensorValuesToBlynk();

/*
  Serial.print("Blynk-Daten gesendet. Intervall = ");
  Serial.print(blynkSendIntervalMs);
  Serial.println(" ms");
  */
}

void controlLoop()
{
  readSensors();

  pumpSafetyCheck();

  // ----------------------------------------------------------
  // Sicherheitsphase nach Wake / Boot
  // ----------------------------------------------------------

// ----------------------------------------------------------
// Betriebsart muss nach Boot bekannt sein
// ----------------------------------------------------------

if (!operationModeSynced)
{
  manualPumpRequest = false;

  if (pumpOn) {
    pumpStop("Warte auf Betriebsart");
  }

  setSystemState("Warte auf Betriebsart V35");
  return;
}


// ----------------------------------------------------------
// Sleep-/Nachtruhesteuerung ausschließlich OUTDOOR
// ----------------------------------------------------------

if (operationMode == OPERATION_OUTDOOR)
{
  if (!sleepConfigReady() ||
      !systemTimeValid())
  {
    manualPumpRequest = false;

    if (pumpOn) {
      pumpStop("System noch nicht bereit");
    }

    setSystemState(
      "Warte auf Konfiguration / Uhrzeit"
    );

    return;
  }

  if (isNightNow())
  {
    manualPumpRequest = false;

    if (pumpOn) {
      pumpStop("Nachtruhe");
    }

    setSystemState("Nachtruhe");

    updateWaterWarningStatus();
    sendSensorValuesToBlynkIfDue();

    firstControlCycleDone = true;

    handleSleepManager();
    return;
  }
}

  // ----------------------------------------------------------
  // normale Betriebslogik
  // ----------------------------------------------------------

  switch (currentMode) {
    case MODE_MANUAL:
      handleManualMode();
      break;

    case MODE_FIXED:
      handleFixedMode();
      break;

    case MODE_AUTO:
      handleAutoMode();
      break;

    default:
      currentMode = MODE_MANUAL;
      Blynk.virtualWrite(VPIN_MODE, 0);
      setError("Ungültiger Modus: zurück auf Manuell");
      break;
  }

  updateWaterWarningStatus();

  sendSensorValuesToBlynkIfDue();

  firstControlCycleDone = true;

  handleSleepManager();
}

/************************************************************
  Blynk Handler
************************************************************/

BLYNK_WRITE(VPIN_PUMP_COOLING_DURATION) {
  pumpCoolingDurationMin = param.asFloat();

  if (pumpCoolingDurationMin < 0.1) {
    pumpCoolingDurationMin = 0.1;
  }

  if (pumpCoolingDurationMin > 180.0) {
    pumpCoolingDurationMin = 180.0;
  }

  Blynk.virtualWrite(VPIN_PUMP_COOLING_DURATION, pumpCoolingDurationMin);

  Serial.print("APP -> ESP | V20 | Abkuehldauer nach Giessimpuls = ");
  Serial.print(pumpCoolingDurationMin, 2);
  Serial.println(" min");
}

BLYNK_CONNECTED() {

  blynkConnectedMillis = millis();

  Serial.println("[SLEEP] Blynk verbunden - Online-Fenster gestartet");

  Blynk.syncVirtual(
    VPIN_MODE,
    VPIN_FIXED_DURATION_MIN,
    VPIN_FIXED_FREQUENCY,
    VPIN_START_FIRST_HOUR,
    VPIN_START_SECOND_HOUR,
    VPIN_AUTO_MIN_MOISTURE,
    VPIN_AUTO_MAX_MOISTURE,
    VPIN_BLYNK_SEND_INTERVAL_MS,
    VPIN_MAX_RUNTIME_MIN,
    VPIN_PUMP_COOLING_DURATION,
    VPIN_WATER_FULL_L,
    VPIN_WATER_EMPTY_RAW,
    VPIN_WATER_FULL_RAW,
    VPIN_MESSAGING_ACTIVE,
    VPIN_MESSAGING_WATER_WARNING_PCT,
    VPIN_M1_DRY,
    VPIN_M1_WET,
    VPIN_M2_DRY,
    VPIN_M2_WET,
    VPIN_M3_DRY,
    VPIN_M3_WET,
    VPIN_DEV_MODE,
    VPIN_OPERATION_MODE,
    VPIN_WAKE_INTERVAL_MIN,
    VPIN_NIGHT_SLEEP_START,
    VPIN_NIGHT_SLEEP_END,
    VPIN_MOISTURE_CONTROL_MODE
  );

  publishModeToBlynk();
  publishWaterCalibrationValues();
  publishMoistureCalibrationValues();

  manualPumpRequest = false;
  Blynk.virtualWrite(VPIN_MANUAL_PUMP, 0);
  
  updateWidgetAvailability();

  Blynk.virtualWrite(VPIN_SYSTEM_STATE_TEXT, systemStateText);
}

/************************************************************
  Betriebsparameter über Blynk
************************************************************/

BLYNK_WRITE(VPIN_FIXED_FREQUENCY) {
  fixedFrequencyMode = param.asInt();

  if (fixedFrequencyMode < 0) {
    fixedFrequencyMode = 0;
  }

  if (fixedFrequencyMode > 3) {
    fixedFrequencyMode = 3;
  }

  Blynk.virtualWrite(VPIN_FIXED_FREQUENCY, fixedFrequencyMode);
  updateWidgetAvailability();
  Serial.print("APP -> ESP | V13 | Fixed Frequenzmodus = ");
  Serial.println(fixedFrequencyMode);
}

BLYNK_WRITE(VPIN_START_FIRST_HOUR) {
  startfirstHour = param.asInt();

  if (startfirstHour < 0) startfirstHour = 0;
  if (startfirstHour > 23) startfirstHour = 23;
}

BLYNK_WRITE(VPIN_START_SECOND_HOUR) {
  startsecondHour = param.asInt();

  if (startsecondHour < 0) startsecondHour = 0;
  if (startsecondHour > 23) startsecondHour = 23;
}

BLYNK_WRITE(VPIN_AUTO_MIN_MOISTURE) {
  autoStartMoisturePct = param.asFloat();
  autoStartMoisturePct = clampFloat(autoStartMoisturePct, 0.0, 100.0);
}

BLYNK_WRITE(VPIN_AUTO_MAX_MOISTURE) {
  autoStopMoisturePct = param.asFloat();
  autoStopMoisturePct = clampFloat(autoStopMoisturePct, 0.0, 100.0);
}

BLYNK_WRITE(VPIN_BLYNK_SEND_INTERVAL_MS) {
  blynkSendIntervalMs = param.asInt();

  if (blynkSendIntervalMs < 1000UL) {
    blynkSendIntervalMs = 1000UL;
  }

  if (blynkSendIntervalMs > 3600000UL) {
    blynkSendIntervalMs = 3600000UL;
  }

  Blynk.virtualWrite(VPIN_BLYNK_SEND_INTERVAL_MS, blynkSendIntervalMs);

  // Nach Änderung sofort einmal senden
  lastBlynkSendMillis = 0;

  Serial.print("APP -> ESP | V19 | Blynk-Sendeintervall = ");
  Serial.print(blynkSendIntervalMs);
  Serial.println(" ms");
}

BLYNK_WRITE(VPIN_MAX_RUNTIME_MIN) {
  maxRuntimeMin = param.asFloat();

  if (maxRuntimeMin < 0.1) {
    maxRuntimeMin = 0.1;
  }

  if (maxRuntimeMin > 20.0) {
    maxRuntimeMin = 20.0;
  }

  Blynk.virtualWrite(VPIN_MAX_RUNTIME_MIN, maxRuntimeMin);

  Serial.print("APP -> ESP | V26 | Max. Pumpenlaufzeit = ");
  Serial.print(maxRuntimeMin, 2);
  Serial.println(" min");
}

BLYNK_WRITE(VPIN_MODE) {
  int value = param.asInt();

  Serial.print("APP -> ESP | V10 | Betriebsmodus = ");
  Serial.println(value);

  // Bisherigen Modus merken
  UserMode oldMode = currentMode;

  // ----------------------------------------------------------
  // Neuen Betriebsmodus übernehmen
  // ----------------------------------------------------------

  if (value == 0) {
    currentMode = MODE_MANUAL;
  }
  else if (value == 1) {
    currentMode = MODE_FIXED;
  }
  else if (value == 2) {
    currentMode = MODE_AUTO;
  }
  else {
    currentMode = MODE_MANUAL;

    Blynk.virtualWrite(VPIN_MODE, 0);

    setError(
      "Ungueltiger Modus: zurueck auf Manuell"
    );

    updateWidgetAvailability();
    return;
  }

  // ----------------------------------------------------------
  // Bei echtem Moduswechsel:
  // alten Bewaesserungszustand vollständig beenden
  // ----------------------------------------------------------

  if (currentMode != oldMode)
  {
    // Manuellen Pumpenwunsch löschen
    manualPumpRequest = false;

    if (Blynk.connected()) {
      Blynk.virtualWrite(
        VPIN_MANUAL_PUMP,
        0
      );
    }

    // Laufende Pumpe sicher abschalten
    if (pumpOn) {
      pumpStop(
        "Betriebsmodus gewechselt"
      );
    }

    // Fixed-Zustand zurücksetzen
    fixedCycleActive = false;

    // Vollautomatik zurücksetzen
    autoPhase = AUTO_IDLE;

    Serial.print(
      "Moduswechsel: "
    );
    Serial.print(
      modeToText(oldMode)
    );
    Serial.print(
      " -> "
    );
    Serial.println(
      modeToText(currentMode)
    );
  }

  // ----------------------------------------------------------
  // Status aktualisieren
  // ----------------------------------------------------------

  clearError();

  setSystemState(
    "Betriebsmodus: " +
    modeToText(currentMode)
  );

  publishModeToBlynk();

  updateWaterWarningStatus();

  updateWidgetAvailability();
}

BLYNK_WRITE(VPIN_MANUAL_PUMP) {
  int value = param.asInt();

  Serial.print("APP -> ESP | V11 | Manuelle Pumpe = ");
  Serial.println(value);

  if (currentMode != MODE_MANUAL) {
    manualPumpRequest = false;
    Blynk.virtualWrite(VPIN_MANUAL_PUMP, 0);
    updateWidgetAvailability();

    Serial.println("Manueller Pumpenbefehl ignoriert: Modus ist nicht Manuell");
    return;
  }

  if (value == 1 && !hasWater()) {
    manualPumpRequest = false;
    Blynk.virtualWrite(VPIN_MANUAL_PUMP, 0);
    setError(WATER_WARNING_TEXT);
    updateWidgetAvailability();

    Serial.println("Manueller Pumpenstart blockiert: Wasserstand unter Mindestgrenze");
    return;
  }

  manualPumpRequest = value == 1;

  if (manualPumpRequest) {
    pumpStart("Manueller Start");
  } else {
    pumpStop("Manueller Stopp");
  }

  updateWidgetAvailability();
}

BLYNK_WRITE(VPIN_FIXED_DURATION_MIN) {
  fixedDurationMin = param.asFloat();

  if (fixedDurationMin < 0.1) {
    fixedDurationMin = 0.1;
  }

  if (fixedDurationMin > maxRuntimeMin) {
    fixedDurationMin = maxRuntimeMin;
  }

  if (fixedDurationMin > 30.0) {
    fixedDurationMin = 30.0;
  }

  Blynk.virtualWrite(VPIN_FIXED_DURATION_MIN, fixedDurationMin);

  Serial.print("APP -> ESP | V12 | Giessdauer je Pumpenlauf = ");
  Serial.print(fixedDurationMin, 2);
  Serial.println(" min");
}


/************************************************************
  Kalibrierwerte über Blynk
************************************************************/
void publishMoistureCalibrationValues() {
  Blynk.virtualWrite(VPIN_M1_DRY, m1DryRaw);
  Blynk.virtualWrite(VPIN_M1_WET, m1WetRaw);

  Blynk.virtualWrite(VPIN_M2_DRY, m2DryRaw);
  Blynk.virtualWrite(VPIN_M2_WET, m2WetRaw);

  Blynk.virtualWrite(VPIN_M3_DRY, m3DryRaw);
  Blynk.virtualWrite(VPIN_M3_WET, m3WetRaw);
}

BLYNK_WRITE(VPIN_SET_M1_DRY) {
  if (param.asInt() == 1) {
    readSensors();

    m1DryRaw = rawMoisture1;

    Blynk.virtualWrite(VPIN_M1_DRY, m1DryRaw);
    Blynk.virtualWrite(VPIN_SET_M1_DRY, 0);

    Serial.print("APP -> ESP | V86 | S1 TROCKEN gespeichert = ");
    Serial.println(m1DryRaw);

    setSystemState("S1 Trockenwert gespeichert: " + String(m1DryRaw));
    printMoistureCalibrationToSerial();
  }
}

BLYNK_WRITE(VPIN_SET_M1_WET) {
  if (param.asInt() == 1) {
    readSensors();

    m1WetRaw = rawMoisture1;

    Blynk.virtualWrite(VPIN_M1_WET, m1WetRaw);
    Blynk.virtualWrite(VPIN_SET_M1_WET, 0);

    Serial.print("APP -> ESP | V87 | S1 NASS gespeichert = ");
    Serial.println(m1WetRaw);

    setSystemState("S1 Nasswert gespeichert: " + String(m1WetRaw));
    printMoistureCalibrationToSerial();
  }
}

BLYNK_WRITE(VPIN_SET_M2_DRY) {
  if (param.asInt() == 1) {
    readSensors();

    m2DryRaw = rawMoisture2;

    Blynk.virtualWrite(VPIN_M2_DRY, m2DryRaw);
    Blynk.virtualWrite(VPIN_SET_M2_DRY, 0);

    Serial.print("APP -> ESP | V88 | S2 TROCKEN gespeichert = ");
    Serial.println(m2DryRaw);

    setSystemState("S2 Trockenwert gespeichert: " + String(m2DryRaw));
    printMoistureCalibrationToSerial();
  }
}

BLYNK_WRITE(VPIN_SET_M2_WET) {
  if (param.asInt() == 1) {
    readSensors();

    m2WetRaw = rawMoisture2;

    Blynk.virtualWrite(VPIN_M2_WET, m2WetRaw);
    Blynk.virtualWrite(VPIN_SET_M2_WET, 0);

    Serial.print("APP -> ESP | V89 | S2 NASS gespeichert = ");
    Serial.println(m2WetRaw);

    setSystemState("S2 Nasswert gespeichert: " + String(m2WetRaw));
    printMoistureCalibrationToSerial();
  }
}

BLYNK_WRITE(VPIN_SET_M3_DRY) {
  if (param.asInt() == 1) {
    readSensors();

    m3DryRaw = rawMoisture3;

    Blynk.virtualWrite(VPIN_M3_DRY, m3DryRaw);
    Blynk.virtualWrite(VPIN_SET_M3_DRY, 0);

    Serial.print("APP -> ESP | V90 | S3 TROCKEN gespeichert = ");
    Serial.println(m3DryRaw);

    setSystemState("S3 Trockenwert gespeichert: " + String(m3DryRaw));
    printMoistureCalibrationToSerial();
  }
}

BLYNK_WRITE(VPIN_SET_M3_WET) {
  if (param.asInt() == 1) {
    readSensors();

    m3WetRaw = rawMoisture3;

    Blynk.virtualWrite(VPIN_M3_WET, m3WetRaw);
    Blynk.virtualWrite(VPIN_SET_M3_WET, 0);

    Serial.print("APP -> ESP | V91 | S3 NASS gespeichert = ");
    Serial.println(m3WetRaw);

    setSystemState("S3 Nasswert gespeichert: " + String(m3WetRaw));
    printMoistureCalibrationToSerial();
  }
}

void printMoistureCalibrationToSerial() {
  Serial.println("----- FEUCHTE-KALIBRIERUNG -----");

  Serial.print("S1 trocken: ");
  Serial.print(m1DryRaw);
  Serial.print(" | nass: ");
  Serial.println(m1WetRaw);

  Serial.print("S2 trocken: ");
  Serial.print(m2DryRaw);
  Serial.print(" | nass: ");
  Serial.println(m2WetRaw);

  Serial.print("S3 trocken: ");
  Serial.print(m3DryRaw);
  Serial.print(" | nass: ");
  Serial.println(m3WetRaw);
  
 

  Serial.println("--------------------------------");
}

BLYNK_WRITE(VPIN_M1_DRY) {
  m1DryRaw = constrain(param.asInt(), 0, 4095);
  Serial.print("BLYNK -> ESP | V80 | S1 trocken direkt gesetzt = ");
  Serial.println(m1DryRaw);
}

BLYNK_WRITE(VPIN_M1_WET) {
  m1WetRaw = constrain(param.asInt(), 0, 4095);
  Serial.print("BLYNK -> ESP | V81 | S1 nass direkt gesetzt = ");
  Serial.println(m1WetRaw);
}

BLYNK_WRITE(VPIN_M2_DRY) {
  m2DryRaw = constrain(param.asInt(), 0, 4095);
  Serial.print("BLYNK -> ESP | V82 | S2 trocken direkt gesetzt = ");
  Serial.println(m2DryRaw);
}

BLYNK_WRITE(VPIN_M2_WET) {
  m2WetRaw = constrain(param.asInt(), 0, 4095);
  Serial.print("BLYNK -> ESP | V83 | S2 nass direkt gesetzt = ");
  Serial.println(m2WetRaw);
}

BLYNK_WRITE(VPIN_M3_DRY) {
  m3DryRaw = constrain(param.asInt(), 0, 4095);
  Serial.print("BLYNK -> ESP | V84 | S3 trocken direkt gesetzt = ");
  Serial.println(m3DryRaw);
}

BLYNK_WRITE(VPIN_M3_WET) {
  m3WetRaw = constrain(param.asInt(), 0, 4095);
  Serial.print("BLYNK -> ESP | V85 | S3 nass direkt gesetzt = ");
  Serial.println(m3WetRaw);
}

void printDebugValues();

void scanI2C() {
  Serial.println();
  Serial.println("----- I2C SCAN -----");
  Serial.print("SDA GPIO: ");
  Serial.println(PIN_I2C_SDA);
  Serial.print("SCL GPIO: ");
  Serial.println(PIN_I2C_SCL);

  int found = 0;

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C Geraet gefunden: 0x");

      if (address < 16) {
        Serial.print("0");
      }

      Serial.println(address, HEX);
      found++;
    }
  }

  if (found == 0) {
    Serial.println("KEIN I2C-Geraet gefunden!");
  }

  Serial.println("--------------------");
  Serial.println();
}

void initBatteryMonitor() {
  Serial.println("MAX17048 wird initialisiert...");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);

  delay(100);

  // Erst den gesamten Bus untersuchen
  scanI2C();

  // MAX17048 gezielt testen
  Wire.beginTransmission(0x36);
  uint8_t error = Wire.endTransmission();

  Serial.print("Direkter Test Adresse 0x36, Ergebnis = ");
  Serial.println(error);

  if (error != 0) {
    Serial.println("FEHLER: MAX17048 antwortet nicht auf 0x36.");
    batteryMonitorOk = false;
    return;
  }

  Serial.println("Adresse 0x36 antwortet.");

  batteryMonitorOk = batteryMonitor.begin(&Wire);

  if (!batteryMonitorOk) {
    Serial.println("FEHLER: Adresse 0x36 antwortet, aber MAX17048-Library begin() schlaegt fehl.");
    return;
  }

  Serial.println("MAX17048 gefunden.");

  batteryMonitor.wake();

  Serial.println("MAX17048 aufgeweckt.");

  Serial.print("Chip ID: 0x");
  Serial.println(batteryMonitor.getChipID(), HEX);
}

/************************************************************
  Setup
************************************************************/

void setup()
{
  Serial.begin(115200);
  delay(100);

  handleSolarWakeReset();

  // ----------------------------------------------------------
  // GPIO15 nach Deep-Sleep-Wakeup sicher übernehmen
  // ----------------------------------------------------------

  // Gewünschter Zustand nach dem Aufwachen:
  // Sensor-Boost soll zunächst AUS bleiben
  pinMode(PIN_SENSOR_POWER, OUTPUT);
  digitalWrite(PIN_SENSOR_POWER, LOW);

  // Deep-Sleep-Hold deaktivieren
  gpio_deep_sleep_hold_dis();

  // GPIO15-Hold lösen.
  // Da Mode und Pegel vorher gesetzt wurden,
  // bleibt GPIO15 dabei LOW.
  gpio_hold_dis((gpio_num_t)PIN_SENSOR_POWER);


  // ----------------------------------------------------------
  // Sichere Ausgangszustaende
  // ----------------------------------------------------------

  pinMode(PIN_PUMP, OUTPUT);
  digitalWrite(PIN_PUMP, LOW);


  // UART vorbereiten
  WaterSerial.begin(
  9600,
  SERIAL_8N1,
  PIN_WATER_RX,
  -1
  );
  WaterSerial.setTimeout(20);


  Serial.println();
  Serial.println("====================================");
  Serial.println("ESP32 Bewaesserungssystem startet");
  Serial.print("FW_VERSION: ");
  Serial.println(FW_VERSION);
  Serial.println("====================================");


  // ----------------------------------------------------------
  // Sensorversorgung einschalten
  // ----------------------------------------------------------

  pinMode(PIN_LED_RING, OUTPUT);
  digitalWrite(PIN_LED_RING, LOW);

  sensorPowerOn();

  // ----------------------------------------------------------
  // LED-Ring initialisieren
  // ----------------------------------------------------------

  ring.begin();
  ring.setBrightness(LED_RING_BRIGHTNESS);
  ringInitialized = true;

  // Bis die Betriebsart V35 von Blynk bekannt ist,
  // bleibt der Ring ausgeschaltet.
  ringOff();

  analogReadResolution(12);

  // Erst nach eingeschalteter Sensorversorgung
  tempSensor.begin();

  initBatteryMonitor();


  Blynk.begin(
    BLYNK_AUTH_TOKEN,
    ssid,
    pass
  );


  configTzTime(
    "CET-1CEST,M3.5.0/2,M10.5.0/3",
    "pool.ntp.org",
    "time.nist.gov"
  );


  timer.setInterval(1000L, controlLoop);
  timer.setInterval(500L, pumpSafetyCheck);
  timer.setInterval(5000L, readBatteryMonitor);
  timer.setInterval(5000L, printDebugValues);


  setSystemState("System bereit");
}

void readBatteryMonitor()
{
  // ----------------------------------------------------------
  // Monitor momentan nicht verfügbar?
  // Dann automatisch Wiederherstellung versuchen.
  // ----------------------------------------------------------
  if (!batteryMonitorOk)
  {
    Serial.println("[BAT] MAX17048 nicht bereit -> Reinitialisierung");

    initBatteryMonitor();

    if (!batteryMonitorOk)
    {
      Serial.println("[BAT] Reinitialisierung fehlgeschlagen");
      return;
    }

    Serial.println("[BAT] MAX17048 wieder verfuegbar");
  }

  float voltage = batteryMonitor.cellVoltage();
  float percent = batteryMonitor.cellPercent();

  // ----------------------------------------------------------
  // Ungueltiger Messwert
  // ----------------------------------------------------------
  if (isnan(voltage) || isnan(percent))
  {
    batteryReadErrorCount++;

    Serial.print("[BAT] Ungueltiger Messwert ");
    Serial.print(batteryReadErrorCount);
    Serial.print("/");
    Serial.println(BATTERY_MAX_READ_ERRORS);

    // Ein einzelner Fehler darf den Monitor NICHT deaktivieren
    if (batteryReadErrorCount >= BATTERY_MAX_READ_ERRORS)
    {
      Serial.println("[BAT] Zu viele Fehler -> MAX17048 neu initialisieren");

      batteryMonitorOk = false;
      batteryReadErrorCount = 0;

      initBatteryMonitor();
    }

    return;
  }

  // ----------------------------------------------------------
  // Messung erfolgreich
  // ----------------------------------------------------------
  batteryReadErrorCount = 0;

  batteryVoltage = voltage;
  batteryPercent = constrain(percent, 0.0f, 100.0f);

  checkBatteryAlarm();
}

void printDebugValues() {
  

  time_t now = time(nullptr);
  struct tm timeinfo;

if (now >= 1700000000) {
  localtime_r(&now, &timeinfo);

  char timeBuffer[20];
  strftime(timeBuffer, sizeof(timeBuffer), "%d.%m.%Y %H:%M:%S", &timeinfo);

  Serial.print("----- ");
  Serial.print(timeBuffer);
  Serial.println(" -----");
} else {
  Serial.println("----- Uhrzeit noch nicht verfuegbar -----");
}

  Serial.print("Modus: ");
  Serial.println(modeToText(currentMode));

  //Serial.print("Messaging active: ");
  //Serial.print(messagingActive);
  //Serial.println(" %");

  //Serial.print("Feuchte 1: ");
  //Serial.print(moisture1Pct);
  //Serial.print(" %   raw: ");
  //Serial.println(rawMoisture1);

  //Serial.print("Feuchte 2: ");
  //Serial.print(moisture2Pct);
  //Serial.print(" %   raw: ");
  //Serial.println(rawMoisture2);

  //Serial.print("Feuchte 3: ");
  //Serial.print(moisture3Pct);
  //Serial.print(" %   raw: ");
  //Serial.println(rawMoisture3);

  //Serial.print("Start ab Feuchte: ");
  //Serial.print(autoStartMoisturePct);
  //Serial.print(" | Stopp bei Feuchte: ");
  //Serial.print(autoStopMoisturePct);

  Serial.print("Solar Reset beim Wake: ");
  Serial.println(solarResetThisBoot ? "JA" : "NEIN");

  Serial.print("Solar Reset Zaehler: ");
  Serial.println(solarResetCounter);

  Serial.print("Feuchte Mittelwert: ");
  Serial.print(moistureAvgPct);
  Serial.println(" %");

  Serial.print("Wasserstand: ");
  Serial.print(waterLiters);
  Serial.print(" L   Abstand: ");
  Serial.print(rawWaterLevel);
  Serial.println(" mm");
  Serial.print("Kalibrierung: LEER = ");
  Serial.print(waterEmptyRaw);
  Serial.print(" mm   VOLL = ");
  Serial.print(waterFullRaw);
  Serial.println(" mm");
  Serial.print("Wasserstand: ");
  Serial.print(waterLiters, 2);
  Serial.print(" L   ");
  Serial.print(getWaterLevelPercent(), 2);
  Serial.print(" %   Warnschwelle: ");
  Serial.print(messagingWaterWarningPct);
  Serial.println(" %");

  Serial.print("Temperatur: ");
  if (isnan(temperatureC)) {
    Serial.println("ungueltig / kein DS18B20 erkannt");
  } else {
    Serial.print(temperatureC);
    Serial.println(" °C");
  }

  //Serial.print("Pumpe: ");
  //Serial.println(pumpOn ? "AN" : "AUS");

  Serial.print("SystemState: ");
  Serial.println(systemStateText);

  Serial.print("Akku: ");

if (!batteryMonitorOk) {
  Serial.println("MAX17048 nicht verfuegbar");
} else {
  Serial.print(batteryVoltage, 3);
  Serial.print(" V   ");
  Serial.print(batteryPercent, 1);
  Serial.println(" %");
}

  Serial.println("------------------------");
}

/************************************************************
  Loop
************************************************************/

void loop() {

  
  Blynk.run();
  timer.run();
}