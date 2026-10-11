#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <LittleFS.h>

// ============================================================
//                 ESP32 ROCKET AVIONICS SYSTEM
// ============================================================
//
// FEATURES
// 1. ESP32-WROOM-32
// 2. SX1278 / RA-02 LoRa
// 3. BMP280 (altitude zeroed at boot = launch pad is 0 m)
// 4. D13 wire-cut release interrupt
// 5. Buzzer (BUZZER_PIN)
// 6. D27 (EVENT 1) / D12 (EVENT 2) SAFE TEST OUTPUTS ONLY
// 7. Timer starts when the wire is cut
// 8. Buzzer: one long beep at power-on when BMP280 and LoRa are both
//    initialised; after wire cut it beeps once per second for the
//    first 3 seconds only
// 9. EVENT 1 fires 12 s after wire cut, EVENT 2 fires automatically
//    3 s after EVENT 1 (= 15 s after wire cut), no dependency on feedback
// 10. LoRa telemetry at 2 packets/sec (compact binary protocol,
//     compatible with the reference receiver)
// 11. Complete internal flash logging
// 12. Serial flight-data dump ('D') and erase ('E')
//
// WIRING NOTES
// - D13: wire loop to GND. Wire connected = LOW. Wire cut = pulled
//   HIGH by INPUT_PULLUP -> RISING interrupt.
// - D34: EVENT 1 feedback (input-only pin, NO internal pull).
//   Use an external pull-down (10k to GND). INFORMATIONAL ONLY.
// - D27/D12 are logic-level test outputs only. They do NOT drive
//   pyrotechnics directly.
// - D12 is a strapping pin: nothing may pull it HIGH at boot.
// - Status LED is on D15 (NOT D13, which is the release input).
// - Power on the rocket at the launch pad, at rest, with the wire
//   connected: ground pressure is measured at boot (about 1.5 s).
// ============================================================


// ============================================================
//                     PIN CONFIGURATION
// ============================================================

// LoRa SX1278
constexpr uint8_t LORA_CS   = 5;
constexpr uint8_t LORA_RST  = 14;
constexpr uint8_t LORA_DIO0 = 26;
constexpr uint8_t LORA_SCK  = 18;
constexpr uint8_t LORA_MISO = 19;
constexpr uint8_t LORA_MOSI = 23;

// BMP280
constexpr uint8_t BMP_SDA = 21;
constexpr uint8_t BMP_SCL = 22;

// Release (wire cut) input
constexpr uint8_t RELEASE_PIN = 13;

// Event outputs (safe test outputs)
constexpr uint8_t EVENT_TEST_1 = 27;
constexpr uint8_t EVENT_TEST_2 = 12;

// Event 1 feedback input (informational only)
constexpr uint8_t EVENT1_FEEDBACK_PIN = 34;

// Buzzer (change to 32 if that is where you wired it)
constexpr uint8_t BUZZER_PIN = 25;

// Status LED (moved off D13 to avoid clashing with the release input)
constexpr uint8_t STATUS_LED = 15;


// ============================================================
//        LORA PARAMETERS  (MUST MATCH THE RECEIVER EXACTLY)
// ============================================================
// >>> Copy these values from the receiver / reference Config.h <<<

constexpr long     LORA_FREQUENCY       = 434500000L;  // LORA_FREQ
constexpr long     LORA_BANDWIDTH       = 125000L;     // LORA_BANDWIDTH
constexpr uint8_t  LORA_SPREADING_FACTOR = 7;          // LORA_SPREADFACTOR
constexpr uint8_t  LORA_CODING_RATE      = 5;          // LORA_CODINGRATE
constexpr uint16_t LORA_PREAMBLE         = 8;          // LORA_PREAMBLE
constexpr int      LORA_SYNC_WORD        = 0xF3;       // LORA_SYNC_WORD  <-- CHANGE to receiver's value
constexpr int8_t   LORA_TX_POWER         = 10;         // dBm, PA_BOOST pin (RA-02)


// ============================================================
//        PACKET PROTOCOL  (MUST MATCH THE RECEIVER EXACTLY)
// ============================================================
// >>> Verify against the receiver's Protocol.h <<<

constexpr uint8_t ROCKET_ID                = 1;  // ROCKET_ID in Config.h
constexpr uint8_t PKT_TYPE_NORMAL_ALTITUDE = 0;  // values from Protocol.h
constexpr uint8_t PKT_TYPE_APOGEE          = 1;  // (not used by this sequencer)
constexpr uint8_t PKT_TYPE_EVENT1          = 2;  // NEW: only if receiver is updated
constexpr uint8_t PKT_TYPE_EVENT2          = 3;  // NEW: only if receiver is updated

// Set true ONLY after the receiver knows the EVENT1/EVENT2 packet types.
// While false, the receiver sees only normal altitude packets.
constexpr bool SEND_EVENT_PACKETS = false;

#pragma pack(push, 1)
struct TelemetryPacket
{
  uint8_t  rocket_id;
  uint8_t  type;
  uint16_t sequence;   // big-endian on air
  uint16_t altitude;   // metres x 10, big-endian on air
};
#pragma pack(pop)

uint16_t swapEndian(uint16_t val)
{
  return (val << 8) | (val >> 8);
}


// ============================================================
//                    TIMING PARAMETERS
// ============================================================

constexpr unsigned long TELEMETRY_PERIOD = 500UL;      // 2 packets/sec
constexpr unsigned long EVENT_DELAY = 12000UL;         // wire cut -> event 1 (12 s)
constexpr unsigned long EVENT_GAP = 2000UL;            // event 1 -> event 2 (3 s, i.e. 15 s total)
constexpr unsigned long WIRECUT_BEEP_PERIOD = 1000UL;  // buzzer every second
constexpr unsigned long BEEP_DURATION = 120UL;
constexpr unsigned long WIRECUT_BEEP_WINDOW = 3000UL;  // beeps only during first 3 s after wire cut
constexpr unsigned long BOOT_BEEP_DURATION = 1000UL;   // power-on 'all OK' beep


// ============================================================
//                    BMP280 / FLASH
// ============================================================

// Ground pressure measured at boot; altitude is relative to this (0 m at pad)
float groundPressure = 1013.25F;   // hPa (fallback if calibration fails)
bool groundCalibrated = false;
constexpr uint8_t CALIBRATION_SAMPLES = 50;

const char *FLIGHT_LOG = "/flight.csv";


// ============================================================
//                    OBJECTS
// ============================================================

Adafruit_BMP280 bmp;


// ============================================================
//                    SYSTEM STATUS
// ============================================================

bool bmpOK = false;
bool loraOK = false;
bool flashOK = false;


// ============================================================
//                    INTERRUPT VARIABLES
// ============================================================

volatile bool releaseInterrupt = false;
volatile unsigned long releaseTimestamp = 0;


// ============================================================
//                    SYSTEM TIMERS
// ============================================================

unsigned long bootTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long packetID = 0;
uint16_t packetSequence = 0;


// ============================================================
//                    EVENT STATE
// ============================================================

enum EventState
{
  EVENT_IDLE,
  EVENT_COUNTDOWN,   // wire cut, counting down to event 1
  EVENT1_FIRED,      // event 1 fired, waiting for event 2
  EVENT2_FIRED       // both events fired
};

EventState eventState = EVENT_IDLE;

unsigned long eventTimerStartTime = 0;
unsigned long event1FireTime = 0;

bool wireCutDetected = false;
unsigned long lastWireCutBeep = 0;


// ============================================================
//                    BUZZER / COUNTDOWN STATE
// ============================================================

bool buzzerActive = false;
unsigned long buzzerStartTime = 0;

int lastCountdownSecond = -1;

String lastLoRaStatus = "NOT_STARTED";


// ============================================================
//                    INTERRUPT SERVICE ROUTINE
// ============================================================

void IRAM_ATTR releaseISR()
{
  releaseInterrupt = true;
  releaseTimestamp = millis();
}


// ============================================================
//                    EVENT STATE TEXT
// ============================================================

const char *eventStateText()
{
  switch (eventState)
  {
    case EVENT_IDLE:      return "IDLE";
    case EVENT_COUNTDOWN: return "COUNTDOWN";
    case EVENT1_FIRED:    return "EVENT1_FIRED";
    case EVENT2_FIRED:    return "EVENT2_FIRED";
  }

  return "UNKNOWN";
}


// ============================================================
//                    BMP280
// ============================================================

bool initializeBMP280()
{
  Wire.begin(BMP_SDA, BMP_SCL);

  if (bmp.begin(0x76))
  {
    Serial.println("[OK] BMP280 detected at 0x76");
    return true;
  }

  if (bmp.begin(0x77))
  {
    Serial.println("[OK] BMP280 detected at 0x77");
    return true;
  }

  Serial.println("[WARNING] BMP280 NOT detected.");
  return false;
}


void configureBMP280()
{
  if (!bmpOK)
    return;

  // Lighter filtering than before so altitude does not lag in a fast ascent
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X8,
    Adafruit_BMP280::FILTER_X4,
    Adafruit_BMP280::STANDBY_MS_1
  );
}


// Averages pressure on the ground so altitude = 0 m at the launch pad.
void calibrateGroundLevel()
{
  if (!bmpOK)
    return;

  // Let the filtered / oversampled sensor settle; discard these readings
  for (uint8_t i = 0; i < 20; i++)
  {
    bmp.readPressure();
    delay(20);
  }

  float sum = 0.0F;
  uint8_t good = 0;

  for (uint8_t i = 0; i < CALIBRATION_SAMPLES; i++)
  {
    float p = bmp.readPressure() / 100.0F;

    if (!isnan(p) && p > 300.0F && p < 1100.0F)
    {
      sum += p;
      good++;
    }

    delay(20);
  }

  if (good > 0)
  {
    groundPressure = sum / good;
    groundCalibrated = true;

    Serial.print("[OK] Ground pressure calibrated: ");
    Serial.print(groundPressure, 2);
    Serial.println(" hPa (altitude = 0 m)");
  }
  else
  {
    Serial.println("[WARNING] Ground calibration FAILED, using default pressure.");
  }
}


float getTemperature()
{
  if (!bmpOK)
    return NAN;

  return bmp.readTemperature();
}


float getPressure()
{
  if (!bmpOK)
    return NAN;

  return bmp.readPressure() / 100.0F;
}


// Altitude above the launch pad (0 m at boot)
float getAltitude()
{
  if (!bmpOK)
    return NAN;

  return bmp.readAltitude(groundPressure);
}


// ============================================================
//                    LORA INITIALIZATION
// ============================================================

bool initializeLoRa()
{
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQUENCY))
  {
    Serial.println("[WARNING] LoRa initialization FAILED.");
    lastLoRaStatus = "INIT_FAILED";
    return false;
  }

  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setPreambleLength(LORA_PREAMBLE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();
  LoRa.setTxPower(LORA_TX_POWER, PA_OUTPUT_PA_BOOST_PIN);

  lastLoRaStatus = "READY";

  Serial.println("[OK] LoRa initialized.");
  Serial.print("Frequency : "); Serial.println(LORA_FREQUENCY);
  Serial.print("Bandwidth : "); Serial.println(LORA_BANDWIDTH);
  Serial.print("Spreading : SF"); Serial.println(LORA_SPREADING_FACTOR);
  Serial.print("Coding    : 4/"); Serial.println(LORA_CODING_RATE);
  Serial.print("Preamble  : "); Serial.println(LORA_PREAMBLE);
  Serial.print("Sync word : 0x"); Serial.println(LORA_SYNC_WORD, HEX);
  Serial.println("CRC       : ENABLED");
  Serial.print("TX Power  : "); Serial.print(LORA_TX_POWER); Serial.println(" dBm (PA_BOOST)");

  return true;
}


// ============================================================
//                    FLASH
// ============================================================

bool initializeFlash()
{
  // false = never auto-format; existing flight data is preserved.
  if (!LittleFS.begin(false))
  {
    Serial.println("[WARNING] LittleFS mount FAILED.");
    return false;
  }

  Serial.println("[OK] ESP32 internal flash mounted.");

  if (!LittleFS.exists(FLIGHT_LOG))
  {
    File file = LittleFS.open(FLIGHT_LOG, FILE_WRITE);

    if (!file)
    {
      Serial.println("[WARNING] Cannot create flight log.");
      return false;
    }

    file.println(
      "TIME_MS,"
      "PACKET_ID,"
      "TEMPERATURE_C,"
      "PRESSURE_HPA,"
      "ALTITUDE_M,"
      "EVENT_STATE,"
      "WIRE_CUT,"
      "BUZZER,"
      "BMP_OK,"
      "LORA_OK,"
      "LORA_STATUS,"
      "RECORD_TYPE"
    );

    file.flush();
    file.close();

    Serial.println("[OK] New flight log created.");
  }
  else
  {
    Serial.println("[OK] Existing flight log found.");
  }

  return true;
}


bool writeFlashRecord(const String &record)
{
  if (!flashOK)
    return false;

  File file = LittleFS.open(FLIGHT_LOG, FILE_APPEND);

  if (!file)
  {
    Serial.println("[FLASH ERROR] Cannot open log.");
    return false;
  }

  file.println(record);
  file.flush();
  file.close();

  return true;
}


// ============================================================
//                    LOG SYSTEM EVENT
// ============================================================

void logSystemEvent(const char *eventName)
{
  if (!flashOK)
    return;

  String record;

  record += String(millis());
  record += ",-,-,-,-,";
  record += eventName;
  record += ",";
  record += wireCutDetected ? "1" : "0";
  record += ",";
  record += buzzerActive ? "1" : "0";
  record += ",";
  record += bmpOK ? "1" : "0";
  record += ",";
  record += loraOK ? "1" : "0";
  record += ",";
  record += lastLoRaStatus;
  record += ",EVENT";

  writeFlashRecord(record);
}


// ============================================================
//                    TELEMETRY
// ============================================================

// Flash record uses the same sensor readings that were just transmitted
String createFlashTelemetryRecord(float temperature, float pressure, float altitude)
{
  String record;

  record += String(millis());
  record += ",";
  record += String(packetID);
  record += ",";
  record += String(temperature, 2);
  record += ",";
  record += String(pressure, 2);
  record += ",";
  record += String(altitude, 2);
  record += ",";
  record += eventStateText();
  record += ",";
  record += wireCutDetected ? "1" : "0";
  record += ",";
  record += buzzerActive ? "1" : "0";
  record += ",";
  record += bmpOK ? "1" : "0";
  record += ",";
  record += loraOK ? "1" : "0";
  record += ",";
  record += lastLoRaStatus;
  record += ",TELEMETRY";

  return record;
}


// Sends one binary packet. Returns true if the radio reported success.
bool transmitPacket(uint8_t type, float altitude)
{
  if (!loraOK)
  {
    lastLoRaStatus = "NOT_AVAILABLE";
    return false;
  }

  // Negative noise on the pad must not wrap the unsigned 16-bit value
  if (isnan(altitude) || altitude < 0.0F)
    altitude = 0.0F;
  if (altitude > 6553.0F)
    altitude = 6553.0F;

  TelemetryPacket pkt;
  pkt.rocket_id = ROCKET_ID;
  pkt.type      = type;
  pkt.sequence  = swapEndian(packetSequence++);
  pkt.altitude  = swapEndian((uint16_t)round(altitude * 10.0F));

  LoRa.beginPacket();
  LoRa.write((uint8_t *)&pkt, sizeof(TelemetryPacket));
  int result = LoRa.endPacket();   // blocking: waits for TX done

  lastLoRaStatus = (result == 1) ? "TX_OK" : "TX_ERROR";
  return (result == 1);
}


void sendTelemetry()
{
  float temperature = getTemperature();
  float pressure    = getPressure();
  float altitude    = getAltitude();

  packetID++;

  bool ok = transmitPacket(PKT_TYPE_NORMAL_ALTITUDE, altitude);

  Serial.println();
  Serial.println("[TELEMETRY]");
  Serial.print("ID=");   Serial.print(packetID);
  Serial.print(" T=");   Serial.print(temperature, 2);
  Serial.print(" P=");   Serial.print(pressure, 2);
  Serial.print(" ALT="); Serial.print(altitude, 2);
  Serial.print(" EVT="); Serial.println(eventStateText());
  Serial.println(ok ? "LoRa: TRANSMITTED" :
                 (loraOK ? "LoRa: TRANSMISSION ERROR" : "LoRa: NOT AVAILABLE"));

  if (flashOK)
  {
    if (!writeFlashRecord(createFlashTelemetryRecord(temperature, pressure, altitude)))
      Serial.println("[WARNING] Telemetry flash write failed.");
  }
}


// Optional one-off packets when an event fires (see SEND_EVENT_PACKETS)
void sendEventPacket(uint8_t type)
{
  if (!SEND_EVENT_PACKETS)
    return;

  transmitPacket(type, getAltitude());
}


// ============================================================
//                    BUZZER
// ============================================================

void startBuzzer()
{
  digitalWrite(BUZZER_PIN, HIGH);

  buzzerActive = true;
  buzzerStartTime = millis();

  Serial.println("BUZZER: HIGH");

  logSystemEvent("BUZZER_ON");
}


void processBuzzer()
{
  if (!buzzerActive)
    return;

  if (millis() - buzzerStartTime >= BEEP_DURATION)
  {
    digitalWrite(BUZZER_PIN, LOW);

    buzzerActive = false;

    Serial.println("BUZZER: LOW");

    logSystemEvent("BUZZER_OFF");
  }
}


// Beep once per second for the first 3 seconds after the wire is cut
// (beeps at 0 s, 1 s, 2 s), then stay silent.
void processWireCutBuzzer()
{
  if (!wireCutDetected)
    return;

  if (millis() - eventTimerStartTime >= WIRECUT_BEEP_WINDOW)
    return;

  if (!buzzerActive &&
      millis() - lastWireCutBeep >= WIRECUT_BEEP_PERIOD)
  {
    lastWireCutBeep = millis();
    startBuzzer();  // processBuzzer() turns it off after BEEP_DURATION
  }
}


// One long beep at power-on to signal BMP280 and LoRa are both ready.
void bootReadyBeep()
{
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerActive = true;
  Serial.println("BUZZER: BOOT READY BEEP");
  logSystemEvent("BOOT_READY_BEEP");

  delay(BOOT_BEEP_DURATION);

  digitalWrite(BUZZER_PIN, LOW);
  buzzerActive = false;
}


// ============================================================
//                    EVENT SEQUENCE
// ============================================================

void startEventSequence()
{
  if (eventState != EVENT_IDLE)
    return;

  eventState = EVENT_COUNTDOWN;
  eventTimerStartTime = millis();
  lastCountdownSecond = -1;

  wireCutDetected = true;
  lastWireCutBeep = millis() - WIRECUT_BEEP_PERIOD;  // first beep immediately

  Serial.println();
  Serial.println("========================================");
  Serial.println("       WIRE CUT DETECTED");
  Serial.println("       EVENT TIMER STARTED");
  Serial.println("========================================");

  logSystemEvent("WIRE_CUT_DETECTED");
}


void processEventSequence()
{
  unsigned long now = millis();

  // ---------------- COUNTDOWN -> EVENT 1 ----------------
  if (eventState == EVENT_COUNTDOWN)
  {
    unsigned long elapsed = now - eventTimerStartTime;

    int second = elapsed / 1000;  // 0,1,2,3...

    if (second < 3 && second != lastCountdownSecond)
    {
      lastCountdownSecond = second;

      Serial.print("COUNTDOWN: ");
      Serial.print(second + 1);
      Serial.println(" second(s)");

      if (second == 0) logSystemEvent("COUNTDOWN_1");
      if (second == 1) logSystemEvent("COUNTDOWN_2");
      if (second == 2) logSystemEvent("COUNTDOWN_3");
    }

    if (elapsed >= EVENT_DELAY)
    {
      Serial.println("EVENT 1: FIRED (test output D27) - 12 s after wire cut");

      digitalWrite(EVENT_TEST_1, HIGH);
      digitalWrite(STATUS_LED, HIGH);

      event1FireTime = now;
      eventState = EVENT1_FIRED;

      logSystemEvent("EVENT1_FIRED");
      sendEventPacket(PKT_TYPE_EVENT1);
    }

    return;
  }

  // ---------------- GAP -> EVENT 2 ----------------
  if (eventState == EVENT1_FIRED)
  {
    if (now - event1FireTime >= EVENT_GAP)
    {
      Serial.println("EVENT 2: FIRED (test output D12) - 3 s after event 1");

      digitalWrite(EVENT_TEST_2, HIGH);

      eventState = EVENT2_FIRED;

      logSystemEvent("EVENT2_FIRED");
      sendEventPacket(PKT_TYPE_EVENT2);

      // Informational only: record what the feedback pin read
      if (digitalRead(EVENT1_FEEDBACK_PIN) == HIGH)
      {
        Serial.println("EVENT 1 FEEDBACK: HIGH");
        logSystemEvent("EVENT1_FEEDBACK_HIGH");
      }
      else
      {
        Serial.println("EVENT 1 FEEDBACK: LOW");
        logSystemEvent("EVENT1_FEEDBACK_LOW");
      }
    }
  }
}


// ============================================================
//                    RELEASE INTERRUPT PROCESSING
// ============================================================

void processReleaseInterrupt()
{
  if (!releaseInterrupt)
    return;

  noInterrupts();
  releaseInterrupt = false;
  unsigned long detectedTime = releaseTimestamp;
  interrupts();

  // Ignore a noise glitch: the pin must still read open (HIGH)
  if (digitalRead(RELEASE_PIN) != HIGH)
  {
    Serial.println("[INTERRUPT] Glitch ignored (pin back to LOW)");
    return;
  }

  Serial.println();
  Serial.println("[INTERRUPT] WIRE CUT DETECTED");

  Serial.print("[INTERRUPT] Time = ");
  Serial.print(detectedTime);
  Serial.println(" ms");

  startEventSequence();
}


// Polling backup for the interrupt. The wire must be seen connected (LOW)
// at least once, then open (HIGH) for 50 ms, to count as a cut.
bool releaseArmed = false;
unsigned long releaseHighSince = 0;

void pollReleasePin()
{
  if (eventState != EVENT_IDLE)
    return;

  if (digitalRead(RELEASE_PIN) == LOW)
  {
    if (!releaseArmed)
      Serial.println("[OK] Release wire connected and armed.");

    releaseArmed = true;
    releaseHighSince = 0;
    return;
  }

  if (!releaseArmed)
    return;   // never seen connected, so do not trigger

  if (releaseHighSince == 0)
  {
    releaseHighSince = millis();
  }
  else if (millis() - releaseHighSince >= 50UL)
  {
    Serial.println("[POLL] WIRE CUT DETECTED");
    startEventSequence();
  }
}


// ============================================================
//                    DUMP / ERASE FLASH DATA
// ============================================================

void dumpFlightLog()
{
  if (!flashOK)
  {
    Serial.println("[ERROR] Flash unavailable.");
    return;
  }

  File file = LittleFS.open(FLIGHT_LOG, FILE_READ);

  if (!file)
  {
    Serial.println("[ERROR] Cannot open flight log.");
    return;
  }

  Serial.println();
  Serial.println("========================================");
  Serial.println("          STORED FLIGHT DATA");
  Serial.println("========================================");

  while (file.available())
  {
    Serial.write(file.read());
  }

  file.close();

  Serial.println();
  Serial.println("========================================");
  Serial.println("          END OF FLIGHT DATA");
  Serial.println("========================================");
}


void eraseFlightLog()
{
  if (!flashOK)
    return;

  LittleFS.remove(FLIGHT_LOG);

  File file = LittleFS.open(FLIGHT_LOG, FILE_WRITE);

  if (file)
  {
    file.println(
      "TIME_MS,PACKET_ID,TEMPERATURE_C,PRESSURE_HPA,ALTITUDE_M,"
      "EVENT_STATE,WIRE_CUT,BUZZER,BMP_OK,LORA_OK,LORA_STATUS,RECORD_TYPE"
    );
    file.flush();
    file.close();
  }

  Serial.println("[OK] Flight log erased.");
}


// Serial commands: 'D' = dump log, 'E' = erase log
void processSerialCommands()
{
  while (Serial.available())
  {
    char c = Serial.read();

    if (c == 'D' || c == 'd')
      dumpFlightLog();
    else if (c == 'E' || c == 'e')
      eraseFlightLog();
  }
}


// ============================================================
//                    SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("========================================");
  Serial.println("      ESP32 ROCKET AVIONICS BOOT");
  Serial.println("========================================");

  // Outputs first, forced LOW
  pinMode(EVENT_TEST_1, OUTPUT);
  pinMode(EVENT_TEST_2, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);

  digitalWrite(EVENT_TEST_1, LOW);
  digitalWrite(EVENT_TEST_2, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(STATUS_LED, LOW);

  // Inputs
  pinMode(RELEASE_PIN, INPUT_PULLUP);       // wire to GND; cut = HIGH
  pinMode(EVENT1_FEEDBACK_PIN, INPUT);      // needs external pull-down

  bootTime = millis();

  bmpOK = initializeBMP280();
  configureBMP280();
  calibrateGroundLevel();                   // altitude = 0 m at the pad

  loraOK = initializeLoRa();

  flashOK = initializeFlash();

  logSystemEvent("BOOT");
  if (groundCalibrated)
    logSystemEvent("GROUND_CAL");

  // Buzz once if the sensors/radio came up correctly
  if (bmpOK && loraOK)
    bootReadyBeep();

  // Warn if the wire is already cut at boot (RISING interrupt won't fire)
  if (digitalRead(RELEASE_PIN) == HIGH)
  {
    Serial.println("[WARNING] Release wire reads CUT/OPEN at boot!");
  }

  attachInterrupt(
    digitalPinToInterrupt(RELEASE_PIN),
    releaseISR,
    RISING
  );

  Serial.println("[READY] Send 'D' to dump flight log, 'E' to erase it.");
}


// ============================================================
//                    LOOP
// ============================================================

void loop()
{
  processReleaseInterrupt();
  pollReleasePin();
  processEventSequence();
  processWireCutBuzzer();
  processBuzzer();
  processSerialCommands();

  unsigned long now = millis();

  if (now - lastTelemetryTime >= TELEMETRY_PERIOD)
  {
    lastTelemetryTime = now;
    sendTelemetry();
  }
}