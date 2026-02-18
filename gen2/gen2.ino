/**
 * Interactive: building up to Beaker Interactive
 * File: gen2.ino
 * Description: multi-tag simultaneous read on keyboard "enter" key press
 *
 * Author: Isai Sanchez (library written by paulvha)
 * Date: 2-8-26
 * Board used: Waveshare's ESP32-S3-ETH module
 * Notes:
 * - Uses CONTINUOUS READING mode with proper Gen2 parameters:
 *   -- Session S1   → tag flags persist ~500ms-5s, suppressing re-reads
 *   -- Target AB    → inventory A until exhausted, then B, then repeat
 *   -- Dynamic Q=2  → starts with 4 slots, auto-adjusts for population
 * - Approach:
 *   -- On keyboard press → open a collection window → accumulate unique EPCs
 *    via check()/parseResponse() → match against known puck EPCs → light LEDs.
 * - Uncompatibility:
 *   -- readTagEPC()/readData() uses an 8-byte command format that may be 
 *    incompatible with the M7E Hecto (SparkFun's M7E update added 3 required 
 *    bytes that paulvha's readData() does not include).
 * - Puck-to-LED Mapping:
 *   -- Yellow LED (GPIO 1)  ← EPC: E2 80 68 94 00 00 40 33 56 39 29 0A
 *   -- Blue   LED (GPIO 2)  ← EPC: E2 80 68 94 00 00 40 33 56 38 ED 0A
 *   -- Green  LED (GPIO 3)  ← EPC: E2 80 68 94 00 00 50 33 56 39 2D 0A
 *   -- Red    LED (GPIO 15) ← EPC: E2 80 68 94 00 00 50 33 56 38 F1 0A
 *
 * (c) Thanksgiving Point Exhibits Electronics Team — 2025
*/

#include "src/SparkFun_UHF_RFID_Reader.h"

// ===== HARDWARE CONFIG =====
#define RFID_REGION REGION_NORTHAMERICA

constexpr uint32_t RFID_BAUD = 115200;
constexpr uint8_t RXD1 = 18;  // ESP32-S3 RX ← M7E
constexpr uint8_t TXD1 = 17;  // ESP32-S3 TX → M7E RX
constexpr uint8_t PIN_LED_YELLOW = 1;
constexpr uint8_t PIN_LED_BLUE = 2;
constexpr uint8_t PIN_LED_GREEN = 3;
constexpr uint8_t PIN_LED_RED = 15;

// ===== RFID SCAN CONFIG =====
constexpr uint8_t MAX_TAGS = 10;           // Maximum unique tags to track
constexpr uint8_t EPC_MAX_BYTES = 12;      // Standard EPC length (96 bits)
constexpr uint32_t SCAN_WINDOW_MS = 500;   // How long to collect tags after trigger (ms)
constexpr uint32_t READ_POWER = 1500;      // adjust during testing (ex 1500 = 15.00 dBm)
constexpr uint32_t LED_DISPLAY_MS = 3000;  // How long LEDs stay lit after a scan

// ===== KNOWN PUCK EPCs =====
constexpr uint8_t NUM_PUCKS = 4;
constexpr uint8_t EPC_LENGTH = 12;
static const uint8_t PUCK_EPC_YELLOW[EPC_LENGTH] = {
  0xE2, 0x80, 0x68, 0x94, 0x00, 0x00, 0x40, 0x33, 0x56, 0x39, 0x29, 0x0A
};
static const uint8_t PUCK_EPC_BLUE[EPC_LENGTH] = {
  0xE2, 0x80, 0x68, 0x94, 0x00, 0x00, 0x40, 0x33, 0x56, 0x38, 0xED, 0x0A
};
static const uint8_t PUCK_EPC_GREEN[EPC_LENGTH] = {
  0xE2, 0x80, 0x68, 0x94, 0x00, 0x00, 0x50, 0x33, 0x56, 0x39, 0x2D, 0x0A
};
static const uint8_t PUCK_EPC_RED[EPC_LENGTH] = {
  0xE2, 0x80, 0x68, 0x94, 0x00, 0x00, 0x50, 0x33, 0x56, 0x38, 0xF1, 0x0A
};

// ===== PUCK STRUCT =====
struct PuckConfig {
  const uint8_t *epc;
  uint8_t ledPin;
  const char *name;
  bool detected;
};

static PuckConfig pucks[NUM_PUCKS] = {
  { PUCK_EPC_YELLOW, PIN_LED_YELLOW, "Yellow", false },
  { PUCK_EPC_BLUE, PIN_LED_BLUE, "Blue", false },
  { PUCK_EPC_GREEN, PIN_LED_GREEN, "Green", false },
  { PUCK_EPC_RED, PIN_LED_RED, "Red", false },
};

// ===== TAG STORAGE =====
struct DetectedTag {
  byte epc[EPC_MAX_BYTES];
  uint8_t epcLen;
  int8_t rssi;
  uint32_t lastSeenMs;
};

DetectedTag tagInventory[MAX_TAGS];
int tagCount = 0;
bool readerRunning = false;

RFID rfidModule;

// ─── Module Initialization ──────────────────────────────────────────────────
bool initializeModule() {
  rfidModule.begin(Serial1, ThingMagic_M7E_HECTO);
  delay(200);

  // Drain any startup noise from the module
  while (Serial1.available()) { Serial1.read(); }

  // Attempt communication
  rfidModule.getVersion();

  if (rfidModule.msg[0] == ERROR_WRONG_OPCODE_RESPONSE) {
    // Module was in continuous read mode from a previous session
    Serial.println(F("  Module was mid-read, sending stop command..."));
    rfidModule.stopReading();
    delay(1500);  // Wait for module to fully stop
    rfidModule.getVersion();
  }

  if (rfidModule.msg[0] != ALL_GOOD) {
    // Module might be at a different baud rate. Try the most common default.
    Serial.println(F("  No response at 115200, trying module reset..."));
    Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
    delay(100);
    rfidModule.getVersion();

    if (rfidModule.msg[0] != ALL_GOOD) {
      return false;
    }
  }

  // Print firmware version for diagnostics
  Serial.print(F("  Module firmware: "));
  for (uint8_t i = 5; i < rfidModule.msg[1] + 3; i++) {
    if (rfidModule.msg[i] < 0x10) Serial.print('0');
    Serial.print(rfidModule.msg[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  // Core module configuration (order matters)
  Serial.println(F("  Configuring module..."));
  rfidModule.setTagProtocol();        // GEN2 (0x05)
  rfidModule.setAntennaPort();        // TX=1, RX=1
  rfidModule.setAntennaSearchList();  // Configure antenna search list
  rfidModule.setRegion(RFID_REGION);
  rfidModule.setReadPower(READ_POWER);

  Serial.print(F("  Read power set to "));
  Serial.print(READ_POWER / 100);
  Serial.print('.');
  Serial.print(READ_POWER % 100);
  Serial.println(F(" dBm"));

  return true;
}

// ─── Gen2 Parameter Configuration ───────────────────────────────────────────
void configureGen2Parameters() {
  Serial.println(F("\nConfiguring Gen2 parameters:"));

  Serial.print(F("  Session S1 ......... "));
  printResult(rfidModule.setGen2Session(TMR_GEN2_SESSION_S1));

  Serial.print(F("  Target AB .......... "));
  printResult(rfidModule.setGen2Target(TMR_GEN2_TARGET_AB));

  Serial.print(F("  Q Dynamic (init=4) . "));
  printResult(rfidModule.setGen2Q(TMR_SR_GEN2_Q_DYNAMIC, 3, true));

  Serial.print(F("  RFMode 250/M4/20 ... "));
  printResult(rfidModule.setGen2RFmode(TMR_GEN2_RFMODE_250_M4_20));
}

// ─── Multi-Tag Scan (Continuous Read Approach) ──────────────────────────────
void performScan() {
  Serial.println(F("\n>>> SCANNING <<<\n"));

  // Reset tag inventory and puck detection flags
  tagCount = 0;
  memset(tagInventory, 0, sizeof(tagInventory));
  for (int i = 0; i < NUM_PUCKS; i++) {
    pucks[i].detected = false;
    digitalWrite(pucks[i].ledPin, LOW);
  }

  // Start continuous reading — module will stream tag records over UART
  rfidModule.startReading();
  readerRunning = true;

  uint32_t scanStart = millis();
  uint16_t totalReads = 0;
  uint16_t parseErrors = 0;
  uint8_t pucksFound = 0;

  // ── Collection window: poll for tags until time expires (with early exit if all found) ──
  while (millis() - scanStart < SCAN_WINDOW_MS) {

    if (rfidModule.check()) {

      uint8_t responseType = rfidModule.parseResponse();

      if (responseType == RESPONSE_IS_TAGFOUND) {
        totalReads++;

        uint8_t tagDataBytes = rfidModule.getTagDataBytes();
        uint8_t epcBytes = rfidModule.getTagEPCBytes();
        int8_t rssi = rfidModule.getTagRSSI();

        if (epcBytes == 0 || epcBytes > EPC_MAX_BYTES) {
          parseErrors++;
          continue;
        }

        uint8_t epcStartIdx = 31 + tagDataBytes;

        // Deduplicate
        if (findTagByEPC(&rfidModule.msg[epcStartIdx], epcBytes) >= 0) continue;
        if (tagCount >= MAX_TAGS) continue;

        // Store new tag
        DetectedTag *tag = &tagInventory[tagCount];
        memcpy(tag->epc, &rfidModule.msg[epcStartIdx], epcBytes);
        tag->epcLen = epcBytes;
        tag->rssi = rssi;
        tag->lastSeenMs = millis();
        tagCount++;

        Serial.print(F("  [NEW] Tag #"));
        Serial.print(tagCount);
        Serial.print(F(" | RSSI: "));
        Serial.print(rssi);
        Serial.print(F(" dBm | EPC: "));
        printEPC(tag->epc, tag->epcLen);

        // Check if this is a known puck
        int puckIdx = matchPuck(tag->epc, tag->epcLen);
        if (puckIdx >= 0 && !pucks[puckIdx].detected) {
          pucks[puckIdx].detected = true;
          pucksFound++;

          // All pucks found — no reason to keep scanning
          if (pucksFound >= NUM_PUCKS) break;
        }
      } else if (responseType == RESPONSE_IS_TEMPTHROTTLE) {
        Serial.println(F("  WARNING: Thermal throttling!"));
      } else if (responseType == ERROR_CORRUPT_RESPONSE) {
        parseErrors++;
      }
    }

    yield();
  }

  uint32_t scanElapsed = millis() - scanStart;

  // ── Stop continuous reading — fast drain ──
  rfidModule.stopReading();
  readerRunning = false;

  // Brief settle instead of the old delay(500)
  delay(50);
  while (Serial1.available()) { Serial1.read(); }

  // ── Results ──
  Serial.println(F("\n────────────────────────────────────────"));
  Serial.print(F("SCAN COMPLETE: "));
  Serial.print(tagCount);
  Serial.print(F(" unique tag(s), "));
  Serial.print(pucksFound);
  Serial.print(F("/"));
  Serial.print(NUM_PUCKS);
  Serial.print(F(" pucks in "));
  Serial.print(scanElapsed);
  Serial.println(F(" ms"));
  Serial.print(F("  Total reads: "));
  Serial.print(totalReads);
  if (parseErrors > 0) {
    Serial.print(F("  Errors: "));
    Serial.print(parseErrors);
  }
  Serial.println();
  Serial.println(F("────────────────────────────────────────"));

  if (tagCount > 0) {
    Serial.println(F("\nAll Detected Tags:"));
    for (int i = 0; i < tagCount; i++) {
      Serial.print(F("  #"));
      Serial.print(i + 1);
      Serial.print(F(" | RSSI: "));
      char rssiBuf[8];
      snprintf(rssiBuf, sizeof(rssiBuf), "%4d", tagInventory[i].rssi);
      Serial.print(rssiBuf);
      Serial.print(F(" dBm | EPC: "));
      printEPC(tagInventory[i].epc, tagInventory[i].epcLen);
    }
  } else {
    Serial.println(F("\nNo tags detected."));
    Serial.print(F("  → Try increasing READ_POWER (currently "));
    Serial.print(READ_POWER);
    Serial.println(F(")."));
  }

  // ── Light LEDs ──
  if (pucksFound > 0) {
    Serial.print(F("\nLEDs ON: "));
    for (int i = 0; i < NUM_PUCKS; i++) {
      if (pucks[i].detected) {
        digitalWrite(pucks[i].ledPin, HIGH);
        Serial.print(pucks[i].name);
        Serial.print(' ');
      }
    }
    Serial.println();

    delay(LED_DISPLAY_MS);

    for (int i = 0; i < NUM_PUCKS; i++) {
      digitalWrite(pucks[i].ledPin, LOW);
    }
    Serial.println(F("LEDs OFF."));
  }

  Serial.println(F("\nPress any key to scan again...\n"));
}

// ─── Tag Lookup ─────────────────────────────────────────────────────────────
int findTagByEPC(byte *epc, uint8_t epcLen) {
  for (int i = 0; i < tagCount; i++) {
    if (tagInventory[i].epcLen == epcLen && memcmp(tagInventory[i].epc, epc, epcLen) == 0) {
      return i;
    }
  }
  return -1;
}

// ─── Utility ────────────────────────────────────────────────────────────────
void printEPC(byte *epc, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if (epc[i] < 0x10) Serial.print('0');
    Serial.print(epc[i], HEX);
    if (i < len - 1) Serial.print(' ');
  }
  Serial.println();
}

void printResult(bool success) {
  Serial.println(success ? F("OK") : F("FAILED"));
}

void flushSerialInput() {
  delay(50);
  while (Serial.available()) {
    Serial.read();
    delay(5);
  }
}

// Checks if an EPC matches any known puck
int matchPuck(byte *epc, uint8_t epcLen) {
  if (epcLen != EPC_LENGTH) return -1;
  for (int i = 0; i < NUM_PUCKS; i++) {
    if (memcmp(epc, pucks[i].epc, EPC_LENGTH) == 0) return i;
  }
  return -1;
}

/*
* =======================
*         MAIN 
* =======================
*/

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println(F("\n========================================"));
  Serial.println(F("  Multi-Tag Scan — Continuous Read Mode"));
  Serial.println(F("========================================\n"));

  // Configure LED pins
  for (int i = 0; i < NUM_PUCKS; i++) {
    pinMode(pucks[i].ledPin, OUTPUT);
    digitalWrite(pucks[i].ledPin, LOW);
  }

  // Initialize UART to M7E with explicit ESP32-S3 pin mapping
  Serial1.begin(RFID_BAUD, SERIAL_8N1, RXD1, TXD1);

  if (!initializeModule()) {
    Serial.println(F("FATAL: RFID module not responding. Check wiring and power."));
    while (1) { delay(1000); }
  }

  configureGen2Parameters();

  // Brief LED test — all on, then off
  Serial.println(F("\nLED test..."));
  for (int i = 0; i < NUM_PUCKS; i++) digitalWrite(pucks[i].ledPin, HIGH);
  delay(500);
  for (int i = 0; i < NUM_PUCKS; i++) digitalWrite(pucks[i].ledPin, LOW);

  Serial.println(F("\n────────────────────────────────────────"));
  Serial.println(F("Place pucks in beaker, then press any key to scan."));
  Serial.println(F("────────────────────────────────────────\n"));
}

void loop() {
  if (Serial.available()) {
    flushSerialInput();
    performScan();
  }
}
