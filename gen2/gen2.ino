/*
  Multi-Tag Simultaneous Read — Corrected Implementation
  ESP32-S3 + M7E Hecto (using paulvha's library fork)
  
  Uses CONTINUOUS READING mode with proper Gen2 parameters:
    - Session S1   → tag flags persist ~500ms-5s, suppressing re-reads
    - Target AB    → inventory A until exhausted, then B, then repeat
    - Dynamic Q=2  → starts with 4 slots, auto-adjusts for population
  
  Approach:
    On keyboard press → open a collection window → accumulate unique EPCs
    via check()/parseResponse() → match against known puck EPCs → light LEDs.
  
  This avoids readTagEPC()/readData() which uses an 8-byte command format 
  that may be incompatible with M7E Hecto (SparkFun's M7E update added 
  3 required bytes that paulvha's readData() does not include).

  Puck-to-LED Mapping:
    Yellow LED (GPIO 1)  ← EPC: E2 80 68 94 00 00 40 33 56 39 29 0A
    Blue   LED (GPIO 2)  ← EPC: E2 80 68 94 00 00 40 33 56 38 ED 0A
    Green  LED (GPIO 3)  ← EPC: E2 80 68 94 00 00 50 33 56 39 2D 0A
*/

#include "src/SparkFun_UHF_RFID_Reader.h"

// ─── Hardware Configuration ──────────────────────────────────────────────────

#define RXD1 18  // ESP32-S3 RX ← M7E TX
#define TXD1 17  // ESP32-S3 TX → M7E RX
#define RFID_BAUD 115200
#define RFID_REGION REGION_NORTHAMERICA

// ─── LED Pins ───────────────────────────────────────────────────────────────

#define PIN_LED_YELLOW 1
#define PIN_LED_BLUE 2
#define PIN_LED_GREEN 3

// ─── Scan Configuration ─────────────────────────────────────────────────────

#define MAX_TAGS 10          // Maximum unique tags to track
#define EPC_MAX_BYTES 12     // Standard EPC length (96 bits)
#define SCAN_WINDOW_MS 500   // How long to collect tags after trigger (ms)
#define READ_POWER 1000      // adjust during testing (ex 1500 = 15.00 dBm)
#define LED_DISPLAY_MS 3000  // How long LEDs stay lit after a scan

// ─── Known Puck EPCs ────────────────────────────────────────────────────────

#define NUM_PUCKS 3
#define EPC_LENGTH 12

static const uint8_t PUCK_EPC_YELLOW[EPC_LENGTH] = {
  0xE2, 0x80, 0x68, 0x94, 0x00, 0x00, 0x40, 0x33, 0x56, 0x39, 0x29, 0x0A
};
static const uint8_t PUCK_EPC_BLUE[EPC_LENGTH] = {
  0xE2, 0x80, 0x68, 0x94, 0x00, 0x00, 0x40, 0x33, 0x56, 0x38, 0xED, 0x0A
};
static const uint8_t PUCK_EPC_GREEN[EPC_LENGTH] = {
  0xE2, 0x80, 0x68, 0x94, 0x00, 0x00, 0x50, 0x33, 0x56, 0x39, 0x2D, 0x0A
};

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
};

// ─── Tag Storage ────────────────────────────────────────────────────────────

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

// ─── Setup ──────────────────────────────────────────────────────────────────

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

// ─── Main Loop ──────────────────────────────────────────────────────────────

void loop() {
  if (Serial.available()) {
    flushSerialInput();
    performScan();
  }
}

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
  printResult(rfidModule.setGen2Q(TMR_SR_GEN2_Q_DYNAMIC, 4, true));

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
  }

  // Ensure LEDs are off at start of scan
  for (int i = 0; i < NUM_PUCKS; i++) {
    digitalWrite(pucks[i].ledPin, LOW);
  }

  // Start continuous reading — module will stream tag records over UART
  rfidModule.startReading();
  readerRunning = true;

  uint32_t scanStart = millis();
  uint16_t totalReads = 0;
  uint16_t keepAlives = 0;
  uint16_t parseErrors = 0;

  // ── Collection window: poll for tags until time expires ──
  while (millis() - scanStart < SCAN_WINDOW_MS) {

    // check() reads bytes from UART and returns true when a complete
    // message frame is assembled in rfidModule.msg[]
    if (rfidModule.check()) {

      uint8_t responseType = rfidModule.parseResponse();

      switch (responseType) {
        case RESPONSE_IS_TAGFOUND:
          {
            totalReads++;

            // Extract EPC from the parsed message
            uint8_t tagDataBytes = rfidModule.getTagDataBytes();
            uint8_t epcBytes = rfidModule.getTagEPCBytes();
            int8_t rssi = rfidModule.getTagRSSI();

            // Sanity check — corrupt frames can report absurd lengths
            if (epcBytes == 0 || epcBytes > EPC_MAX_BYTES) {
              parseErrors++;
              break;
            }

            // EPC data starts at byte 31 + tagDataBytes offset in msg[]
            uint8_t epcStartIdx = 31 + tagDataBytes;

            // Check if this EPC is already in our inventory
            int existingIdx = findTagByEPC(&rfidModule.msg[epcStartIdx], epcBytes);

            if (existingIdx >= 0) {
              // Update existing tag's timestamp and RSSI
              tagInventory[existingIdx].lastSeenMs = millis();
              tagInventory[existingIdx].rssi = rssi;
            } else if (tagCount < MAX_TAGS) {
              // New unique tag — add to inventory
              DetectedTag *tag = &tagInventory[tagCount];
              memcpy(tag->epc, &rfidModule.msg[epcStartIdx], epcBytes);
              tag->epcLen = epcBytes;
              tag->rssi = rssi;
              tag->lastSeenMs = millis();
              tagCount++;

              // Print immediately for real-time feedback
              Serial.print(F("  [NEW] Tag #"));
              Serial.print(tagCount);
              Serial.print(F(" | RSSI: "));
              Serial.print(rssi);
              Serial.print(F(" dBm | EPC: "));
              printEPC(tag->epc, tag->epcLen);
            }
            break;
          }

        case RESPONSE_IS_KEEPALIVE:
          keepAlives++;
          break;

        case RESPONSE_IS_TEMPERATURE:
          break;

        case RESPONSE_IS_TEMPTHROTTLE:
          Serial.println(F("  WARNING: Module is thermal throttling!"));
          break;

        case ERROR_CORRUPT_RESPONSE:
          parseErrors++;
          break;

        default:
          break;
      }
    }

    // Yield to ESP32 watchdog
    // yield();
  }

  // ── Stop continuous reading ──
  rfidModule.stopReading();
  readerRunning = false;
  delay(100);

  // Drain any remaining bytes from the UART buffer
  while (Serial1.available()) { Serial1.read(); }

  // ── Match detected tags against known pucks ──
  int pucksFound = 0;
  for (int i = 0; i < tagCount; i++) {
    for (int p = 0; p < NUM_PUCKS; p++) {
      if (!pucks[p].detected && tagInventory[i].epcLen == EPC_LENGTH && memcmp(tagInventory[i].epc, pucks[p].epc, EPC_LENGTH) == 0) {
        pucks[p].detected = true;
        pucksFound++;
      }
    }
  }

  // ── Print Results ──
  Serial.println(F("\n────────────────────────────────────────"));
  Serial.print(F("SCAN COMPLETE: "));
  Serial.print(tagCount);
  Serial.print(F(" unique tag(s) found in "));
  Serial.print(SCAN_WINDOW_MS);
  Serial.println(F(" ms"));
  Serial.print(F("  Total tag reads: "));
  Serial.println(totalReads);
  Serial.print(F("  Keep-alives:     "));
  Serial.println(keepAlives);
  if (parseErrors > 0) {
    Serial.print(F("  Parse errors:    "));
    Serial.println(parseErrors);
  }
  Serial.print(F("  Known pucks:     "));
  Serial.print(pucksFound);
  Serial.print(F("/"));
  Serial.println(NUM_PUCKS);
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
    Serial.println(F("  → Check that pucks are within antenna range."));
    Serial.print(F("  → Try increasing READ_POWER (currently "));
    Serial.print(READ_POWER);
    Serial.println(F(")."));
  }

  // ── Light LEDs for matched pucks ──
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
