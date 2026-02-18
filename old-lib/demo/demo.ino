/*
  RFID Library Limitation Demo
  For Waveshare ESP32S3-ETH + SparkFun Simultaneous RFID Reader (M7E Hecto)
  
  PURPOSE:
  Demonstrates the limitations of the SparkFun UHF RFID library for
  multi-tag inventory scenarios. This demo is intended to show stakeholders
  why we need to extend the library with Gen2 session/singulation support.
  
  KEY LIMITATIONS DEMONSTRATED:
  1. Only one tag (strongest signal) is reliably read when multiple tags present
  2. No session-based singulation - same tag read repeatedly
  3. No true inventory capability - can't systematically enumerate all tags
  4. Library lacks Select commands for tag filtering
  
  HARDWARE:
  - Waveshare ESP32S3-ETH
  - SparkFun Simultaneous RFID Reader (M7E Hecto)
  - 3x LEDs (Yellow, Red, Green) for visual feedback
  
  MOLECULE PUCK UIDs:
  - Puck 1 (Yellow): 20 00 50 33 56 38 E9 0A
  - Puck 2 (Red):    20 00 40 33 56 38 ED 0A  
  - Puck 3 (Green):  20 00 50 33 56 38 35 0A
  
  CONTROL:
  - Serial commands only (1, 2, 3, ?)
*/

#include <HardwareSerial.h>
#include "SparkFun_UHF_RFID_Reader.h"

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// UART for RFID module
#define RFID_UART_NUM 1
#define RFID_RX_PIN 18
#define RFID_TX_PIN 17
#define RFID_BAUD 115200
#define MODULE_TYPE ThingMagic_M7E_HECTO

// LEDs - one per molecule puck
#define LED_PUCK_1 1  // Yellow LED - Molecule 1
#define LED_PUCK_2 2  // Red LED - Molecule 2
#define LED_PUCK_3 3  // Green LED - Molecule 3

// ============================================================================
// KNOWN TAG UIDs (Your molecule pucks)
// ============================================================================

const uint8_t PUCK_1_UID[] = { 0x20, 0x00, 0x50, 0x33, 0x56, 0x38, 0xE9, 0x0A };
const uint8_t PUCK_2_UID[] = { 0x20, 0x00, 0x40, 0x33, 0x56, 0x38, 0xED, 0x0A };
const uint8_t PUCK_3_UID[] = { 0x20, 0x00, 0x50, 0x33, 0x56, 0x39, 0x35, 0x0A };
const uint8_t UID_LENGTH = 8;

const char* PUCK_NAMES[] = { "Molecule-1 (Yellow)", "Molecule-2 (Red)", "Molecule-3 (Green)" };
const uint8_t LED_PINS[] = { LED_PUCK_1, LED_PUCK_2, LED_PUCK_3 };

// ============================================================================
// STATISTICS TRACKING
// ============================================================================

struct TagStats {
  uint32_t readCount;
  int8_t lastRSSI;
  uint32_t lastSeenTime;
  bool detectedThisRound;
};

TagStats puckStats[3] = { { 0, 0, 0, false }, { 0, 0, 0, false }, { 0, 0, 0, false } };
uint32_t unknownTagCount = 0;
uint32_t totalReadAttempts = 0;

// ============================================================================
// DEMO STATE
// ============================================================================

enum DemoMode {
  MODE_IDLE,
  MODE_SINGLE_READ,
  MODE_CONTINUOUS_INVENTORY,
  MODE_TIMED_INVENTORY
};

DemoMode currentMode = MODE_IDLE;
uint32_t inventoryStartTime = 0;
const uint32_t INVENTORY_DURATION_MS = 5000;

// ============================================================================
// RFID OBJECTS
// ============================================================================

HardwareSerial rfidSerial(RFID_UART_NUM);
RFID rfidModule;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

bool compareUID(const uint8_t* uid1, const uint8_t* uid2, uint8_t length) {
  for (uint8_t i = 0; i < length; i++) {
    if (uid1[i] != uid2[i]) return false;
  }
  return true;
}

int8_t identifyPuck(const uint8_t* uid, uint8_t length) {
  if (length != UID_LENGTH) return -1;

  if (compareUID(uid, PUCK_1_UID, UID_LENGTH)) return 0;
  if (compareUID(uid, PUCK_2_UID, UID_LENGTH)) return 1;
  if (compareUID(uid, PUCK_3_UID, UID_LENGTH)) return 2;

  return -1;
}

void printUID(const uint8_t* uid, uint8_t length) {
  Serial.print("[");
  for (uint8_t i = 0; i < length; i++) {
    if (uid[i] < 0x10) Serial.print("0");
    Serial.print(uid[i], HEX);
    if (i < length - 1) Serial.print(" ");
  }
  Serial.print("]");
}

void setAllLEDs(bool state) {
  digitalWrite(LED_PUCK_1, state);
  digitalWrite(LED_PUCK_2, state);
  digitalWrite(LED_PUCK_3, state);
}

void flashLED(uint8_t pin, uint16_t duration = 100) {
  digitalWrite(pin, HIGH);
  delay(duration);
  digitalWrite(pin, LOW);
}

void resetStats() {
  for (int i = 0; i < 3; i++) {
    puckStats[i].readCount = 0;
    puckStats[i].lastRSSI = 0;
    puckStats[i].lastSeenTime = 0;
    puckStats[i].detectedThisRound = false;
  }
  unknownTagCount = 0;
  totalReadAttempts = 0;
}

void printStats() {
  Serial.println(F("\n╔════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║                    INVENTORY RESULTS                       ║"));
  Serial.println(F("╠════════════════════════════════════════════════════════════╣"));

  uint8_t detectedCount = 0;

  for (int i = 0; i < 3; i++) {
    Serial.print(F("║  "));
    Serial.print(PUCK_NAMES[i]);

    for (int p = strlen(PUCK_NAMES[i]); p < 22; p++) Serial.print(" ");

    if (puckStats[i].readCount > 0) {
      detectedCount++;
      Serial.print(F("│ DETECTED │ Reads: "));
      if (puckStats[i].readCount < 10) Serial.print(" ");
      if (puckStats[i].readCount < 100) Serial.print(" ");
      Serial.print(puckStats[i].readCount);
      Serial.print(F(" │ RSSI: "));
      Serial.print(puckStats[i].lastRSSI);
      Serial.println(F(" dBm ║"));
    } else {
      Serial.println(F("│   MISSED  │                          ║"));
    }
  }

  Serial.println(F("╠════════════════════════════════════════════════════════════╣"));
  Serial.print(F("║  Tags Detected: "));
  Serial.print(detectedCount);
  Serial.print(F("/3"));

  if (totalReadAttempts > 0) {
    Serial.print(F("    │    Total Read Events: "));
    if (totalReadAttempts < 10) Serial.print(" ");
    if (totalReadAttempts < 100) Serial.print(" ");
    Serial.print(totalReadAttempts);
    Serial.println(F("     ║"));
  } else {
    Serial.println(F("                                        ║"));
  }

  if (unknownTagCount > 0) {
    Serial.print(F("║  Unknown tags detected: "));
    Serial.print(unknownTagCount);
    Serial.println(F("                                  ║"));
  }

  Serial.println(F("╚════════════════════════════════════════════════════════════╝"));

  if (detectedCount < 3 && totalReadAttempts > 0) {
    Serial.println();
    Serial.println(F("⚠️  LIMITATION DEMONSTRATED:"));
    Serial.println(F("    Even with multiple tags present, only some were detected."));
    Serial.println(F("    The library lacks Gen2 session/singulation support to"));
    Serial.println(F("    systematically inventory all tags in the field."));
  }

  if (totalReadAttempts > 10 && detectedCount == 1) {
    Serial.println();
    Serial.println(F("⚠️  SINGLE TAG DOMINANCE:"));
    Serial.println(F("    One tag dominated all reads. Without session management,"));
    Serial.println(F("    the strongest tag responds every time, blocking others."));
  }
}

// ============================================================================
// RFID MODULE SETUP
// ============================================================================

boolean setupRfidModule(long baudRate) {
  rfidModule.begin(rfidSerial, MODULE_TYPE);

  rfidSerial.begin(baudRate, SERIAL_8N1, RFID_RX_PIN, RFID_TX_PIN);
  delay(100);

  while (rfidSerial.available()) {
    rfidSerial.read();
  }

  rfidModule.getVersion();

  if (rfidModule.msg[0] == ERROR_WRONG_OPCODE_RESPONSE) {
    rfidModule.stopReading();
    Serial.println(F("Module was in continuous read. Stopped."));
    delay(1500);
  } else {
    rfidSerial.begin(115200, SERIAL_8N1, RFID_RX_PIN, RFID_TX_PIN);
    rfidModule.setBaud(baudRate);
    rfidSerial.begin(baudRate, SERIAL_8N1, RFID_RX_PIN, RFID_TX_PIN);
    delay(250);
  }

  rfidModule.getVersion();
  if (rfidModule.msg[0] != ALL_GOOD) {
    return false;
  }

  rfidModule.setTagProtocol();
  rfidModule.setAntennaPort();

  return true;
}

// ============================================================================
// DEMO MODE HANDLERS
// ============================================================================

void runSingleRead() {
  Serial.println(F("\n▶ SINGLE READ MODE"));
  Serial.println(F("  Attempting to read one tag..."));

  resetStats();
  setAllLEDs(LOW);

  uint8_t uid[UID_LENGTH];
  uint8_t uidLength = UID_LENGTH;

  uint8_t response = rfidModule.readTID(uid, uidLength);

  if (response == RESPONSE_SUCCESS) {
    totalReadAttempts++;

    Serial.print(F("  Tag found: "));
    printUID(uid, uidLength);

    int8_t puckIndex = identifyPuck(uid, uidLength);

    if (puckIndex >= 0) {
      Serial.print(F(" → "));
      Serial.println(PUCK_NAMES[puckIndex]);

      puckStats[puckIndex].readCount++;
      puckStats[puckIndex].detectedThisRound = true;

      digitalWrite(LED_PINS[puckIndex], HIGH);

    } else {
      Serial.println(F(" → UNKNOWN TAG"));
      unknownTagCount++;
    }
  } else {
    Serial.println(F("  No tag detected."));
  }

  printStats();

  delay(2000);
  setAllLEDs(LOW);

  currentMode = MODE_IDLE;
}

void runTimedInventory() {
  Serial.println(F("\n▶ TIMED INVENTORY MODE"));
  Serial.print(F("  Running for "));
  Serial.print(INVENTORY_DURATION_MS / 1000);
  Serial.println(F(" seconds..."));
  Serial.println(F("  Place ALL THREE pucks on the reader now."));
  Serial.println();

  resetStats();
  setAllLEDs(LOW);

  rfidModule.startReading();

  inventoryStartTime = millis();
  uint32_t lastPrintTime = 0;

  while (millis() - inventoryStartTime < INVENTORY_DURATION_MS) {

    if (rfidModule.check()) {
      uint8_t responseType = rfidModule.parseResponse();

      if (responseType == RESPONSE_IS_TAGFOUND) {
        totalReadAttempts++;
      }
    }

    if (millis() - lastPrintTime > 500) {
      lastPrintTime = millis();

      rfidModule.stopReading();
      delay(50);

      uint8_t uid[UID_LENGTH];
      uint8_t uidLength = UID_LENGTH;

      if (rfidModule.readTID(uid, uidLength) == RESPONSE_SUCCESS) {
        int8_t puckIndex = identifyPuck(uid, uidLength);

        if (puckIndex >= 0) {
          puckStats[puckIndex].readCount++;
          puckStats[puckIndex].lastRSSI = rfidModule.getTagRSSI();
          puckStats[puckIndex].lastSeenTime = millis();
          puckStats[puckIndex].detectedThisRound = true;

          digitalWrite(LED_PINS[puckIndex], HIGH);

          Serial.print(puckIndex + 1);
        } else {
          unknownTagCount++;
          Serial.print("?");
        }
      } else {
        Serial.print(".");
      }

      rfidModule.startReading();
    }

    delay(10);
  }

  rfidModule.stopReading();

  Serial.println();
  printStats();

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PINS[i], puckStats[i].readCount > 0 ? HIGH : LOW);
  }

  delay(3000);
  setAllLEDs(LOW);

  currentMode = MODE_IDLE;
}

void runRapidFireDemo() {
  Serial.println(F("\n▶ RAPID-FIRE READ MODE"));
  Serial.println(F("  Performing 20 consecutive single reads..."));
  Serial.println(F("  This demonstrates single-tag dominance.\n"));

  resetStats();
  setAllLEDs(LOW);

  const int NUM_READS = 20;

  for (int i = 0; i < NUM_READS; i++) {
    uint8_t uid[UID_LENGTH];
    uint8_t uidLength = UID_LENGTH;

    if (rfidModule.readTID(uid, uidLength) == RESPONSE_SUCCESS) {
      totalReadAttempts++;

      int8_t puckIndex = identifyPuck(uid, uidLength);

      Serial.print(F("  Read "));
      if (i + 1 < 10) Serial.print(" ");
      Serial.print(i + 1);
      Serial.print(F(": "));

      if (puckIndex >= 0) {
        puckStats[puckIndex].readCount++;
        puckStats[puckIndex].detectedThisRound = true;

        Serial.print(PUCK_NAMES[puckIndex]);

        flashLED(LED_PINS[puckIndex], 50);

      } else {
        unknownTagCount++;
        Serial.print(F("Unknown tag"));
      }
      Serial.println();

    } else {
      Serial.print(F("  Read "));
      if (i + 1 < 10) Serial.print(" ");
      Serial.print(i + 1);
      Serial.println(F(": [no response]"));
    }

    delay(100);
  }

  printStats();

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PINS[i], puckStats[i].readCount > 0 ? HIGH : LOW);
  }

  delay(3000);
  setAllLEDs(LOW);

  currentMode = MODE_IDLE;
}

// ============================================================================
// COMMAND PARSER
// ============================================================================

void printMenu() {
  Serial.println(F("\n┌────────────────────────────────────────────────────────────┐"));
  Serial.println(F("│           RFID LIBRARY LIMITATION DEMO                     │"));
  Serial.println(F("├────────────────────────────────────────────────────────────┤"));
  Serial.println(F("│  Commands (send via Serial Monitor):                       │"));
  Serial.println(F("│    1 = Single read                                         │"));
  Serial.println(F("│    2 = Timed inventory (5 seconds)                         │"));
  Serial.println(F("│    3 = Rapid-fire reads (20 consecutive)                   │"));
  Serial.println(F("│    ? = Show this menu                                      │"));
  Serial.println(F("│                                                            │"));
  Serial.println(F("│  Known Tags:                                               │"));
  Serial.print(F("│    Puck 1 (Yellow): "));
  printUID(PUCK_1_UID, UID_LENGTH);
  Serial.println(F("              │"));
  Serial.print(F("│    Puck 2 (Red):    "));
  printUID(PUCK_2_UID, UID_LENGTH);
  Serial.println(F("              │"));
  Serial.print(F("│    Puck 3 (Green):  "));
  printUID(PUCK_3_UID, UID_LENGTH);
  Serial.println(F("              │"));
  Serial.println(F("│                                                            │"));
  Serial.println(F("│  LEDs:                                                     │"));
  Serial.println(F("│    GPIO 1 = Yellow (Puck 1)                                │"));
  Serial.println(F("│    GPIO 2 = Red    (Puck 2)                                │"));
  Serial.println(F("│    GPIO 3 = Green  (Puck 3)                                │"));
  Serial.println(F("└────────────────────────────────────────────────────────────┘"));
}

void processSerialCommand() {
  if (Serial.available()) {
    char cmd = Serial.read();

    while (Serial.available()) Serial.read();

    switch (cmd) {
      case '1':
        currentMode = MODE_SINGLE_READ;
        break;

      case '2':
        currentMode = MODE_TIMED_INVENTORY;
        break;

      case '3':
        currentMode = MODE_CONTINUOUS_INVENTORY;
        break;

      case '?':
      case 'h':
      case 'H':
        printMenu();
        break;

      default:
        break;
    }
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);

  unsigned long startTime = millis();
  while (!Serial && (millis() - startTime < 3000)) {
    delay(10);
  }

  // Initialize LEDs
  pinMode(LED_PUCK_1, OUTPUT);
  pinMode(LED_PUCK_2, OUTPUT);
  pinMode(LED_PUCK_3, OUTPUT);

  // LED test sequence
  Serial.println(F("\nLED Test..."));
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PINS[i], HIGH);
    delay(200);
    digitalWrite(LED_PINS[i], LOW);
  }

  // Initialize RFID serial
  rfidSerial.begin(RFID_BAUD, SERIAL_8N1, RFID_RX_PIN, RFID_TX_PIN);
  delay(100);

  Serial.println(F("\nInitializing RFID module..."));

  if (!setupRfidModule(RFID_BAUD)) {
    Serial.println(F("ERROR: RFID module failed to respond!"));
    Serial.println(F("Check wiring: VBUS→VCC, GND→GND, GPIO17→RXI, GPIO18→TXO"));

    // Error indication: all LEDs blink
    while (1) {
      setAllLEDs(HIGH);
      delay(200);
      setAllLEDs(LOW);
      delay(200);
    }
  }

  rfidModule.setRegion(REGION_NORTHAMERICA);
  rfidModule.setReadPower(1000);  // 10.00 dBm

  Serial.println(F("RFID module ready."));

  // Startup animation
  for (int j = 0; j < 2; j++) {
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_PINS[i], HIGH);
      delay(100);
    }
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_PINS[i], LOW);
      delay(100);
    }
  }

  printMenu();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  processSerialCommand();

  switch (currentMode) {
    case MODE_SINGLE_READ:
      runSingleRead();
      break;

    case MODE_TIMED_INVENTORY:
      runTimedInventory();
      break;

    case MODE_CONTINUOUS_INVENTORY:
      runRapidFireDemo();
      break;

    case MODE_IDLE:
    default:
      // Idle - do nothing, wait for serial command
      delay(10);
      break;
  }
}
