/*
  Example: Gen2 Parameter Control for Multi-Tag Inventory
  Based on SparkFun Example31, simplified for ESP32-S3 + M7E Hecto
  
  Hardware: ESP32-S3 → M7E Hecto via UART
    - ESP32 RX (GPIO18) → M7E TX
    - ESP32 TX (GPIO17) → M7E RX
  
  Key Gen2 Parameters for simultaneous tag detection:
    - Session (S0-S3): Controls tag state persistence. S1/S2 recommended for multi-tag.
    - Target (A/B/AB/BA): Controls which tag states respond. AB/BA enables round-robin.
    - Q: Slot count for anti-collision. Dynamic recommended for varying tag counts.
    - RFMode: Link frequency, encoding, and timing configuration.
*/

#include "SparkFun_UHF_RFID_Reader.h"

// === Hardware Configuration ===
#define RXD1 18  // ESP32-S3 RX ← M7E TX
#define TXD1 17  // ESP32-S3 TX → M7E RX
#define RFID_BAUD 115200
#define RFID_REGION REGION_NORTHAMERICA
#define DEBUG_LEVEL 0  // 0: off, 1: show UART traffic

RFID rfidModule;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("ESP32-S3 + M7E Hecto: Gen2 Parameter Test"));

  if (DEBUG_LEVEL) rfidModule.enableDebugging(Serial);

  // Initialize Serial1 with custom pins
  Serial1.begin(RFID_BAUD, SERIAL_8N1, RXD1, TXD1);
  
  if (!setupRfidModule()) {
    Serial.println(F("ERROR: Module not responding. Check wiring."));
    while (1);
  }

  rfidModule.setRegion(RFID_REGION);
  rfidModule.setReadPower(500);  // 5.00 dBm - low power for near-field

  // Configure Gen2 for multi-tag inventory
  setSession(TMR_GEN2_SESSION_S1);        // Persistent tag state (~500ms-5s)
  setTarget(TMR_GEN2_TARGET_AB);          // Inventory A, then B (round-robin)
  setRFMode(TMR_GEN2_RFMODE_250_M4_20);   // 250KHz BLF, Miller-4, 20µs Tari

  printMenu();
  Serial.println(F("\nPress any key to start scanning..."));
  flushSerial();
  while (!Serial.available());
  flushSerial();
}

void loop() {
  byte epc[12];
  byte epcLength = sizeof(epc);
  
  Serial.println(F("Scanning..."));
  uint8_t response = rfidModule.readTagEPC(epc, epcLength, 500);
  
  if (response == RESPONSE_SUCCESS) {
    Serial.print(F("EPC: "));
    for (byte i = 0; i < epcLength; i++) {
      if (epc[i] < 0x10) Serial.print(F("0"));
      Serial.print(epc[i], HEX);
      Serial.print(F(" "));
    }
    Serial.println();
  }
  
  handleSerialInput();
}

// === Gen2 Parameter Setters ===

void setSession(TMR_GEN2_Session session) {
  if (!rfidModule.setGen2Session(session)) {
    Serial.println(F("ERROR: Failed to set Session"));
  }
}

void setTarget(TMR_GEN2_Target target) {
  if (!rfidModule.setGen2Target(target)) {
    Serial.println(F("ERROR: Failed to set Target"));
  }
}

void setRFMode(TMR_GEN2_RFMode mode) {
  if (!rfidModule.setGen2RFmode(mode)) {
    Serial.println(F("ERROR: Failed to set RFMode"));
  }
}

void setQ(TMR_SR_GEN2_QType qType, uint8_t initQ = 4, bool setInit = false) {
  if (!rfidModule.setGen2Q(qType, initQ, setInit)) {
    Serial.println(F("ERROR: Failed to set Q"));
  }
}

// === Serial Input Handler ===

void handleSerialInput() {
  if (!Serial.available()) return;
  
  char c1 = Serial.read();
  delay(50);
  char c2 = Serial.available() ? Serial.read() : 0;
  flushSerial();

  // Target: a, b, ab, ba
  if (c1 == 'a' && c2 == 'b')      { Serial.println(F("Target → AB")); setTarget(TMR_GEN2_TARGET_AB); }
  else if (c1 == 'a')              { Serial.println(F("Target → A"));  setTarget(TMR_GEN2_TARGET_A); }
  else if (c1 == 'b' && c2 == 'a') { Serial.println(F("Target → BA")); setTarget(TMR_GEN2_TARGET_BA); }
  else if (c1 == 'b')              { Serial.println(F("Target → B"));  setTarget(TMR_GEN2_TARGET_B); }
  
  // Session: s0, s1, s2, s3
  else if (c1 == 's') {
    switch (c2) {
      case '0': Serial.println(F("Session → S0")); setSession(TMR_GEN2_SESSION_S0); break;
      case '1': Serial.println(F("Session → S1")); setSession(TMR_GEN2_SESSION_S1); break;
      case '2': Serial.println(F("Session → S2")); setSession(TMR_GEN2_SESSION_S2); break;
      case '3': Serial.println(F("Session → S3")); setSession(TMR_GEN2_SESSION_S3); break;
      default:  Serial.println(F("Invalid session (s0-s3)")); break;
    }
  }
  
  // Q: qd (dynamic), qs (static)
  else if (c1 == 'q') {
    if (c2 == 'd')      { Serial.println(F("Q → Dynamic")); setQ(TMR_SR_GEN2_Q_DYNAMIC); }
    else if (c2 == 's') { Serial.println(F("Q → Static"));  setQ(TMR_SR_GEN2_Q_STATIC); }
    else                { Serial.println(F("Invalid Q (qd/qs)")); }
  }
  
  // RFMode: r0-r7
  else if (c1 == 'r') {
    TMR_GEN2_RFMode modes[] = {
      TMR_GEN2_RFMODE_160_M8_20,   // r0
      TMR_GEN2_RFMODE_250_M4_20,   // r1
      TMR_GEN2_RFMODE_320_M2_15,   // r2
      TMR_GEN2_RFMODE_320_M2_20,   // r3
      TMR_GEN2_RFMODE_320_M4_20,   // r4
      TMR_GEN2_RFMODE_640_FM0_7_5, // r5
      TMR_GEN2_RFMODE_640_M2_7_5,  // r6
      TMR_GEN2_RFMODE_640_M4_7_5   // r7
    };
    int idx = c2 - '0';
    if (idx >= 0 && idx <= 7) {
      Serial.print(F("RFMode → ")); Serial.println(idx);
      setRFMode(modes[idx]);
    } else {
      Serial.println(F("Invalid RFMode (r0-r7)"));
    }
  }
  
  else if (c1 == '?') { printMenu(); }
  
  else { Serial.println(F("Unknown command. Press ? for help.")); }
}

void printMenu() {
  Serial.println(F("\n=== Gen2 Parameter Commands ==="));
  Serial.println(F("  Target:  a / b / ab / ba"));
  Serial.println(F("  Session: s0 / s1 / s2 / s3"));
  Serial.println(F("  Q:       qd (dynamic) / qs (static)"));
  Serial.println(F("  RFMode:  r0-r7"));
  Serial.println(F("  Help:    ?"));
}

void flushSerial() {
  delay(50);
  while (Serial.available()) { Serial.read(); delay(10); }
}

// === Module Initialization ===

bool setupRfidModule() {
  rfidModule.begin(Serial1, ThingMagic_M7E_HECTO);
  delay(200);
  
  // Clear any startup messages
  while (Serial1.available()) Serial1.read();
  
  rfidModule.getVersion();
  
  if (rfidModule.msg[0] == ERROR_WRONG_OPCODE_RESPONSE) {
    // Module is mid-read, stop it
    Serial.println(F("Module was reading, stopping..."));
    rfidModule.stopReading();
    delay(1500);
  }
  else if (rfidModule.msg[0] != ALL_GOOD) {
    // Try default baud rate
    Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
    rfidModule.setBaud(RFID_BAUD);
    Serial1.begin(RFID_BAUD, SERIAL_8N1, RXD1, TXD1);
    rfidModule.getVersion();
    if (rfidModule.msg[0] != ALL_GOOD) return false;
  }
  
  rfidModule.setTagProtocol();  // GEN2
  rfidModule.setAntennaPort();  // Antenna port 1
  return true;
}
