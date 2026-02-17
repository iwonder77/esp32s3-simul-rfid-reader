/*
  Reading RFID Tag Unique ID (TID)
  Modified for Waveshare ESP32S3-ETH
  
  Original by: Nathan Seidle @ SparkFun Electronics
  https://github.com/sparkfun/Simultaneous_RFID_Tag_Reader

  Every tag has a Tag ID (the TID) that is not editable. This memory
  contains the chip vendor ID and model ID for the tag (24 bits). All tags
  must have these bits. Almost all tags also include a UID or unique ID
  in the TID memory area. 

  UIDs can range in length. We've seen 8 bytes (64 bit) most commonly but some
  UIDs are 20 bytes (160 bits).

  UIDs can range from 0 to 20 bytes. If the sketch tries to read a UID
  that is longer than a tag actually has then the read will fail. Start with 8 
  and increase it if you have a tag with a longer UID.
*/

#include <HardwareSerial.h>
#include "SparkFun_UHF_RFID_Reader.h"

// ============================================================================
// UART CONFIGURATION FOR WAVESHARE ESP32S3-ETH
// ============================================================================
//
// UART0 (GPIO 43/44) is reserved for USB-Serial debug output.
// We use UART1 with custom pins for the RFID module.
//
// Wiring:
//   ESP32S3-ETH     RFID Module
//   -----------     -----------
//   VBUS (5V)   --> VCC
//   GND         --> GND
//   GPIO 17     --> RXI  (ESP32 transmits, RFID receives)
//   GPIO 18     --> TXO  (RFID transmits, ESP32 receives)

#define RFID_UART_NUM 1  // Use hardware UART1
#define RFID_RX_PIN 18   // ESP32 RX <- RFID TXO
#define RFID_TX_PIN 17   // ESP32 TX -> RFID RXI

// Create HardwareSerial instance for RFID on UART1
HardwareSerial rfidSerial(RFID_UART_NUM);

// Create instance of the RFID module
RFID rfidModule;

// Module settings
#define RFID_BAUD 115200
#define MODULE_TYPE ThingMagic_M7E_HECTO

// ============================================================================
// RFID MODULE INITIALIZATION
// ============================================================================
boolean setupRfidModule(long baudRate) {
  rfidModule.begin(rfidSerial, MODULE_TYPE);

  // Clear any startup data from the module
  while (rfidSerial.available()) {
    rfidSerial.read();
  }

  rfidModule.getVersion();

  if (rfidModule.msg[0] == ERROR_WRONG_OPCODE_RESPONSE) {
    // Module is already in continuous read mode - stop it
    rfidModule.stopReading();
    Serial.println(F("Module was continuously reading. Stopped."));
    delay(1500);
  } else {
    // Module may have just powered on at default 115200
    // Reconfigure to desired baud rate
    rfidSerial.begin(115200, SERIAL_8N1, RFID_RX_PIN, RFID_TX_PIN);
    rfidModule.setBaud(baudRate);
    rfidSerial.begin(baudRate, SERIAL_8N1, RFID_RX_PIN, RFID_TX_PIN);
    delay(250);
  }

  // Verify communication
  rfidModule.getVersion();
  if (rfidModule.msg[0] != ALL_GOOD) {
    return false;
  }

  rfidModule.setTagProtocol();  // Set protocol to GEN2
  rfidModule.setAntennaPort();  // Set TX/RX antenna ports to 1

  return true;
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // Debug serial (uses USB CDC on ESP32-S3)
  Serial.begin(115200);

  // Wait for USB serial with timeout (prevents hanging if no USB connected)
  unsigned long startTime = millis();
  while (!Serial && (millis() - startTime < 3000)) {
    delay(10);
  }

  Serial.println(F("==========================================="));
  Serial.println(F("  RFID Tag UID Reader - ESP32S3-ETH"));
  Serial.println(F("==========================================="));
  Serial.print(F("RFID UART: "));
  Serial.println(RFID_UART_NUM);
  Serial.print(F("RX Pin (from RFID TXO): GPIO "));
  Serial.println(RFID_RX_PIN);
  Serial.print(F("TX Pin (to RFID RXI):   GPIO "));
  Serial.println(RFID_TX_PIN);
  Serial.println();

  // Initialize RFID serial with explicit pin assignment
  rfidSerial.begin(RFID_BAUD, SERIAL_8N1, RFID_RX_PIN, RFID_TX_PIN);
  delay(100);

  Serial.println(F("Initializing RFID module..."));

  if (!setupRfidModule(RFID_BAUD)) {
    Serial.println(F("ERROR: Module failed to respond!"));
    Serial.println(F("Check: wiring, power (VBUS->VCC, GND->GND)"));
    while (1) {
      delay(1000);  // Halt
    }
  }

  rfidModule.setRegion(REGION_NORTHAMERICA);
  rfidModule.setReadPower(500);  // 5.00 dBm - safe for USB power

  rfidModule.enableDebugging();  // Show commands sent/received

  Serial.println(F("RFID module initialized successfully."));
  Serial.println();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  Serial.println(F("Place a tag near the reader, then press any key to read its UID."));

  while (!Serial.available()) {
    delay(10);  // Wait for user input
  }
  Serial.read();  // Discard the character

  byte response;
  byte myUID[8];  // UIDs typically 8 bytes; increase if needed (max 20)
  byte uidLength = sizeof(myUID);

  Serial.println(F("Reading tag UID..."));

  response = rfidModule.readTID(myUID, uidLength);

  if (response == RESPONSE_SUCCESS) {
    Serial.print(F("UID: ["));
    for (byte x = 0; x < uidLength; x++) {
      if (myUID[x] < 0x10) Serial.print("0");
      Serial.print(myUID[x], HEX);
      if (x < uidLength - 1) Serial.print(" ");
    }
    Serial.println(F("]"));
  } else {
    Serial.println(F("Read failed. Ensure tag is within range."));
  }

  Serial.println();
}
