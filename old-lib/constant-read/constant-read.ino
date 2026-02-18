/*
  Reading multiple RFID tags, simultaneously!
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 3rd, 2016
  https://github.com/sparkfun/Simultaneous_RFID_Tag_Reader

  Constantly reads and outputs any tags heard

  If using the Simultaneous RFID Tag Reader (SRTR) shield, make sure the serial slide
  switch is in the 'SW-UART' position
*/
#include <HardwareSerial.h>
#include "SparkFun_UHF_RFID_Reader.h"

// Define which hardware UART to use for RFID (1 or 2, not 0)
#define RFID_UART_NUM 1

// Define the GPIO pins for RFID communication
// IMPORTANT: Verify these don't conflict with Ethernet SPI or other peripherals
// on your specific board revision!
#define RFID_RX_PIN 18  // ESP32-S3 RX <- RFID TXO (we receive data on this pin)
#define RFID_TX_PIN 17  // ESP32-S3 TX -> RFID RXI (we transmit data on this pin)

// Create the HardwareSerial instance for RFID
// On ESP32-S3, pass the UART number to the constructor
HardwareSerial rfidSerial(RFID_UART_NUM);

// Create instance of the RFID module
RFID rfidModule;

// Module configuration
#define RFID_BAUD 115200
#define MODULE_TYPE ThingMagic_M7E_HECTO

// LED pin - verify this is available on your board
#define GREEN_LED_PIN 2

// State tracking
boolean tagDetected = false;
unsigned long lastSeen = 0;
int counter = 0;

// ============================================================================
// RFID MODULE SETUP
// ============================================================================
boolean setupRfidModule(long baudRate) {
  rfidModule.begin(rfidSerial, MODULE_TYPE);

  // Initialize serial with explicit pin assignment
  // Signature: begin(baud, config, rxPin, txPin)
  rfidSerial.begin(baudRate, SERIAL_8N1, RFID_RX_PIN, RFID_TX_PIN);
  delay(100);

  // Clear any startup garbage from the module
  while (rfidSerial.available()) {
    rfidSerial.read();
  }

  rfidModule.getVersion();

  if (rfidModule.msg[0] == ERROR_WRONG_OPCODE_RESPONSE) {
    // Module is in continuous read mode - stop it
    rfidModule.stopReading();
    Serial.println(F("Module was continuously reading. Stopped."));
    delay(1500);
  } else {
    // Module may have just powered on at 115200, reconfigure if needed
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

  rfidModule.setTagProtocol();  // GEN2 protocol
  rfidModule.setAntennaPort();  // Antenna port 1

  return true;
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // Debug serial - uses UART0 / USB CDC automatically
  Serial.begin(115200);

  // Wait for serial connection (useful for native USB on ESP32-S3)
  // Add timeout to prevent hanging if no USB connected
  unsigned long startTime = millis();
  while (!Serial && (millis() - startTime < 3000)) {
    delay(10);
  }

  Serial.println(F("Waveshare ESP32S3-ETH RFID Reader"));
  Serial.println(F("=================================="));
  Serial.print(F("RFID UART: "));
  Serial.println(RFID_UART_NUM);
  Serial.print(F("RFID RX Pin (ESP32 receives): "));
  Serial.println(RFID_RX_PIN);
  Serial.print(F("RFID TX Pin (ESP32 transmits): "));
  Serial.println(RFID_TX_PIN);

  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, LOW);

  Serial.println(F("Initializing RFID module..."));

  if (!setupRfidModule(RFID_BAUD)) {
    Serial.println(F("ERROR: RFID module failed to respond!"));
    Serial.println(F("Check: wiring, power supply, baud rate"));
    while (1) {
      digitalWrite(GREEN_LED_PIN, !digitalRead(GREEN_LED_PIN));
      delay(200);  // Fast blink = error state
    }
  }

  rfidModule.setRegion(REGION_NORTHAMERICA);
  rfidModule.setReadPower(500);  // Low power for range testing
  rfidModule.startReading();

  Serial.println(F("RFID reader initialized. Scanning..."));
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  if (rfidModule.check()) {
    byte responseType = rfidModule.parseResponse();

    if (responseType == RESPONSE_IS_TAGFOUND) {
      Serial.print(F("Tag detected: "));
      Serial.println(counter++);

      if (!tagDetected) {
        tagDetected = true;
        digitalWrite(GREEN_LED_PIN, HIGH);
      } else if (millis() - lastSeen > 250) {
        digitalWrite(GREEN_LED_PIN, HIGH);
      }
      lastSeen = millis();
    }
  }

  if (tagDetected && (millis() - lastSeen) > 1000) {
    Serial.println(F("Tag lost."));
    tagDetected = false;
    digitalWrite(GREEN_LED_PIN, LOW);
  }

  // Pause scanning on serial input
  if (Serial.available()) {
    rfidModule.stopReading();
    while (Serial.available()) Serial.read();

    Serial.println(F("Scanning paused. Press any key to resume."));
    while (!Serial.available()) delay(10);
    while (Serial.available()) Serial.read();

    rfidModule.startReading();
    Serial.println(F("Scanning resumed."));
  }
}
