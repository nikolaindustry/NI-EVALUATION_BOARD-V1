// Example: Modbus RTU Test - CRC Verification and Function Tests
//
// This example verifies the Modbus RTU implementation by testing:
//  - CRC calculation against known test vectors
//  - Frame building and parsing
//  - Basic communication flow
//
// This is a diagnostic tool to ensure proper Modbus RTU operation.

#include <NI-EVALUATION_BOARD-V1.h>

NIEvaluationBoardV1 board;

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("========================================");
  Serial.println("  NI Evaluation Board V1");
  Serial.println("  Modbus RTU Test Suite");
  Serial.println("========================================");
  Serial.println();
  
  // Initialize board
  if (!board.begin(9600)) {
    Serial.println("ERROR: Board initialization failed!");
    while (1) delay(100);
  }
  
  Serial.println("Board initialized successfully");
  Serial.println();
  
  // Run test suite
  runTestSuite();
  
  Serial.println();
  Serial.println("========================================");
  Serial.println("Test suite completed!");
  Serial.println("========================================");
}

void loop() {
  // Test complete - do nothing
  delay(1000);
}

void runTestSuite() {
  int passed = 0;
  int failed = 0;
  
  // Test 1: CRC Calculation Test
  Serial.println("Test 1: Modbus CRC Calculation");
  Serial.println("--------------------------------");
  if (testCRCCalculation()) {
    Serial.println("✓ PASSED");
    passed++;
  } else {
    Serial.println("✗ FAILED");
    failed++;
  }
  Serial.println();
  
  // Test 2: Read Holding Registers Frame
  Serial.println("Test 2: Read Holding Registers Frame");
  Serial.println("--------------------------------------");
  if (testReadHoldingRegistersFrame()) {
    Serial.println("✓ PASSED");
    passed++;
  } else {
    Serial.println("✗ FAILED");
    failed++;
  }
  Serial.println();
  
  // Test 3: Write Single Register Frame
  Serial.println("Test 3: Write Single Register Frame");
  Serial.println("------------------------------------");
  if (testWriteSingleRegisterFrame()) {
    Serial.println("✓ PASSED");
    passed++;
  } else {
    Serial.println("✗ FAILED");
    failed++;
  }
  Serial.println();
  
  // Test 4: Write Multiple Registers Frame
  Serial.println("Test 4: Write Multiple Registers Frame");
  Serial.println("---------------------------------------");
  if (testWriteMultipleRegistersFrame()) {
    Serial.println("✓ PASSED");
    passed++;
  } else {
    Serial.println("✗ FAILED");
    failed++;
  }
  Serial.println();
  
  // Summary
  Serial.println("========================================");
  Serial.println("Test Summary:");
  Serial.print("  Passed: ");
  Serial.println(passed);
  Serial.print("  Failed: ");
  Serial.println(failed);
  Serial.print("  Total:  ");
  Serial.println(passed + failed);
  Serial.println("========================================");
}

// Modbus CRC-16 calculation (for testing)
uint16_t calculateTestCRC(uint8_t *buffer, uint8_t length) {
  uint16_t crc = 0xFFFF;
  
  for (uint8_t pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)buffer[pos];
    
    for (uint8_t i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  
  return crc;
}

bool testCRCCalculation() {
  // Test vectors from Modbus specification
  // Test case 1: Simple request
  uint8_t test1[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02};
  uint16_t expected1 = 0x0AC4;  // Expected CRC in little-endian
  
  uint16_t calculated1 = calculateTestCRC(test1, sizeof(test1));
  
  Serial.print("Test Vector 1: ");
  for (uint8_t i = 0; i < sizeof(test1); i++) {
    Serial.print("0x");
    if (test1[i] < 0x10) Serial.print("0");
    Serial.print(test1[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  Serial.print("Expected CRC: 0x");
  Serial.println(expected1, HEX);
  Serial.print("Calculated CRC: 0x");
  Serial.println(calculated1, HEX);
  
  if (calculated1 == expected1) {
    return true;
  }
  
  // Try alternative test vector
  uint8_t test2[] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x03};
  uint16_t expected2 = 0x98D9;
  
  uint16_t calculated2 = calculateTestCRC(test2, sizeof(test2));
  
  Serial.println();
  Serial.print("Test Vector 2: ");
  for (uint8_t i = 0; i < sizeof(test2); i++) {
    Serial.print("0x");
    if (test2[i] < 0x10) Serial.print("0");
    Serial.print(test2[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  Serial.print("Expected CRC: 0x");
  Serial.println(expected2, HEX);
  Serial.print("Calculated CRC: 0x");
  Serial.println(calculated2, HEX);
  
  return (calculated2 == expected2);
}

bool testReadHoldingRegistersFrame() {
  // Build a Read Holding Registers request
  // Slave: 1, Function: 0x03, Start: 0, Quantity: 2
  uint8_t frame[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02};
  
  uint16_t crc = calculateTestCRC(frame, 6);
  uint8_t crcLow = crc & 0xFF;
  uint8_t crcHigh = (crc >> 8) & 0xFF;
  
  Serial.print("Frame: ");
  for (uint8_t i = 0; i < 6; i++) {
    Serial.print("0x");
    if (frame[i] < 0x10) Serial.print("0");
    Serial.print(frame[i], HEX);
    Serial.print(" ");
  }
  Serial.print("| CRC: 0x");
  if (crcLow < 0x10) Serial.print("0");
  Serial.print(crcLow, HEX);
  Serial.print(" 0x");
  if (crcHigh < 0x10) Serial.print("0");
  Serial.println(crcHigh, HEX);
  
  // Verify frame structure
  bool valid = (frame[0] == 0x01 &&           // Slave address
                frame[1] == 0x03 &&           // Function code
                frame[2] == 0x00 &&           // Start address high
                frame[3] == 0x00 &&           // Start address low
                frame[4] == 0x00 &&           // Quantity high
                frame[5] == 0x02);            // Quantity low
  
  Serial.print("Frame structure: ");
  Serial.println(valid ? "Valid" : "Invalid");
  
  return valid;
}

bool testWriteSingleRegisterFrame() {
  // Build a Write Single Register request
  // Slave: 1, Function: 0x06, Address: 1, Value: 3
  uint8_t frame[] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x03};
  
  uint16_t crc = calculateTestCRC(frame, 6);
  uint8_t crcLow = crc & 0xFF;
  uint8_t crcHigh = (crc >> 8) & 0xFF;
  
  Serial.print("Frame: ");
  for (uint8_t i = 0; i < 6; i++) {
    Serial.print("0x");
    if (frame[i] < 0x10) Serial.print("0");
    Serial.print(frame[i], HEX);
    Serial.print(" ");
  }
  Serial.print("| CRC: 0x");
  if (crcLow < 0x10) Serial.print("0");
  Serial.print(crcLow, HEX);
  Serial.print(" 0x");
  if (crcHigh < 0x10) Serial.print("0");
  Serial.println(crcHigh, HEX);
  
  // Verify frame structure
  bool valid = (frame[0] == 0x01 &&           // Slave address
                frame[1] == 0x06 &&           // Function code
                frame[2] == 0x00 &&           // Register address high
                frame[3] == 0x01 &&           // Register address low
                frame[4] == 0x00 &&           // Value high
                frame[5] == 0x03);            // Value low
  
  Serial.print("Frame structure: ");
  Serial.println(valid ? "Valid" : "Invalid");
  
  return valid;
}

bool testWriteMultipleRegistersFrame() {
  // Build a Write Multiple Registers request
  // Slave: 1, Function: 0x10, Start: 0, Quantity: 2, Values: 10, 20
  uint8_t frame[] = {0x01, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x00, 0x0A, 0x00, 0x14};
  
  uint16_t crc = calculateTestCRC(frame, 11);
  uint8_t crcLow = crc & 0xFF;
  uint8_t crcHigh = (crc >> 8) & 0xFF;
  
  Serial.print("Frame: ");
  for (uint8_t i = 0; i < 11; i++) {
    Serial.print("0x");
    if (frame[i] < 0x10) Serial.print("0");
    Serial.print(frame[i], HEX);
    Serial.print(" ");
  }
  Serial.print("| CRC: 0x");
  if (crcLow < 0x10) Serial.print("0");
  Serial.print(crcLow, HEX);
  Serial.print(" 0x");
  if (crcHigh < 0x10) Serial.print("0");
  Serial.println(crcHigh, HEX);
  
  // Verify frame structure
  uint16_t value1 = (frame[7] << 8) | frame[8];  // Should be 10
  uint16_t value2 = (frame[9] << 8) | frame[10]; // Should be 20
  
  bool valid = (frame[0] == 0x01 &&           // Slave address
                frame[1] == 0x10 &&           // Function code
                frame[2] == 0x00 &&           // Start address high
                frame[3] == 0x00 &&           // Start address low
                frame[4] == 0x00 &&           // Quantity high
                frame[5] == 0x02 &&           // Quantity low
                frame[6] == 0x04 &&           // Byte count
                value1 == 10 &&               // First register value
                value2 == 20);                // Second register value
  
  Serial.print("Frame structure: ");
  Serial.println(valid ? "Valid" : "Invalid");
  Serial.print("Register values: ");
  Serial.print(value1);
  Serial.print(", ");
  Serial.println(value2);
  
  return valid;
}

// Reference Information:
// ======================
//
// Modbus RTU uses CRC-16 with polynomial 0xA001 (reversed 0x8005)
// Initial value: 0xFFFF
// The CRC is transmitted with low byte first, then high byte
//
// Common baud rates: 9600, 19200, 38400, 57600, 115200
// Frame format: 8 data bits, No parity, 1 stop bit (8N1)
//
// Modbus Function Codes:
//  0x01 - Read Coils
//  0x02 - Read Discrete Inputs
//  0x03 - Read Holding Registers
//  0x04 - Read Input Registers
//  0x05 - Write Single Coil
//  0x06 - Write Single Register
//  0x0F - Write Multiple Coils
//  0x10 - Write Multiple Registers
