// Modbus RTU Master Test - Board 1
//
// This sketch runs on the MASTER board to test Modbus RTU communication
// between two NI Evaluation Boards.
//
// Hardware Setup:
//  - Connect RS485 A to RS485 A between both boards
//  - Connect RS485 B to RS485 B between both boards
//  - Connect GND to GND between both boards
//  - Add 120Ω termination resistor between A and B on each board
//
// Test Functions:
//  - Reads holding registers from slave
//  - Writes single register to control slave's DAC
//  - Writes coil to control slave's relay
//  - Reads input registers (digital inputs) from slave
//
// Upload this to Board 1 (Master)

#include <NI-EVALUATION_BOARD-V1.h>

NIEvaluationBoardV1 board;
NIModbusRTUMaster *modbus;

// Modbus Configuration
const uint8_t SLAVE_ADDRESS = 1;        // Must match slave address
const uint32_t MODBUS_BAUD = 9600;      // Must match on both boards

unsigned long lastTestTime = 0;
const unsigned long TEST_INTERVAL = 2000;  // Test every 2 seconds
int testCounter = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("========================================");
  Serial.println("  MODBUS RTU MASTER TEST");
  Serial.println("  NI Evaluation Board V1");
  Serial.println("========================================");
  Serial.println();
  
  // Initialize board with Modbus baud rate
  if (!board.begin(MODBUS_BAUD)) {
    Serial.println("ERROR: Board initialization failed!");
    while (1) delay(100);
  }
  
  Serial.println("✓ Board initialized");
  
  // Create Modbus Master
  modbus = board.createModbusMaster();
  
  Serial.print("✓ Modbus Master ready at ");
  Serial.print(MODBUS_BAUD);
  Serial.println(" baud");
  Serial.print("✓ Slave address: ");
  Serial.println(SLAVE_ADDRESS);
  Serial.println();
  Serial.println("Waiting 3 seconds for slave to initialize...");
  delay(3000);
  
  Serial.println("Starting Modbus tests...");
  Serial.println();
}

void loop() {
  if (millis() - lastTestTime >= TEST_INTERVAL) {
    lastTestTime = millis();
    testCounter++;
    
    Serial.println("========================================");
    Serial.print("Test Cycle #");
    Serial.println(testCounter);
    Serial.println("========================================");
    Serial.println();
    
    // Test 1: Read Holding Registers
    testReadHoldingRegisters();
    delay(100);
    
    // Test 2: Write Single Register (DAC Control)
    testWriteSingleRegister();
    delay(100);
    
    // Test 3: Write Single Coil (Relay Control)
    testWriteSingleCoil();
    delay(100);
    
    // Test 4: Read Input Registers (Digital Inputs)
    testReadInputRegisters();
    delay(100);
    
    // Test 5: Write Multiple Registers
    testWriteMultipleRegisters();
    
    Serial.println();
    Serial.println("Waiting for next test cycle...");
    Serial.println();
  }
}

void testReadHoldingRegisters() {
  Serial.println("Test 1: Read Holding Registers");
  Serial.println("-------------------------------");
  
  uint16_t registers[4];
  
  if (modbus->readHoldingRegisters(SLAVE_ADDRESS, 0, 4, registers)) {
    Serial.println("✓ SUCCESS");
    Serial.print("  Register 0 (DAC0 mV): ");
    Serial.println(registers[0]);
    Serial.print("  Register 1 (DAC1 mV): ");
    Serial.println(registers[1]);
    Serial.print("  Register 2 (PWM1): ");
    Serial.println(registers[2]);
    Serial.print("  Register 3 (PWM2): ");
    Serial.println(registers[3]);
  } else {
    Serial.println("✗ FAILED");
    Serial.print("  Exception: 0x");
    Serial.println(modbus->getLastException(), HEX);
  }
  
  Serial.println();
}

void testWriteSingleRegister() {
  Serial.println("Test 2: Write Single Register (Control Slave DAC)");
  Serial.println("--------------------------------------------------");
  
  // Calculate DAC voltage based on test cycle (0-10V)
  uint16_t dacVoltage = (testCounter % 11) * 1000;  // 0, 1000, 2000...10000 mV
  
  Serial.print("  Writing ");
  Serial.print(dacVoltage);
  Serial.println(" mV to slave DAC channel 0...");
  
  if (modbus->writeSingleRegister(SLAVE_ADDRESS, 0, dacVoltage)) {
    Serial.println("✓ SUCCESS - Slave DAC0 should now output ");
    Serial.print("  ");
    Serial.print(dacVoltage / 1000.0);
    Serial.println(" V");
    
    // Verify by reading back
    uint16_t readback;
    if (modbus->readHoldingRegisters(SLAVE_ADDRESS, 0, 1, &readback)) {
      Serial.print("  Verified: ");
      Serial.print(readback);
      Serial.println(" mV");
    }
  } else {
    Serial.println("✗ FAILED");
    Serial.print("  Exception: 0x");
    Serial.println(modbus->getLastException(), HEX);
  }
  
  Serial.println();
}

void testWriteSingleCoil() {
  Serial.println("Test 3: Write Single Coil (Control Slave Relay)");
  Serial.println("------------------------------------------------");
  
  // Toggle relay state based on test cycle
  bool relayState = (testCounter % 2) == 0;
  
  Serial.print("  Turning slave Relay 1 ");
  Serial.println(relayState ? "ON" : "OFF");
  
  if (modbus->writeSingleCoil(SLAVE_ADDRESS, 0, relayState)) {
    Serial.println("✓ SUCCESS - Slave Relay 1 toggled");
    Serial.println("  Check if you hear the relay click on slave board");
  } else {
    Serial.println("✗ FAILED");
    Serial.print("  Exception: 0x");
    Serial.println(modbus->getLastException(), HEX);
  }
  
  Serial.println();
}

void testReadInputRegisters() {
  Serial.println("Test 4: Read Input Registers (Slave Digital Inputs)");
  Serial.println("----------------------------------------------------");
  
  uint16_t inputs[8];
  
  if (modbus->readInputRegisters(SLAVE_ADDRESS, 0, 8, inputs)) {
    Serial.println("✓ SUCCESS - Slave Digital Input States:");
    for (int i = 0; i < 8; i++) {
      Serial.print("  Input ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(inputs[i] ? "HIGH" : "LOW");
    }
  } else {
    Serial.println("✗ FAILED");
    Serial.print("  Exception: 0x");
    Serial.println(modbus->getLastException(), HEX);
  }
  
  Serial.println();
}

void testWriteMultipleRegisters() {
  Serial.println("Test 5: Write Multiple Registers (PWM Control)");
  Serial.println("-----------------------------------------------");
  
  // Set PWM duty cycles
  uint16_t pwmValues[2];
  pwmValues[0] = (testCounter * 25) % 256;  // PWM1: 0, 25, 50...255
  pwmValues[1] = 255 - pwmValues[0];        // PWM2: inverse of PWM1
  
  Serial.print("  Writing PWM1=");
  Serial.print(pwmValues[0]);
  Serial.print(", PWM2=");
  Serial.println(pwmValues[1]);
  
  if (modbus->writeMultipleRegisters(SLAVE_ADDRESS, 2, 2, pwmValues)) {
    Serial.println("✓ SUCCESS - Slave PWM outputs updated");
  } else {
    Serial.println("✗ FAILED");
    Serial.print("  Exception: 0x");
    Serial.println(modbus->getLastException(), HEX);
  }
  
  Serial.println();
}

// Troubleshooting Tips:
// ====================
//
// If all tests fail:
// 1. Check RS485 wiring (A to A, B to B, GND to GND)
// 2. Verify baud rate matches on both boards (9600)
// 3. Ensure slave board is powered and running
// 4. Check that slave address matches (default: 1)
// 5. Add 120Ω termination resistors if missing
//
// If some tests fail:
// 1. Check Serial Monitor on slave board for errors
// 2. Verify exception code (0x01-0x04)
// 3. Ensure slave callbacks are registered correctly
//
// Success indicators:
// - You should see "✓ SUCCESS" for all tests
// - Relay should click on slave board (every other cycle)
// - DAC voltage should change (use voltmeter on slave)
// - PWM should vary (use LED or oscilloscope)
