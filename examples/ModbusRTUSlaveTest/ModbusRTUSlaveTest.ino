// Modbus RTU Slave Test - Board 2
//
// This sketch runs on the SLAVE board to test Modbus RTU communication
// between two NI Evaluation Boards.
//
// Hardware Setup:
//  - Connect RS485 A to RS485 A between both boards
//  - Connect RS485 B to RS485 B between both boards
//  - Connect GND to GND between both boards
//  - Add 120Ω termination resistor between A and B on each board
//
// Test Functions:
//  - Responds to master's read/write requests
//  - Controls relays, DAC, PWM based on Modbus commands
//  - Provides digital input states to master
//  - Logs all Modbus activity to Serial Monitor
//
// Upload this to Board 2 (Slave)

#include <NI-EVALUATION_BOARD-V1.h>

NIEvaluationBoardV1 board;
NIModbusRTUSlave *modbus;

// Modbus Configuration
const uint8_t SLAVE_ADDRESS = 1;        // Must match master's target address
const uint32_t MODBUS_BAUD = 9600;      // Must match master's baud rate

// Data storage
uint16_t holdingRegisters[20];
bool relay1State = false;
bool relay2State = false;
bool statusLedState = false;

unsigned long lastActivityTime = 0;
unsigned long requestCount = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("========================================");
  Serial.println("  MODBUS RTU SLAVE TEST");
  Serial.println("  NI Evaluation Board V1");
  Serial.println("========================================");
  Serial.println();
  
  // Initialize board with Modbus baud rate
  if (!board.begin(MODBUS_BAUD)) {
    Serial.println("ERROR: Board initialization failed!");
    while (1) delay(100);
  }
  
  Serial.println("✓ Board initialized");
  
  // Initialize holding registers
  memset(holdingRegisters, 0, sizeof(holdingRegisters));
  
  // Create Modbus Slave
  modbus = board.createModbusSlave(SLAVE_ADDRESS);
  
  // Register all callback functions
  modbus->onReadCoil(readCoilCallback);
  modbus->onWriteCoil(writeCoilCallback);
  modbus->onReadHoldingRegister(readHoldingRegisterCallback);
  modbus->onReadInputRegister(readInputRegisterCallback);
  modbus->onWriteHoldingRegister(writeHoldingRegisterCallback);
  
  Serial.print("✓ Modbus Slave ready - Address: ");
  Serial.println(SLAVE_ADDRESS);
  Serial.print("✓ Baud rate: ");
  Serial.println(MODBUS_BAUD);
  Serial.println();
  Serial.println("Waiting for master requests...");
  Serial.println();
  
  // Flash status LED to indicate ready
  for (int i = 0; i < 3; i++) {
    digitalWrite(NIEvaluationBoardV1::PIN_STATUS_LED, HIGH);
    delay(200);
    digitalWrite(NIEvaluationBoardV1::PIN_STATUS_LED, LOW);
    delay(200);
  }
}

void loop() {
  // Process Modbus requests
  modbus->process();
  
  // Check for activity timeout
  if (requestCount > 0 && millis() - lastActivityTime > 5000) {
    Serial.println();
    Serial.println("⏱ No activity for 5 seconds...");
    Serial.print("Total requests processed: ");
    Serial.println(requestCount);
    Serial.println();
    lastActivityTime = millis();
  }
  
  // Small delay
  delay(10);
}

// ============================================
// Modbus Callback Functions
// ============================================

// Read Coil (0x01, 0x02)
uint8_t readCoilCallback(uint16_t address, bool *value) {
  logRequest("READ COIL", address);
  
  if (address == 0) {
    *value = relay1State;
    logResponse("Relay 1 state", relay1State ? "ON" : "OFF");
    return 0;
  } else if (address == 1) {
    *value = relay2State;
    logResponse("Relay 2 state", relay2State ? "ON" : "OFF");
    return 0;
  } else if (address == 2) {
    *value = statusLedState;
    logResponse("Status LED", statusLedState ? "ON" : "OFF");
    return 0;
  } else if (address <= 7) {
    *value = false;  // Reserved
    return 0;
  }
  
  logError("Illegal address");
  return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
}

// Write Coil (0x05, 0x0F)
uint8_t writeCoilCallback(uint16_t address, bool value) {
  logRequest("WRITE COIL", address);
  
  if (address == 0) {
    // Control Relay 1
    relay1State = value;
    board.setRelay(1, value);
    logResponse("Relay 1", value ? "ON" : "OFF");
    Serial.println("  ↳ You should hear relay click!");
    return 0;
    
  } else if (address == 1) {
    // Control Relay 2
    relay2State = value;
    board.setRelay(2, value);
    logResponse("Relay 2", value ? "ON" : "OFF");
    Serial.println("  ↳ You should hear relay click!");
    return 0;
    
  } else if (address == 2) {
    // Control Status LED
    statusLedState = value;
    digitalWrite(NIEvaluationBoardV1::PIN_STATUS_LED, value ? HIGH : LOW);
    logResponse("Status LED", value ? "ON" : "OFF");
    return 0;
    
  } else if (address <= 7) {
    // Reserved coils
    return 0;
  }
  
  logError("Illegal address");
  return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
}

// Read Holding Register (0x03)
uint8_t readHoldingRegisterCallback(uint16_t address, uint16_t *value) {
  logRequest("READ HOLDING REG", address);
  
  if (address < 20) {
    *value = holdingRegisters[address];
    
    if (address == 0) {
      logResponse("DAC0 voltage (mV)", String(*value));
    } else if (address == 1) {
      logResponse("DAC1 voltage (mV)", String(*value));
    } else if (address == 2) {
      logResponse("PWM1 duty", String(*value));
    } else if (address == 3) {
      logResponse("PWM2 duty", String(*value));
    } else {
      logResponse("Register value", String(*value));
    }
    return 0;
  }
  
  logError("Illegal address");
  return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
}

// Read Input Register (0x04)
uint8_t readInputRegisterCallback(uint16_t address, uint16_t *value) {
  logRequest("READ INPUT REG", address);
  
  if (address <= 7) {
    // Digital inputs 1-8
    *value = board.readDigitalInput(address + 1) ? 1 : 0;
    logResponse("Digital Input " + String(address + 1), *value ? "HIGH" : "LOW");
    return 0;
    
  } else if (address == 8) {
    // Pulse counter 1 - low word
    unsigned long count = board.getPulseCount(1, false);
    *value = count & 0xFFFF;
    logResponse("Pulse Count 1 (low)", String(*value));
    return 0;
    
  } else if (address == 9) {
    // Pulse counter 1 - high word
    unsigned long count = board.getPulseCount(1, false);
    *value = (count >> 16) & 0xFFFF;
    logResponse("Pulse Count 1 (high)", String(*value));
    return 0;
    
  } else if (address == 10) {
    // Pulse counter 2 - low word
    unsigned long count = board.getPulseCount(2, false);
    *value = count & 0xFFFF;
    logResponse("Pulse Count 2 (low)", String(*value));
    return 0;
    
  } else if (address == 11) {
    // Pulse counter 2 - high word
    unsigned long count = board.getPulseCount(2, false);
    *value = (count >> 16) & 0xFFFF;
    logResponse("Pulse Count 2 (high)", String(*value));
    return 0;
    
  } else if (address < 20) {
    // Reserved
    *value = 0;
    return 0;
  }
  
  logError("Illegal address");
  return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
}

// Write Holding Register (0x06, 0x10)
uint8_t writeHoldingRegisterCallback(uint16_t address, uint16_t value) {
  logRequest("WRITE HOLDING REG", address);
  
  if (address == 0) {
    // DAC Channel 0 voltage in mV (0-10000)
    if (value > 10000) value = 10000;
    holdingRegisters[0] = value;
    board.setAnalogVoltage(0, value / 1000.0);
    logResponse("DAC0 set to", String(value / 1000.0) + " V");
    Serial.println("  ↳ Measure voltage on DAC0 output!");
    return 0;
    
  } else if (address == 1) {
    // DAC Channel 1 voltage in mV (0-10000)
    if (value > 10000) value = 10000;
    holdingRegisters[1] = value;
    board.setAnalogVoltage(1, value / 1000.0);
    logResponse("DAC1 set to", String(value / 1000.0) + " V");
    Serial.println("  ↳ Measure voltage on DAC1 output!");
    return 0;
    
  } else if (address == 2) {
    // PWM Channel 1 duty cycle (0-255)
    if (value > 255) value = 255;
    holdingRegisters[2] = value;
    board.setPwmDuty(1, value);
    logResponse("PWM1 duty", String(value) + " (" + String(value * 100 / 255) + "%)");
    return 0;
    
  } else if (address == 3) {
    // PWM Channel 2 duty cycle (0-255)
    if (value > 255) value = 255;
    holdingRegisters[3] = value;
    board.setPwmDuty(2, value);
    logResponse("PWM2 duty", String(value) + " (" + String(value * 100 / 255) + "%)");
    return 0;
    
  } else if (address == 10) {
    // Relay control register - bit 0: Relay1, bit 1: Relay2
    relay1State = (value & 0x01) != 0;
    relay2State = (value & 0x02) != 0;
    board.setRelay(1, relay1State);
    board.setRelay(2, relay2State);
    holdingRegisters[10] = value;
    logResponse("Relays", "R1=" + String(relay1State ? "ON" : "OFF") + 
                ", R2=" + String(relay2State ? "ON" : "OFF"));
    return 0;
    
  } else if (address < 20) {
    // Other holding registers
    holdingRegisters[address] = value;
    logResponse("Register updated", String(value));
    return 0;
  }
  
  logError("Illegal address");
  return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
}

// ============================================
// Helper Functions for Serial Logging
// ============================================

void logRequest(String type, uint16_t address) {
  requestCount++;
  lastActivityTime = millis();
  
  Serial.print("📥 [");
  Serial.print(requestCount);
  Serial.print("] ");
  Serial.print(type);
  Serial.print(" @ 0x");
  if (address < 16) Serial.print("0");
  Serial.println(address, HEX);
}

void logResponse(String param, String value) {
  Serial.print("  ✓ ");
  Serial.print(param);
  Serial.print(": ");
  Serial.println(value);
}

void logError(String message) {
  Serial.print("  ✗ ERROR: ");
  Serial.println(message);
}

// Troubleshooting Guide:
// ======================
//
// No requests received:
// 1. Check RS485 wiring (A to A, B to B, GND to GND)
// 2. Verify master board is running and transmitting
// 3. Check baud rate matches (9600)
// 4. Ensure slave address matches (1)
// 5. Swap A and B if no communication
//
// Requests received but errors:
// 1. Check Serial Monitor for exception codes
// 2. Verify register addresses are valid
// 3. Check callback return values
//
// Expected behavior:
// - Every 2 seconds, you should see request logs
// - Relay should click (toggle every cycle)
// - DAC voltage should change (0-10V)
// - PWM duty should vary
// - Status LED may blink based on master commands
//
// Success indicators:
// - Serial shows "📥 [#] REQUEST_TYPE @ 0xADDR"
// - Each request shows "✓ Response"
// - No "✗ ERROR" messages
// - Relay clicks audibly
// - DAC voltage measured with voltmeter changes
