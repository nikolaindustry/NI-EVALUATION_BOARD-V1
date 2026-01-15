// Example: Modbus RTU Slave - Device Control via Modbus
//
// This example demonstrates how to use the NI Evaluation Board as a Modbus RTU slave
// device that can be controlled by PLCs, SCADA systems, or other Modbus masters.
//
// Hardware:
//  - The board includes automatic RS485 direction control via 555 IC
//  - Connect RS485 A/B lines to your Modbus master device
//  - RS485 RX on GPIO14, TX on GPIO13
//
// Modbus Map:
//  Coils (0-9):
//    0: Relay 1 control
//    1: Relay 2 control
//    2: Status LED control
//    3-7: Reserved
//
//  Discrete Inputs (0-7):
//    0-7: Digital inputs 1-8
//
//  Holding Registers (0-19):
//    0: DAC Channel 0 voltage (mV, 0-10000)
//    1: DAC Channel 1 voltage (mV, 0-10000)
//    2: PWM Channel 1 duty cycle (0-255)
//    3: PWM Channel 2 duty cycle (0-255)
//    4-9: Reserved for future use
//    10: Relay status (bit 0: Relay1, bit 1: Relay2)
//    11-19: Reserved
//
//  Input Registers (0-19):
//    0-7: Digital input status (0=LOW, 1=HIGH)
//    8: Pulse counter 1 (low word)
//    9: Pulse counter 1 (high word)
//    10: Pulse counter 2 (low word)
//    11: Pulse counter 2 (high word)
//    12-19: Reserved

#include <NI-EVALUATION_BOARD-V1.h>

NIEvaluationBoardV1 board;
NIModbusRTUSlave *modbus;

// Modbus Configuration
const uint8_t SLAVE_ADDRESS = 1;        // This device's Modbus address
const uint32_t MODBUS_BAUD = 115200;    // Match HypervisorIOT baud rate

// Storage for holding registers
uint16_t holdingRegisters[20];
bool relay1State = false;
bool relay2State = false;
bool statusLedState = false;

void setup() {
  Serial.begin(115200);
  delay(200);
  
  Serial.println("========================================");
  Serial.println("  NI Evaluation Board V1");
  Serial.println("  Modbus RTU Slave Example");
  Serial.println("========================================");
  Serial.println();
  
  // Initialize board with Modbus baud rate
  if (!board.begin(MODBUS_BAUD)) {
    Serial.println("ERROR: Board initialization failed!");
    while (1) delay(100);
  }
  
  Serial.println("Board initialized successfully");
  
  // Initialize holding registers
  memset(holdingRegisters, 0, sizeof(holdingRegisters));
  
  // Create Modbus RTU Slave instance
  modbus = board.createModbusSlave(SLAVE_ADDRESS);
  
  // Register callback functions
  modbus->onReadCoil(readCoilCallback);
  modbus->onWriteCoil(writeCoilCallback);
  modbus->onReadHoldingRegister(readHoldingRegisterCallback);
  modbus->onReadInputRegister(readInputRegisterCallback);
  modbus->onWriteHoldingRegister(writeHoldingRegisterCallback);
  
  Serial.print("Modbus RTU Slave ready - Address: ");
  Serial.print(SLAVE_ADDRESS);
  Serial.print(" - Baud: ");
  Serial.println(MODBUS_BAUD);
  Serial.println();
  Serial.println("Waiting for Modbus commands...");
  Serial.println();
}

void loop() {
  // Process incoming Modbus requests
  modbus->process();
  
  // Small delay to avoid overwhelming the processor
  delay(10);
}

// ============================================
// Modbus Callback Functions
// ============================================

// Read Coil (Discrete Output) - Used for relays and LED control
uint8_t readCoilCallback(uint16_t address, bool *value) {
  if (address == 0) {
    *value = relay1State;
    return 0;  // Success
  } else if (address == 1) {
    *value = relay2State;
    return 0;
  } else if (address == 2) {
    *value = statusLedState;
    return 0;
  } else if (address <= 7) {
    *value = false;  // Reserved coils
    return 0;
  }
  
  return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
}

// Write Coil (Discrete Output) - Control relays and LED
uint8_t writeCoilCallback(uint16_t address, bool value) {
  Serial.print("Modbus Write Coil - Address: ");
  Serial.print(address);
  Serial.print(", Value: ");
  Serial.println(value ? "ON" : "OFF");
  
  if (address == 0) {
    // Control Relay 1
    relay1State = value;
    board.setRelay(1, value);
    return 0;
  } else if (address == 1) {
    // Control Relay 2
    relay2State = value;
    board.setRelay(2, value);
    return 0;
  } else if (address == 2) {
    // Control Status LED
    statusLedState = value;
    digitalWrite(NIEvaluationBoardV1::PIN_STATUS_LED, value ? HIGH : LOW);
    return 0;
  } else if (address <= 7) {
    // Reserved coils - accept but do nothing
    return 0;
  }
  
  return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
}

// Read Holding Register - Used for DAC, PWM settings
uint8_t readHoldingRegisterCallback(uint16_t address, uint16_t *value) {
  if (address < 20) {
    *value = holdingRegisters[address];
    return 0;
  }
  
  return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
}

// Write Holding Register - Control DAC outputs and PWM
uint8_t writeHoldingRegisterCallback(uint16_t address, uint16_t value) {
  Serial.print("Modbus Write Register - Address: ");
  Serial.print(address);
  Serial.print(", Value: ");
  Serial.println(value);
  
  if (address == 0) {
    // DAC Channel 0 voltage in mV (0-10000)
    if (value > 10000) value = 10000;
    holdingRegisters[0] = value;
    board.setAnalogVoltage(0, value / 1000.0);
    return 0;
    
  } else if (address == 1) {
    // DAC Channel 1 voltage in mV (0-10000)
    if (value > 10000) value = 10000;
    holdingRegisters[1] = value;
    board.setAnalogVoltage(1, value / 1000.0);
    return 0;
    
  } else if (address == 2) {
    // PWM Channel 1 duty cycle (0-255)
    if (value > 255) value = 255;
    holdingRegisters[2] = value;
    board.setPwmDuty(1, value);
    return 0;
    
  } else if (address == 3) {
    // PWM Channel 2 duty cycle (0-255)
    if (value > 255) value = 255;
    holdingRegisters[3] = value;
    board.setPwmDuty(2, value);
    return 0;
    
  } else if (address == 10) {
    // Relay status - bit 0: Relay1, bit 1: Relay2
    relay1State = (value & 0x01) != 0;
    relay2State = (value & 0x02) != 0;
    board.setRelay(1, relay1State);
    board.setRelay(2, relay2State);
    holdingRegisters[10] = value;
    return 0;
    
  } else if (address < 20) {
    // Other holding registers - just store the value
    holdingRegisters[address] = value;
    return 0;
  }
  
  return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
}

// Read Input Register - Read sensors, digital inputs, pulse counters
uint8_t readInputRegisterCallback(uint16_t address, uint16_t *value) {
  if (address <= 7) {
    // Digital inputs 1-8 (0=LOW, 1=HIGH)
    *value = board.readDigitalInput(address + 1) ? 1 : 0;
    return 0;
    
  } else if (address == 8) {
    // Pulse counter 1 - low word
    unsigned long count = board.getPulseCount(1, false);
    *value = count & 0xFFFF;
    return 0;
    
  } else if (address == 9) {
    // Pulse counter 1 - high word
    unsigned long count = board.getPulseCount(1, false);
    *value = (count >> 16) & 0xFFFF;
    return 0;
    
  } else if (address == 10) {
    // Pulse counter 2 - low word
    unsigned long count = board.getPulseCount(2, false);
    *value = count & 0xFFFF;
    return 0;
    
  } else if (address == 11) {
    // Pulse counter 2 - high word
    unsigned long count = board.getPulseCount(2, false);
    *value = (count >> 16) & 0xFFFF;
    return 0;
    
  } else if (address < 20) {
    // Reserved input registers
    *value = 0;
    return 0;
  }
  
  return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
}

// ============================================
// Example Modbus Commands to Test This Slave
// ============================================
//
// Use a Modbus master tool (QModMaster, ModbusPoll, etc.) to test:
//
// 1. Control Relay 1 ON:
//    Function: Write Single Coil (0x05)
//    Address: 0
//    Value: 0xFF00 (ON)
//
// 2. Control Relay 1 OFF:
//    Function: Write Single Coil (0x05)
//    Address: 0
//    Value: 0x0000 (OFF)
//
// 3. Set DAC Channel 0 to 5V (5000 mV):
//    Function: Write Single Register (0x06)
//    Address: 0
//    Value: 5000
//
// 4. Set PWM Channel 1 to 50% duty (128/255):
//    Function: Write Single Register (0x06)
//    Address: 2
//    Value: 128
//
// 5. Read Digital Input 1:
//    Function: Read Input Registers (0x04)
//    Address: 0
//    Quantity: 1
//
// 6. Read Pulse Counter 1 (32-bit):
//    Function: Read Input Registers (0x04)
//    Address: 8
//    Quantity: 2
//    (Combine low and high words: count = (high << 16) | low)
