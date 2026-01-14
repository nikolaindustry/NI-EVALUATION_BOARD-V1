// Example: Modbus RTU Master - PLC/SCADA Integration
//
// This example demonstrates how to use the NI Evaluation Board as a Modbus RTU master
// to communicate with PLCs, SCADA systems, or other Modbus slave devices.
//
// Hardware:
//  - The board includes automatic RS485 direction control via 555 IC
//  - Connect RS485 A/B lines to your Modbus slave device
//  - RS485 RX on GPIO14, TX on GPIO13
//
// Modbus Functions Demonstrated:
//  - Read Holding Registers (FC 0x03)
//  - Read Input Registers (FC 0x04)
//  - Write Single Register (FC 0x06)
//  - Write Multiple Registers (FC 0x10)
//
// Usage:
//  - Configure SLAVE_ADDRESS to match your slave device
//  - Upload this sketch to the master board
//  - Open Serial Monitor at 115200 baud

#include <NI-EVALUATION_BOARD-V1.h>

NIEvaluationBoardV1 board;
NIModbusRTUMaster *modbus;

// Modbus Configuration
const uint8_t SLAVE_ADDRESS = 1;        // Address of the slave device
const uint32_t MODBUS_BAUD = 9600;      // Common Modbus baud rate (9600, 19200, 115200)

void setup() {
  Serial.begin(115200);
  delay(200);
  
  Serial.println("========================================");
  Serial.println("  NI Evaluation Board V1");
  Serial.println("  Modbus RTU Master Example");
  Serial.println("========================================");
  Serial.println();
  
  // Initialize board with Modbus baud rate
  if (!board.begin(MODBUS_BAUD)) {
    Serial.println("ERROR: Board initialization failed!");
    while (1) delay(100);
  }
  
  Serial.println("Board initialized successfully");
  
  // Create Modbus RTU Master instance
  modbus = board.createModbusMaster();
  
  Serial.print("Modbus RTU Master ready at ");
  Serial.print(MODBUS_BAUD);
  Serial.println(" baud");
  Serial.println();
  
  delay(1000);
}

void loop() {
  Serial.println("========================================");
  Serial.println("Starting Modbus operations...");
  Serial.println();
  
  // Example 1: Read Holding Registers (FC 0x03)
  // Read 4 registers starting from address 0
  readHoldingRegistersExample();
  
  delay(1000);
  
  // Example 2: Read Input Registers (FC 0x04)
  // Read 2 input registers starting from address 100
  readInputRegistersExample();
  
  delay(1000);
  
  // Example 3: Write Single Register (FC 0x06)
  // Write value 1234 to register 0
  writeSingleRegisterExample();
  
  delay(1000);
  
  // Example 4: Write Multiple Registers (FC 0x10)
  // Write multiple values to registers
  writeMultipleRegistersExample();
  
  delay(1000);
  
  // Example 5: Read and Display as PLC Values
  readPLCValues();
  
  Serial.println();
  Serial.println("Cycle complete. Waiting 5 seconds...");
  Serial.println();
  
  delay(5000);
}

void readHoldingRegistersExample() {
  Serial.println("--- Read Holding Registers (FC 0x03) ---");
  
  uint16_t registers[4];
  uint16_t startAddr = 0;
  uint16_t quantity = 4;
  
  Serial.print("Reading ");
  Serial.print(quantity);
  Serial.print(" holding registers from address ");
  Serial.print(startAddr);
  Serial.println("...");
  
  if (modbus->readHoldingRegisters(SLAVE_ADDRESS, startAddr, quantity, registers)) {
    Serial.println("SUCCESS! Register values:");
    for (uint16_t i = 0; i < quantity; i++) {
      Serial.print("  Register ");
      Serial.print(startAddr + i);
      Serial.print(": ");
      Serial.print(registers[i]);
      Serial.print(" (0x");
      Serial.print(registers[i], HEX);
      Serial.println(")");
    }
  } else {
    Serial.print("FAILED! Exception code: 0x");
    Serial.println(modbus->getLastException(), HEX);
  }
  
  Serial.println();
}

void readInputRegistersExample() {
  Serial.println("--- Read Input Registers (FC 0x04) ---");
  
  uint16_t registers[2];
  uint16_t startAddr = 100;
  uint16_t quantity = 2;
  
  Serial.print("Reading ");
  Serial.print(quantity);
  Serial.print(" input registers from address ");
  Serial.print(startAddr);
  Serial.println("...");
  
  if (modbus->readInputRegisters(SLAVE_ADDRESS, startAddr, quantity, registers)) {
    Serial.println("SUCCESS! Register values:");
    for (uint16_t i = 0; i < quantity; i++) {
      Serial.print("  Input Register ");
      Serial.print(startAddr + i);
      Serial.print(": ");
      Serial.println(registers[i]);
    }
  } else {
    Serial.print("FAILED! Exception code: 0x");
    Serial.println(modbus->getLastException(), HEX);
  }
  
  Serial.println();
}

void writeSingleRegisterExample() {
  Serial.println("--- Write Single Register (FC 0x06) ---");
  
  uint16_t addr = 0;
  uint16_t value = 1234;
  
  Serial.print("Writing value ");
  Serial.print(value);
  Serial.print(" to register ");
  Serial.print(addr);
  Serial.println("...");
  
  if (modbus->writeSingleRegister(SLAVE_ADDRESS, addr, value)) {
    Serial.println("SUCCESS! Register written");
    
    // Verify by reading back
    uint16_t readback;
    if (modbus->readHoldingRegisters(SLAVE_ADDRESS, addr, 1, &readback)) {
      Serial.print("Verification: Register value is ");
      Serial.println(readback);
    }
  } else {
    Serial.print("FAILED! Exception code: 0x");
    Serial.println(modbus->getLastException(), HEX);
  }
  
  Serial.println();
}

void writeMultipleRegistersExample() {
  Serial.println("--- Write Multiple Registers (FC 0x10) ---");
  
  uint16_t startAddr = 10;
  uint16_t values[] = {100, 200, 300, 400, 500};
  uint16_t quantity = 5;
  
  Serial.print("Writing ");
  Serial.print(quantity);
  Serial.print(" registers starting at address ");
  Serial.print(startAddr);
  Serial.println("...");
  
  if (modbus->writeMultipleRegisters(SLAVE_ADDRESS, startAddr, quantity, values)) {
    Serial.println("SUCCESS! Registers written");
    
    // Verify by reading back
    uint16_t readback[5];
    if (modbus->readHoldingRegisters(SLAVE_ADDRESS, startAddr, quantity, readback)) {
      Serial.println("Verification:");
      for (uint16_t i = 0; i < quantity; i++) {
        Serial.print("  Register ");
        Serial.print(startAddr + i);
        Serial.print(": ");
        Serial.println(readback[i]);
      }
    }
  } else {
    Serial.print("FAILED! Exception code: 0x");
    Serial.println(modbus->getLastException(), HEX);
  }
  
  Serial.println();
}

void readPLCValues() {
  Serial.println("--- Reading PLC/SCADA Values ---");
  
  // Example: Read temperature, pressure, flow rate from slave device
  // This demonstrates typical industrial automation use case
  
  uint16_t processValues[3];
  
  if (modbus->readInputRegisters(SLAVE_ADDRESS, 0, 3, processValues)) {
    // Convert raw register values to engineering units
    // Example assumes: Reg0=Temperature*10, Reg1=Pressure*10, Reg2=Flow*100
    
    float temperature = processValues[0] / 10.0;
    float pressure = processValues[1] / 10.0;
    float flowRate = processValues[2] / 100.0;
    
    Serial.println("Process Values:");
    Serial.print("  Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
    
    Serial.print("  Pressure: ");
    Serial.print(pressure);
    Serial.println(" bar");
    
    Serial.print("  Flow Rate: ");
    Serial.print(flowRate);
    Serial.println(" L/min");
  } else {
    Serial.println("Failed to read process values");
  }
  
  Serial.println();
}

// Modbus Exception Codes Reference:
// 0x01 - Illegal Function
// 0x02 - Illegal Data Address
// 0x03 - Illegal Data Value
// 0x04 - Slave Device Failure
