#include <NI-EVALUATION_BOARD-V1.h>
#include <hyperwisor-iot.h>

NIEvaluationBoardV1 board;
HyperwisorIOT hyper;
NIModbusRTUMaster *modbus;  // Modbus RTU Master instance

String apikey = "";
String secretkey = "";
String productId = "";
String deviceId = "";
String userId = "";
String msgfrom = "";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Initialize the board with default settings
  if (board.begin()) {
    Serial.println("Board initialized successfully!");
  } else {
    Serial.println("Board initialization failed!");
  }

  // Initialize Modbus RTU Master
  modbus = board.createModbusMaster();
  Serial.println("Modbus RTU Master initialized");

  hyper.setApiKeys(apikey, secretkey);
  // Time and date functions
  hyper.setTimezone("IST");
  // Core functions
  hyper.begin();

  // User-defined command callback
  hyper.setUserCommandHandler([](JsonObject& msg) {
    if (!msg.containsKey("payload")) return;

    msgfrom = msg["from"].as<String>();
    Serial.print("From: ");
    Serial.println(msgfrom);

    JsonObject payload = msg["payload"];
    Serial.println("Full payload:");
    serializeJson(payload, Serial);
    Serial.println();

    // Handle "get_ui" if exists
    if (payload.containsKey("command")) {
      String command = payload["command"].as<String>();
      Serial.print("Command: ");
      Serial.println(command);
      if (command == "get_ui") {
        // Handle dashboard update logic
        update_ui();
        return;
      }
    }

    // ============ RELAY CONTROL ============
    // Control Relay (channel: 1, 2, or 0 for all; state: true/false)
    JsonObject relayControl = hyper.findAction(payload, "Operate", "Relay_Control");
    if (!relayControl.isNull()) {
      uint8_t channel = relayControl["params"]["channel"] | 1;
      bool state = relayControl["params"]["state"] | false;
      
      if (channel == 0) {
        // Control all relays
        Serial.print("Command: All Relays ");
        Serial.println(state ? "ON" : "OFF");
        board.setAllRelays(state);
      } else if (channel >= 1 && channel <= 2) {
        // Control individual relay
        Serial.print("Command: Relay ");
        Serial.print(channel);
        Serial.print(" ");
        Serial.println(state ? "ON" : "OFF");
        board.setRelay(channel, state);
      } else {
        Serial.println("Error: Invalid relay channel (use 0 for all, 1-2 for individual)");
      }
    }

    // ============ DAC OUTPUT CONTROL (0-10V) ============
    // Set DAC Channel 0 Voltage
    JsonObject dac0Voltage = hyper.findAction(payload, "Operate", "DAC_CH0_Voltage");
    if (!dac0Voltage.isNull()) {
      float voltage = dac0Voltage["params"]["voltage"] | 0.0;
      Serial.print("Command: Set DAC CH0 to ");
      Serial.print(voltage);
      Serial.println(" V");
      board.setAnalogVoltage(0, voltage);
    }

    // Set DAC Channel 1 Voltage
    JsonObject dac1Voltage = hyper.findAction(payload, "Operate", "DAC_CH1_Voltage");
    if (!dac1Voltage.isNull()) {
      float voltage = dac1Voltage["params"]["voltage"] | 0.0;
      Serial.print("Command: Set DAC CH1 to ");
      Serial.print(voltage);
      Serial.println(" V");
      board.setAnalogVoltage(1, voltage);
    }

    // Set DAC Channel 0 Raw (0-4095)
    JsonObject dac0Raw = hyper.findAction(payload, "Operate", "DAC_CH0_Raw");
    if (!dac0Raw.isNull()) {
      uint16_t rawValue = dac0Raw["params"]["value"] | 0;
      Serial.print("Command: Set DAC CH0 Raw to ");
      Serial.println(rawValue);
      board.setAnalogRaw(0, rawValue);
    }

    // Set DAC Channel 1 Raw (0-4095)
    JsonObject dac1Raw = hyper.findAction(payload, "Operate", "DAC_CH1_Raw");
    if (!dac1Raw.isNull()) {
      uint16_t rawValue = dac1Raw["params"]["value"] | 0;
      Serial.print("Command: Set DAC CH1 Raw to ");
      Serial.println(rawValue);
      board.setAnalogRaw(1, rawValue);
    }

    // ============ PWM OUTPUT CONTROL ============
    // Set PWM Channel 1 Duty
    JsonObject pwm1Duty = hyper.findAction(payload, "Operate", "PWM_CH1_Duty");
    if (!pwm1Duty.isNull()) {
      uint8_t duty = pwm1Duty["params"]["duty"] | 0;
      Serial.print("Command: Set PWM CH1 Duty to ");
      Serial.println(duty);
      board.setPwmDuty(1, duty);
    }

    // Set PWM Channel 2 Duty
    JsonObject pwm2Duty = hyper.findAction(payload, "Operate", "PWM_CH2_Duty");
    if (!pwm2Duty.isNull()) {
      uint8_t duty = pwm2Duty["params"]["duty"] | 0;
      Serial.print("Command: Set PWM CH2 Duty to ");
      Serial.println(duty);
      board.setPwmDuty(2, duty);
    }

    // Configure PWM Frequency and Resolution
    JsonObject pwmConfig = hyper.findAction(payload, "Operate", "PWM_Configure");
    if (!pwmConfig.isNull()) {
      uint32_t freq = pwmConfig["params"]["frequency"] | 5000;
      uint8_t resolution = pwmConfig["params"]["resolution"] | 8;
      Serial.print("Command: Configure PWM - Freq: ");
      Serial.print(freq);
      Serial.print(" Hz, Resolution: ");
      Serial.print(resolution);
      Serial.println(" bits");
      board.configurePwm(freq, resolution);
    }

    // ============ DIGITAL INPUT READING ============
    // Read Single Digital Input
    JsonObject readInput = hyper.findAction(payload, "Query", "Read_Digital_Input");
    if (!readInput.isNull()) {
      uint8_t channel = readInput["params"]["channel"] | 1;
      bool state = board.readDigitalInput(channel);
      Serial.print("Query: Digital Input ");
      Serial.print(channel);
      Serial.print(" = ");
      Serial.println(state ? "HIGH" : "LOW");
      
      // Send response back
      JsonDocument responseDoc;
      JsonObject response = responseDoc.to<JsonObject>();
      response["input"] = channel;
      response["state"] = state;
      // hyper.sendMessage(msgfrom, response); // Uncomment to send response
    }

    // Read All Digital Inputs
    JsonObject readAllInputs = hyper.findAction(payload, "Query", "Read_All_Digital_Inputs");
    if (!readAllInputs.isNull()) {
      uint8_t inputMask = board.readDigitalInputs();
      Serial.print("Query: All Digital Inputs = 0b");
      Serial.println(inputMask, BIN);
      
      // Send response back
      JsonDocument responseDoc;
      JsonObject response = responseDoc.to<JsonObject>();
      for (uint8_t i = 1; i <= 8; i++) {
        String key = "IN" + String(i);
        response[key] = (inputMask & (1 << (i - 1))) ? true : false;
      }
      // hyper.sendMessage(msgfrom, response); // Uncomment to send response
    }

    // ============ PULSE COUNTER READING ============
    // Read Pulse Counter 1
    JsonObject readPulse1 = hyper.findAction(payload, "Query", "Read_Pulse_Counter_1");
    if (!readPulse1.isNull()) {
      bool reset = readPulse1["params"]["reset"] | false;
      unsigned long count = board.getPulseCount(1, reset);
      Serial.print("Query: Pulse Counter 1 = ");
      Serial.println(count);
      
      // Send response back
      JsonDocument responseDoc;
      JsonObject response = responseDoc.to<JsonObject>();
      response["counter"] = 1;
      response["count"] = count;
      // hyper.sendMessage(msgfrom, response); // Uncomment to send response
    }

    // Read Pulse Counter 2
    JsonObject readPulse2 = hyper.findAction(payload, "Query", "Read_Pulse_Counter_2");
    if (!readPulse2.isNull()) {
      bool reset = readPulse2["params"]["reset"] | false;
      unsigned long count = board.getPulseCount(2, reset);
      Serial.print("Query: Pulse Counter 2 = ");
      Serial.println(count);
      
      // Send response back
      JsonDocument responseDoc;
      JsonObject response = responseDoc.to<JsonObject>();
      response["counter"] = 2;
      response["count"] = count;
      // hyper.sendMessage(msgfrom, response); // Uncomment to send response
    }

    // Reset Pulse Counter
    JsonObject resetPulse = hyper.findAction(payload, "Operate", "Reset_Pulse_Counter");
    if (!resetPulse.isNull()) {
      uint8_t channel = resetPulse["params"]["channel"] | 1;
      board.getPulseCount(channel, true);
      Serial.print("Command: Reset Pulse Counter ");
      Serial.println(channel);
    }

    // ============ RS485 COMMUNICATION ============
    // Send RS485 Message
    JsonObject rs485Send = hyper.findAction(payload, "Operate", "RS485_Send");
    if (!rs485Send.isNull()) {
      String message = rs485Send["params"]["message"] | "";
      bool addNewline = rs485Send["params"]["newline"] | false;
      Serial.print("Command: RS485 Send: ");
      Serial.println(message);
      
      if (addNewline) {
        board.rs485WriteLine(message);
      } else {
        board.rs485Write(message);
      }
    }

    // Read RS485 Message
    JsonObject rs485Read = hyper.findAction(payload, "Query", "RS485_Read");
    if (!rs485Read.isNull()) {
      uint32_t timeout = rs485Read["params"]["timeout"] | 1000;
      String message = board.rs485ReadLine(timeout);
      Serial.print("Query: RS485 Received: ");
      Serial.println(message);
      
      // Send response back
      JsonDocument responseDoc;
      JsonObject response = responseDoc.to<JsonObject>();
      response["message"] = message;
      // hyper.sendMessage(msgfrom, response); // Uncomment to send response
    }

    // ============ STATUS LED CONTROL ============
    // Set Status LED
    JsonObject statusLED = hyper.findAction(payload, "Operate", "Status_LED");
    if (!statusLED.isNull()) {
      bool state = statusLED["params"]["state"] | false;
      Serial.print("Command: Status LED ");
      Serial.println(state ? "ON" : "OFF");
      digitalWrite(board.PIN_STATUS_LED, state ? HIGH : LOW);
    }

    // ============ I2C EXTERNAL DEVICE CONTROL ============
    // I2C Scanner - Scan all addresses
    JsonObject i2cScan = hyper.findAction(payload, "Query", "I2C_Scan");
    if (!i2cScan.isNull()) {
      Serial.println("Query: Scanning I2C bus...");
      JsonDocument responseDoc;
      JsonObject response = responseDoc.to<JsonObject>();
      JsonArray devices = response["devices"].to<JsonArray>();
      
      uint8_t deviceCount = 0;
      for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();
        
        if (error == 0) {
          Serial.print("I2C device found at address 0x");
          Serial.println(address, HEX);
          devices.add(address);
          deviceCount++;
        }
      }
      
      response["count"] = deviceCount;
      Serial.print("Total devices found: ");
      Serial.println(deviceCount);
      // hyper.sendMessage(msgfrom, response); // Uncomment to send response
    }

    // ============ MODBUS RTU MASTER CONTROL ============
    // Modbus Read Holding Registers
    JsonObject modbusReadHolding = hyper.findAction(payload, "Query", "Modbus_Read_Holding_Registers");
    if (!modbusReadHolding.isNull()) {
      uint8_t slaveAddr = modbusReadHolding["params"]["slave_address"] | 1;
      uint16_t startAddr = modbusReadHolding["params"]["start_address"] | 0;
      uint16_t quantity = modbusReadHolding["params"]["quantity"] | 1;
      
      if (quantity > 0 && quantity <= 125) {
        uint16_t registers[125];
        Serial.print("Query: Modbus Read Holding Regs - Slave:");
        Serial.print(slaveAddr);
        Serial.print(", Start:");
        Serial.print(startAddr);
        Serial.print(", Qty:");
        Serial.println(quantity);
        
        if (modbus->readHoldingRegisters(slaveAddr, startAddr, quantity, registers)) {
          Serial.println("Success!");
          
          // Send response back
          JsonDocument responseDoc;
          JsonObject response = responseDoc.to<JsonObject>();
          response["success"] = true;
          response["slave"] = slaveAddr;
          response["start"] = startAddr;
          JsonArray regArray = response["registers"].to<JsonArray>();
          
          for (uint16_t i = 0; i < quantity; i++) {
            regArray.add(registers[i]);
            Serial.print("  Reg ");
            Serial.print(startAddr + i);
            Serial.print(": ");
            Serial.println(registers[i]);
          }
          // hyper.sendMessage(msgfrom, response); // Uncomment to send response
        } else {
          Serial.print("Failed! Exception: 0x");
          Serial.println(modbus->getLastException(), HEX);
          
          // Send error response
          JsonDocument responseDoc;
          JsonObject response = responseDoc.to<JsonObject>();
          response["success"] = false;
          response["exception"] = modbus->getLastException();
          // hyper.sendMessage(msgfrom, response);
        }
      }
    }

    // Modbus Read Input Registers
    JsonObject modbusReadInput = hyper.findAction(payload, "Query", "Modbus_Read_Input_Registers");
    if (!modbusReadInput.isNull()) {
      uint8_t slaveAddr = modbusReadInput["params"]["slave_address"] | 1;
      uint16_t startAddr = modbusReadInput["params"]["start_address"] | 0;
      uint16_t quantity = modbusReadInput["params"]["quantity"] | 1;
      
      if (quantity > 0 && quantity <= 125) {
        uint16_t registers[125];
        Serial.print("Query: Modbus Read Input Regs - Slave:");
        Serial.print(slaveAddr);
        Serial.print(", Start:");
        Serial.print(startAddr);
        Serial.print(", Qty:");
        Serial.println(quantity);
        
        if (modbus->readInputRegisters(slaveAddr, startAddr, quantity, registers)) {
          Serial.println("Success!");
          
          JsonDocument responseDoc;
          JsonObject response = responseDoc.to<JsonObject>();
          response["success"] = true;
          response["slave"] = slaveAddr;
          JsonArray regArray = response["registers"].to<JsonArray>();
          
          for (uint16_t i = 0; i < quantity; i++) {
            regArray.add(registers[i]);
            Serial.print("  Reg ");
            Serial.print(startAddr + i);
            Serial.print(": ");
            Serial.println(registers[i]);
          }
          // hyper.sendMessage(msgfrom, response);
        } else {
          Serial.print("Failed! Exception: 0x");
          Serial.println(modbus->getLastException(), HEX);
        }
      }
    }

    // Modbus Write Single Register
    JsonObject modbusWriteSingle = hyper.findAction(payload, "Operate", "Modbus_Write_Single_Register");
    if (!modbusWriteSingle.isNull()) {
      uint8_t slaveAddr = modbusWriteSingle["params"]["slave_address"] | 1;
      uint16_t regAddr = modbusWriteSingle["params"]["register_address"] | 0;
      uint16_t value = modbusWriteSingle["params"]["value"] | 0;
      
      Serial.print("Command: Modbus Write Single Reg - Slave:");
      Serial.print(slaveAddr);
      Serial.print(", Reg:");
      Serial.print(regAddr);
      Serial.print(", Value:");
      Serial.println(value);
      
      if (modbus->writeSingleRegister(slaveAddr, regAddr, value)) {
        Serial.println("Success!");
        
        JsonDocument responseDoc;
        JsonObject response = responseDoc.to<JsonObject>();
        response["success"] = true;
        response["slave"] = slaveAddr;
        response["register"] = regAddr;
        response["value"] = value;
        // hyper.sendMessage(msgfrom, response);
      } else {
        Serial.print("Failed! Exception: 0x");
        Serial.println(modbus->getLastException(), HEX);
      }
    }

    // Modbus Write Multiple Registers
    JsonObject modbusWriteMultiple = hyper.findAction(payload, "Operate", "Modbus_Write_Multiple_Registers");
    if (!modbusWriteMultiple.isNull()) {
      uint8_t slaveAddr = modbusWriteMultiple["params"]["slave_address"] | 1;
      uint16_t startAddr = modbusWriteMultiple["params"]["start_address"] | 0;
      JsonArray valuesArray = modbusWriteMultiple["params"]["values"];
      
      if (!valuesArray.isNull() && valuesArray.size() > 0 && valuesArray.size() <= 123) {
        uint16_t quantity = valuesArray.size();
        uint16_t values[123];
        
        Serial.print("Command: Modbus Write Multiple Regs - Slave:");
        Serial.print(slaveAddr);
        Serial.print(", Start:");
        Serial.print(startAddr);
        Serial.print(", Qty:");
        Serial.println(quantity);
        
        // Copy values from JSON array
        for (uint16_t i = 0; i < quantity; i++) {
          values[i] = valuesArray[i].as<uint16_t>();
          Serial.print("  Value[");
          Serial.print(i);
          Serial.print("]: ");
          Serial.println(values[i]);
        }
        
        if (modbus->writeMultipleRegisters(slaveAddr, startAddr, quantity, values)) {
          Serial.println("Success!");
          
          JsonDocument responseDoc;
          JsonObject response = responseDoc.to<JsonObject>();
          response["success"] = true;
          response["slave"] = slaveAddr;
          response["start"] = startAddr;
          response["quantity"] = quantity;
          // hyper.sendMessage(msgfrom, response);
        } else {
          Serial.print("Failed! Exception: 0x");
          Serial.println(modbus->getLastException(), HEX);
        }
      }
    }

    // Modbus Write Single Coil
    JsonObject modbusWriteCoil = hyper.findAction(payload, "Operate", "Modbus_Write_Single_Coil");
    if (!modbusWriteCoil.isNull()) {
      uint8_t slaveAddr = modbusWriteCoil["params"]["slave_address"] | 1;
      uint16_t coilAddr = modbusWriteCoil["params"]["coil_address"] | 0;
      bool value = modbusWriteCoil["params"]["value"] | false;
      
      Serial.print("Command: Modbus Write Coil - Slave:");
      Serial.print(slaveAddr);
      Serial.print(", Coil:");
      Serial.print(coilAddr);
      Serial.print(", Value:");
      Serial.println(value ? "ON" : "OFF");
      
      if (modbus->writeSingleCoil(slaveAddr, coilAddr, value)) {
        Serial.println("Success!");
      } else {
        Serial.print("Failed! Exception: 0x");
        Serial.println(modbus->getLastException(), HEX);
      }
    }

    // Modbus Read Coils
    JsonObject modbusReadCoils = hyper.findAction(payload, "Query", "Modbus_Read_Coils");
    if (!modbusReadCoils.isNull()) {
      uint8_t slaveAddr = modbusReadCoils["params"]["slave_address"] | 1;
      uint16_t startAddr = modbusReadCoils["params"]["start_address"] | 0;
      uint16_t quantity = modbusReadCoils["params"]["quantity"] | 1;
      
      if (quantity > 0 && quantity <= 2000) {
        uint8_t coilData[250];  // 2000 bits / 8 = 250 bytes max
        Serial.print("Query: Modbus Read Coils - Slave:");
        Serial.print(slaveAddr);
        Serial.print(", Start:");
        Serial.print(startAddr);
        Serial.print(", Qty:");
        Serial.println(quantity);
        
        if (modbus->readCoils(slaveAddr, startAddr, quantity, coilData)) {
          Serial.println("Success!");
          
          JsonDocument responseDoc;
          JsonObject response = responseDoc.to<JsonObject>();
          response["success"] = true;
          JsonArray coilArray = response["coils"].to<JsonArray>();
          
          for (uint16_t i = 0; i < quantity; i++) {
            bool coilState = (coilData[i / 8] & (1 << (i % 8))) != 0;
            coilArray.add(coilState);
            if (i < 20) {  // Print first 20 only
              Serial.print("  Coil ");
              Serial.print(startAddr + i);
              Serial.print(": ");
              Serial.println(coilState ? "ON" : "OFF");
            }
          }
          // hyper.sendMessage(msgfrom, response);
        } else {
          Serial.print("Failed! Exception: 0x");
          Serial.println(modbus->getLastException(), HEX);
        }
      }
    }

    // I2C Write - Write bytes to external I2C device
    JsonObject i2cWrite = hyper.findAction(payload, "Operate", "I2C_Write");
    if (!i2cWrite.isNull()) {
      uint8_t address = i2cWrite["params"]["address"] | 0;
      JsonArray dataArray = i2cWrite["params"]["data"];
      
      if (address > 0 && !dataArray.isNull()) {
        Serial.print("Command: I2C Write to 0x");
        Serial.print(address, HEX);
        Serial.print(" - Data: ");
        
        Wire.beginTransmission(address);
        for (JsonVariant value : dataArray) {
          uint8_t byte = value.as<uint8_t>();
          Wire.write(byte);
          Serial.print("0x");
          Serial.print(byte, HEX);
          Serial.print(" ");
        }
        uint8_t error = Wire.endTransmission();
        Serial.println();
        
        if (error == 0) {
          Serial.println("I2C Write: Success");
        } else {
          Serial.print("I2C Write: Error ");
          Serial.println(error);
        }
      } else {
        Serial.println("Error: Invalid I2C address or data");
      }
    }

    // I2C Read - Read bytes from external I2C device
    JsonObject i2cRead = hyper.findAction(payload, "Query", "I2C_Read");
    if (!i2cRead.isNull()) {
      uint8_t address = i2cRead["params"]["address"] | 0;
      uint8_t numBytes = i2cRead["params"]["bytes"] | 1;
      
      if (address > 0 && numBytes > 0) {
        Serial.print("Query: I2C Read from 0x");
        Serial.print(address, HEX);
        Serial.print(" - ");
        Serial.print(numBytes);
        Serial.println(" bytes");
        
        Wire.requestFrom(address, numBytes);
        
        JsonDocument responseDoc;
        JsonObject response = responseDoc.to<JsonObject>();
        JsonArray dataArray = response["data"].to<JsonArray>();
        response["address"] = address;
        
        uint8_t count = 0;
        while (Wire.available() && count < numBytes) {
          uint8_t byte = Wire.read();
          dataArray.add(byte);
          Serial.print("0x");
          Serial.print(byte, HEX);
          Serial.print(" ");
          count++;
        }
        Serial.println();
        
        response["count"] = count;
        // hyper.sendMessage(msgfrom, response); // Uncomment to send response
      } else {
        Serial.println("Error: Invalid I2C address or byte count");
      }
    }

    // I2C Write-Read - Write then read from external I2C device
    JsonObject i2cWriteRead = hyper.findAction(payload, "Query", "I2C_Write_Read");
    if (!i2cWriteRead.isNull()) {
      uint8_t address = i2cWriteRead["params"]["address"] | 0;
      JsonArray writeData = i2cWriteRead["params"]["write_data"];
      uint8_t readBytes = i2cWriteRead["params"]["read_bytes"] | 1;
      
      if (address > 0 && !writeData.isNull() && readBytes > 0) {
        Serial.print("Query: I2C Write-Read to 0x");
        Serial.println(address, HEX);
        
        // Write phase
        Wire.beginTransmission(address);
        for (JsonVariant value : writeData) {
          Wire.write(value.as<uint8_t>());
        }
        uint8_t error = Wire.endTransmission();
        
        if (error == 0) {
          // Read phase
          Wire.requestFrom(address, readBytes);
          
          JsonDocument responseDoc;
          JsonObject response = responseDoc.to<JsonObject>();
          JsonArray dataArray = response["data"].to<JsonArray>();
          response["address"] = address;
          
          uint8_t count = 0;
          while (Wire.available() && count < readBytes) {
            uint8_t byte = Wire.read();
            dataArray.add(byte);
            Serial.print("0x");
            Serial.print(byte, HEX);
            Serial.print(" ");
            count++;
          }
          Serial.println();
          
          response["count"] = count;
          // hyper.sendMessage(msgfrom, response); // Uncomment to send response
        } else {
          Serial.print("I2C Write-Read: Error ");
          Serial.println(error);
        }
      } else {
        Serial.println("Error: Invalid I2C parameters");
      }
    }
  });
}

void loop() {
  // put your main code here, to run repeatedly:
  // Core functions
  hyper.loop();
  
  // Check for incoming RS485 data
  String rs485Data = board.rs485ReadLine(100); // 100ms timeout
  if (rs485Data.length() > 0) {
    Serial.print("RS485 Received: ");
    Serial.println(rs485Data);
    
    // Optionally send to Hyperwisor dashboard
    // JsonDocument doc;
    // JsonObject msg = doc.to<JsonObject>();
    // msg["rs485_data"] = rs485Data;
    // hyper.sendMessage(userId, msg);
  }
  
  delay(50);
}

// Placeholder for update_ui function - implement as needed
void update_ui() {
  Serial.println("Updating UI...");
  
  // Read all digital inputs (8 channels)
  uint8_t inputMask = board.readDigitalInputs();
  Serial.print("Digital Inputs Mask: 0b");
  Serial.println(inputMask, BIN);
  
  // Read pulse counters
  unsigned long pulse1 = board.getPulseCount(1, false);
  unsigned long pulse2 = board.getPulseCount(2, false);
  Serial.print("Pulse Counter 1: ");
  Serial.println(pulse1);
  Serial.print("Pulse Counter 2: ");
  Serial.println(pulse2);
  
  // Update widgets on Hyperwisor dashboard
  // Replace widget IDs with your actual widget IDs from Hyperwisor dashboard
  
  // Update individual digital input widgets (IN1 to IN8)
  for (uint8_t i = 1; i <= 8; i++) {
    bool inputState = (inputMask & (1 << (i - 1))) ? true : false;
    // Example: hyper.updateWidget(userId, "widget_input_1", inputState);
    // Uncomment and replace with actual widget IDs:
    // hyper.updateWidget(userId, "widget_input_" + String(i), inputState);
  }
  
  // Update pulse counter widgets
  // hyper.updateWidget(userId, "widget_pulse1", pulse1);
  // hyper.updateWidget(userId, "widget_pulse2", pulse2);
  
  // If you're tracking relay states, update relay widgets
  // Note: Board doesn't read back relay state, so you need to track it yourself
  // hyper.updateWidget(userId, "widget_relay1", relay1State);
  // hyper.updateWidget(userId, "widget_relay2", relay2State);
  
  // Example with actual widget IDs (replace these with your widget IDs):
  /*
  hyper.updateWidget(userId, "widget_1767185127950", board.readDigitalInput(1));
  hyper.updateWidget(userId, "widget_1767185198459", board.readDigitalInput(2));
  hyper.updateWidget(userId, "widget_1767185209214", pulse1);
  hyper.updateWidget(userId, "widget_1767185220315", pulse2);
  */
}
