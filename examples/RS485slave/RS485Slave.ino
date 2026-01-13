// Example: RS485 Slave - Receive and respond to commands
//
// This example demonstrates a slave device that listens for commands
// over RS485 and sends responses back to the master.
//
// Usage:
//  - Connect the RS485 lines between master and slave boards.
//  - Upload this sketch to the slave board.
//  - Upload RS485Communication.ino to the master board.
//  - Open Serial Monitor on the slave at 115200 baud to see received commands.
//
// This slave responds to:
//  - "LED_ON"  -> turns on status LED and responds "LED_ON_OK"
//  - "LED_OFF" -> turns off status LED and responds "LED_OFF_OK"
//  - "RELAY1_ON"  -> turns on relay 1 and responds "RELAY1_ON_OK"
//  - "RELAY1_OFF" -> turns off relay 1 and responds "RELAY1_OFF_OK"
//  - "STATUS"  -> responds with current status
//  - Any other command -> responds "UNKNOWN_COMMAND"

#include <NI-EVALUATION_BOARD-V1.h>

NIEvaluationBoardV1 board;

void setup() {
  Serial.begin(115200);
  delay(200);

  // Initialize board and RS485 at 115200 baud
  board.begin(115200);

  Serial.println("RS485 Slave Ready");
  Serial.println("Waiting for commands...");
}

void loop() {
  // Check if there's data available on RS485
  String command = board.rs485ReadLine(100);  // 100ms timeout
  
  if (command.length() > 0) {
    // Remove any trailing whitespace
    command.trim();
    
    Serial.print("Received command: ");
    Serial.println(command);
    
    String response = processCommand(command);
    
    // Send response back to master
    board.rs485WriteLine(response);
    
    Serial.print("Sent response: ");
    Serial.println(response);
  }
  
  // Small delay to avoid overwhelming the processor
  delay(10);
}

// Process received command and return appropriate response
String processCommand(String cmd) {
  if (cmd == "LED_ON") {
    digitalWrite(NIEvaluationBoardV1::PIN_STATUS_LED, HIGH);
    return "LED_ON_OK";
  }
  else if (cmd == "LED_OFF") {
    digitalWrite(NIEvaluationBoardV1::PIN_STATUS_LED, LOW);
    return "LED_OFF_OK";
  }
  else if (cmd == "RELAY1_ON") {
    board.setRelay(1, true);
    return "RELAY1_ON_OK";
  }
  else if (cmd == "RELAY1_OFF") {
    board.setRelay(1, false);
    return "RELAY1_OFF_OK";
  }
  else if (cmd == "RELAY2_ON") {
    board.setRelay(2, true);
    return "RELAY2_ON_OK";
  }
  else if (cmd == "RELAY2_OFF") {
    board.setRelay(2, false);
    return "RELAY2_OFF_OK";
  }
  else if (cmd == "STATUS") {
    return "SLAVE_OK";
  }
  else if (cmd.startsWith("READ_INPUT")) {
    // Extract channel number (e.g., "READ_INPUT1" -> channel 1)
    int channel = cmd.substring(10).toInt();
    if (channel >= 1 && channel <= 8) {
      bool state = board.readDigitalInput(channel);
      return "INPUT" + String(channel) + "_" + (state ? "HIGH" : "LOW");
    }
    return "INVALID_CHANNEL";
  }
  else {
    return "UNKNOWN_COMMAND";
  }
}
