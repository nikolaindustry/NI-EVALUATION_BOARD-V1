// Example: RS485 communication using NI-EVALUATION_BOARD-V1.H
//
// This example shows how to send a command over RS485 and wait for a
// response using the built-in RS485 helper functions.
//
// Usage:
//  - Connect the RS485 lines of the master (this board) and a slave device.
//  - The slave should echo or respond to simple text commands (e.g. "LED_ON").
//  - Upload this sketch and open the Serial Monitor at 115200 baud.

#include <NI-EVALUATION_BOARD-V1.h>

NIEvaluationBoardV1 board;

void setup() {
  Serial.begin(115200);
  delay(200);

  // Initialize board and RS485 at 115200 baud
  board.begin(115200);

  Serial.println("RS485 Communication Example Started");
}

void loop() {
  const char *command = "LED_ON";

  Serial.print("Sending command over RS485: ");
  Serial.println(command);

  // Send command with newline terminator
  board.rs485WriteLine(command);

  // Wait for response (up to 1 second)
  String response = board.rs485ReadLine(1000);

  if (response.length() > 0) {
    Serial.print("Received: ");
    Serial.println(response);
  } else {
    Serial.println("No response (timeout)");
  }

  // Send every 5 seconds
  delay(5000);
}
