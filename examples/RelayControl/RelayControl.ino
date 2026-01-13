// Example: Relay control using NI-EVALUATION_BOARD-V1
//
// This example shows how to control the relay outputs connected to the
// PCF8574 relay expander on the evaluation board.
//
// Wiring:
//  - Use the NI Evaluation Board as designed. The relays are already wired
//    to the PCF8574 at address 0x3C.
//
// Usage:
//  - Upload this sketch to your ESP32 on the evaluation board.
//  - Open the Serial Monitor at 115200 baud.
//  - Relays 1 and 2 will toggle on and off every second.
//
// Troubleshooting:
//  - If relays don't respond: Run the I2CScanner example to verify I2C devices
//  - If relays behave inverted: Run the RelayTest example to check pin mapping
//  - Library works even if PCF8574 begin() fails - this is normal

#include <NI-EVALUATION_BOARD-V1.h>

NIEvaluationBoardV1 board;

void setup() {
  Serial.begin(115200);
  delay(200);

  board.begin();

  Serial.println("Relay Control Example Started");
}

void loop() {
  // Turn both relays ON
  Serial.println("Turning both relays ON");
  board.setAllRelays(true);
  delay(1000);

  // Turn both relays OFF
  Serial.println("Turning both relays OFF");
  board.setAllRelays(false);
  delay(1000);

  // Demonstrate individual control
  Serial.println("Relay 1 ON, Relay 2 OFF");
  board.setRelay(1, true);
  board.setRelay(2, false);
  delay(1000);

  Serial.println("Relay 1 OFF, Relay 2 ON");
  board.setRelay(1, false);
  board.setRelay(2, true);
  delay(1000);
}
