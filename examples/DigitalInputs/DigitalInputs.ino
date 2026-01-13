// Example: Digital input reading using NI-EVALUATION_BOARD-V1.H
//
// This example shows how to read the 8 digital inputs connected to the
// PCF8574 input expander on the evaluation board.
//
// Wiring:
//  - Use the NI Evaluation Board as designed. The inputs are already wired
//    to the PCF8574 at address 0x3A.
//
// Usage:
//  - Upload this sketch to your ESP32 on the evaluation board.
//  - Open the Serial Monitor at 115200 baud.
//  - You will see the state of IN1..IN8 printed once per second.

#include <NI-EVALUATION_BOARD-V1.h>

NIEvaluationBoardV1 board;

void setup() {
  Serial.begin(115200);
  delay(200);

  // Initialize the board with default settings
  board.begin();

  Serial.println("Digital Input Example Started");
}

void loop() {
  // Read all inputs as a bit mask
  uint8_t inputs = board.readDigitalInputs();

  Serial.print("Inputs bitmask: 0b");
  for (int i = 7; i >= 0; --i) {
    Serial.print((inputs & (1 << i)) ? '1' : '0');
  }
  Serial.println();

  // Also show each input individually
  for (uint8_t ch = 1; ch <= 8; ++ch) {
    bool state = board.readDigitalInput(ch);
    Serial.print("IN");
    Serial.print(ch);
    Serial.print(" = ");
    Serial.println(state ? "HIGH" : "LOW");
  }

  Serial.println("----");
  delay(1000);
}
