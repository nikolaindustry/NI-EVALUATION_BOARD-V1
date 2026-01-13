// Example: Pulse counting using NI-EVALUATION_BOARD-V1.H
//
// This example shows how to use the pulse counter inputs connected to
// GPIO 34 and 35. The library attaches interrupts that increment counters
// on each falling edge.
//
// Usage:
//  - Upload this sketch to your ESP32 on the evaluation board.
//  - Apply pulses to the pulse input terminals.
//  - Open the Serial Monitor at 115200 baud to see the counts.

#include <NI-EVALUATION_BOARD-V1.h>

NIEvaluationBoardV1 board;

void setup() {
  Serial.begin(115200);
  delay(200);

  board.begin();

  Serial.println("Pulse Counting Example Started");
}

void loop() {
  static unsigned long lastPrint = 0;
  unsigned long now = millis();

  if (now - lastPrint >= 1000) {
    lastPrint = now;

    // Read pulse counts without resetting
    unsigned long count1 = board.getPulseCount(1);
    unsigned long count2 = board.getPulseCount(2);

    Serial.print("Pulse1: ");
    Serial.print(count1);
    Serial.print("\t Pulse2: ");
    Serial.println(count2);
  }
}
