// Example: Push button events using NI-EVALUATION_BOARD-V1.H
//
// This example shows how to use the built-in button event handling.
// The library detects single click, double click, triple click, and long press events
// on the button connected to GPIO 16.
//
// Usage:
//  - Upload this sketch to your ESP32 on the evaluation board.
//  - Open the Serial Monitor at 115200 baud.
//  - Press the onboard button and observe the printed events.

#include <NI-EVALUATION_BOARD-V1.h>

NIEvaluationBoardV1 board;

void setup() {
  Serial.begin(115200);
  delay(200);

  board.begin();

  Serial.println("Push Button Events Example Started");
}

void loop() {
  // Call updateButton() frequently in loop() to detect events
  NIButtonEvent evt = board.updateButton();

  if (evt == NI_BUTTON_SINGLE) {
    Serial.println("Button: SINGLE CLICK");
  } else if (evt == NI_BUTTON_DOUBLE) {
    Serial.println("Button: DOUBLE CLICK");
  } else if (evt == NI_BUTTON_TRIPLE) {
    Serial.println("Button: TRIPLE CLICK");
  } else if (evt == NI_BUTTON_LONG) {
    Serial.println("Button: LONG PRESS");
  }

  // You can also trigger actions here, e.g. toggle relays:
  if (evt == NI_BUTTON_SINGLE) board.setRelay(1, true);
  if (evt == NI_BUTTON_DOUBLE) board.setRelay(2, true);
  if (evt == NI_BUTTON_TRIPLE) board.setAllRelays(false);

  delay(10); // keep loop responsive
}
