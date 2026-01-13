// Simple DAC example: fixed 5 V on channel 0
//
// Upload this sketch to your ESP32 on the NI Evaluation Board.
// Measure DAC channel 0 with a voltmeter: it should read about 5 V.

#include <NI-EVALUATION_BOARD-V1.h>

NIEvaluationBoardV1 board;

void setup() {
  Serial.begin(115200);
  delay(200);

  board.begin();

  // Set channel 0 to 5.0 V (range 0.0–10.0 V)
  board.setAnalogVoltage(0, 5.0f);
  Serial.println("DAC channel 0 set to 5.0 V");
}

void loop() {
  // Output stays at 5.0 V; nothing else to do here
}
