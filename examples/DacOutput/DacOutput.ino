// Example: DAC 0–10 V output using NI-EVALUATION_BOARD-V1.H
//
// This example shows how to generate 0–10 V analog outputs using the
// onboard GP8403 DAC. Channel 0 is ramped from 0 V to 10 V and back.
//
// Usage:
//  - Upload this sketch to your ESP32 on the evaluation board.
//  - Monitor the analog output on the DAC channel 0 with a voltmeter.

#include <NI-EVALUATION_BOARD-V1.h>

NIEvaluationBoardV1 board;

void setup() {
  Serial.begin(115200);
  delay(200);

  board.begin();

  Serial.println("DAC Output Example Started");
}

void loop() {
  // Ramp up from 0 V to 10 V
  Serial.println("Ramping up from 0 V to 10 V");
  for (int i = 0; i <= 100; ++i) {
    float voltage = (10.0 * i) / 100.0;
    board.setAnalogVoltage(0, voltage); // channel 0
    delay(50);
  }

  delay(1000);

  // Ramp down from 10 V to 0 V
  Serial.println("Ramping down from 10 V to 0 V");
  for (int i = 100; i >= 0; --i) {
    float voltage = (10.0 * i) / 100.0;
    board.setAnalogVoltage(0, voltage); // channel 0
    delay(50);
  }

  delay(1000);
}
