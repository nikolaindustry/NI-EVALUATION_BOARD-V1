// Example: PWM output control using NI-EVALUATION_BOARD-V1.H
//
// This example shows how to generate PWM signals on the two gate driver
// outputs (GPIO 32 and 33) using the built-in PWM helpers. It also
// demonstrates how to override the default PWM frequency and resolution.
//
// Usage:
//  - Upload this sketch to your ESP32 on the evaluation board.
//  - Observe the motor/driver outputs connected to PIN_PWM1 and PIN_PWM2.

#include <NI-EVALUATION_BOARD-V1.h>

NIEvaluationBoardV1 board;

void setup() {
  Serial.begin(115200);
  delay(200);

  // Initialize board with custom PWM settings:
  //  - RS485: 115200 baud
  //  - PWM: 20 kHz, 8-bit resolution
  board.begin(115200, 20000, 8);

  Serial.println("PWM Output Example Started");
}

void loop() {
  static int duty = 0;
  static int step = 5;

  // Simple ramp up/down between 0 and 255 on both PWM channels
  board.setPwmDuty(1, duty);
  board.setPwmDuty(2, duty);

  duty += step;
  if (duty >= 255) {
    duty = 255;
    step = -step;
    Serial.println("Reached max duty");
  } else if (duty <= 0) {
    duty = 0;
    step = -step;
    Serial.println("Reached min duty");
  }

  delay(20); // adjust ramp speed
}
