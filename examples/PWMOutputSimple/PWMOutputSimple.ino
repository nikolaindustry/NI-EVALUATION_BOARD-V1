// Simple PWM Output Example
//
// This example demonstrates basic PWM control on the evaluation board.
// It sets a fixed PWM duty cycle on both PWM channels.

#include <NI-EVALUATION_BOARD-V1.h>

NIEvaluationBoardV1 board;

void setup() {
  Serial.begin(115200);
  delay(200);

  // Initialize board with default settings
  board.begin();

  Serial.println("Simple PWM Output Example");
  
  // Set PWM1 to 50% duty cycle (127 out of 255)
  board.setPwmDuty(1, 127);
  
  // Set PWM2 to 75% duty cycle (191 out of 255)
  board.setPwmDuty(2, 191);
  
  Serial.println("PWM1: 50% duty");
  Serial.println("PWM2: 75% duty");
}

void loop() {
  // Nothing to do - PWM runs continuously
  delay(1000);
}
