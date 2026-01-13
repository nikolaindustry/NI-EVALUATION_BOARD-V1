// Simple Digital Input Example - Direct PCF8574 reading
#include <Wire.h>
#include <PCF8574.h>

PCF8574 inputExpander(0x3A);  // Input PCF8574 address

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== Simple Digital Input Test ===");
  
  Wire.begin(4, 5);  // SDA=4, SCL=5
  delay(10);
  
  if (!inputExpander.begin()) {
    Serial.println("PCF8574 begin() failed, but continuing...");
  }
  
  // Configure all pins as inputs
  for (uint8_t pin = 0; pin < 8; pin++) {
    inputExpander.pinMode(pin, INPUT);
  }
  
  Serial.println("Reading inputs (active-LOW: grounded = HIGH)");
  Serial.println();
}

void loop() {
  Serial.println("=== Inputs ===");
  
  // Read each input
  for (uint8_t ch = 1; ch <= 8; ch++) {
    uint8_t pin = ch - 1;
    int rawVal = inputExpander.digitalRead(pin);
    
    // Active-LOW: raw=0 means activated (show as HIGH)
    //             raw=1 means inactive (show as LOW)
    bool state = (rawVal == 0);
    
    Serial.print("IN");
    Serial.print(ch);
    Serial.print(" = ");
    Serial.print(state ? "HIGH" : "LOW");
    Serial.print(" (PCF8574 raw=");
    Serial.print(rawVal);
    Serial.println(")");
  }
  
  Serial.println("----");
  delay(1000);
}
