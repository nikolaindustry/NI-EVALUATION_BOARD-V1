// Direct PCF8574 Input Test - tests raw input reading
#include <Wire.h>
#include <PCF8574.h>

PCF8574 inputExpander(0x3A);  // Input PCF8574 address

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== Direct PCF8574 Input Test ===");
  
  Wire.begin(4, 5);  // SDA=4, SCL=5
  delay(10);
  
  if (!inputExpander.begin()) {
    Serial.println("PCF8574 begin() failed, but continuing anyway...");
  }
  
  // Configure all pins as inputs
  for (uint8_t pin = 0; pin < 8; pin++) {
    inputExpander.pinMode(pin, INPUT);
  }
  
  Serial.println("Reading all 8 PCF8574 input pins...");
  Serial.println("Connect inputs to GND to test (if active-low)");
  Serial.println("or to VCC to test (if active-high)\n");
}

void loop() {
  Serial.println("=== Reading Inputs ===");
  
  // Read each pin individually
  for (uint8_t pin = 0; pin < 8; pin++) {
    int rawValue = inputExpander.digitalRead(pin);
    Serial.print("P");
    Serial.print(pin);
    Serial.print(" (IN");
    Serial.print(pin + 1);
    Serial.print("): ");
    Serial.print(rawValue ? "HIGH (1)" : "LOW (0)");
    Serial.print(" - Raw value: ");
    Serial.println(rawValue);
  }
  
  Serial.println("----");
  delay(1000);
}
