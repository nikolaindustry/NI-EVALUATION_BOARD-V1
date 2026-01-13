// I2C Scanner for NI Evaluation Board
// This will scan the I2C bus and report all devices found

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\nI2C Scanner for NI Evaluation Board");
  Serial.println("====================================");
  
  // Initialize I2C with the pins defined in your library
  // SDA = GPIO 4, SCL = GPIO 5
  Wire.begin(4, 5);
  
  Serial.println("Scanning I2C bus...\n");
  
  byte error, address;
  int deviceCount = 0;
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.print(" (");
      Serial.print(address);
      Serial.println(")");
      
      // Identify known devices
      if (address == 0x3A) Serial.println("  -> Should be: Input PCF8574");
      if (address == 0x3C) Serial.println("  -> Should be: Relay PCF8574");
      if (address == 0x58) Serial.println("  -> Should be: GP8403 DAC");
      
      deviceCount++;
    }
  }
  
  Serial.println("\n====================================");
  if (deviceCount == 0) {
    Serial.println("ERROR: No I2C devices found!");
    Serial.println("\nPossible issues:");
    Serial.println("  1. Check I2C wiring (SDA=GPIO4, SCL=GPIO5)");
    Serial.println("  2. Check if PCF8574 chips are powered");
    Serial.println("  3. Verify pull-up resistors on SDA/SCL");
    Serial.println("  4. Check if devices are soldered properly");
  } else {
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.println(" device(s)");
  }
  Serial.println("====================================\n");
}

void loop() {
  // Rescan every 5 seconds
  delay(5000);
  Serial.println("\nRescanning...");
  setup();
}
