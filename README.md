# NI-EVALUATION_BOARD-V1 Library

[![Arduino Library](https://img.shields.io/badge/Arduino-Library-blue.svg)](https://www.arduino.cc/reference/en/libraries/)
[![ESP32](https://img.shields.io/badge/Platform-ESP32-green.svg)](https://www.espressif.com/en/products/socs/esp32)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**Comprehensive Arduino library for the NI Evaluation Board V1 - Industrial-grade I/O control for ESP32**

This library provides a high-level, unified API for all hardware features of the NI Evaluation Board V1, including digital inputs, relay outputs, 0-10V DAC outputs, pulse counting, RS485 communication, push button events, and PWM control.

---

## 📋 Table of Contents

- [Features](#-features)
- [Hardware Specifications](#-hardware-specifications)
- [Installation](#-installation)
- [Quick Start](#-quick-start)
- [API Reference](#-api-reference)
  - [Initialization](#initialization)
  - [Digital Inputs](#digital-inputs)
  - [Relay Outputs](#relay-outputs)
  - [DAC 0-10V Outputs](#dac-0-10v-outputs)
  - [Pulse Counting](#pulse-counting)
  - [RS485 Communication](#rs485-communication)
  - [Push Button Events](#push-button-events)
  - [PWM Outputs](#pwm-outputs)
- [Examples](#-examples)
- [Advanced Usage](#-advanced-usage)
- [Troubleshooting](#-troubleshooting)
- [Contributing](#-contributing)
- [License](#-license)

---

## 🚀 Features

- **8 Digital Inputs** - Optically isolated inputs via PCF8574 (I2C: 0x3A)
- **2 Relay Outputs** - High-current relay control via PCF8574 (I2C: 0x3C)
- **2 DAC Outputs** - 0-10V analog output with GP8403 12-bit DAC (I2C: 0x58)
- **2 Pulse Counters** - Hardware interrupt-based pulse counting (GPIO 34, 35)
- **RS485 Communication** - Half-duplex serial communication with line drivers
- **Push Button** - Advanced event detection (single, double, triple, long press)
- **2 PWM Outputs** - Configurable frequency and resolution (GPIO 32, 33)
- **Status LED** - Onboard indicator (GPIO 2)
- **Easy-to-use API** - Single object controls all board features
- **Interrupt-driven** - Efficient pulse counting without polling
- **Non-blocking** - Button and RS485 operations don't block execution

---

## 🔧 Hardware Specifications

### Board Components

| Component | Model/Type | I2C Address | GPIO Pins |
|-----------|-----------|-------------|-----------|
| **Digital Input Expander** | PCF8574 | 0x3A | - |
| **Relay Expander** | PCF8574 | 0x3C | - |
| **DAC** | GP8403 | 0x58 | - |
| **I2C Bus** | - | - | SDA: 4, SCL: 5 |
| **Pulse Inputs** | - | - | 34, 35 |
| **Push Button** | - | - | 16 (INPUT_PULLUP) |
| **PWM Outputs** | LEDC | - | 32, 33 |
| **Status LED** | - | - | 2 |
| **RS485** | Serial2 | - | RX: 14, TX: 13 |

### Pin Mapping

```
Digital Inputs (via PCF8574 @ 0x3A):
  IN1-IN8 → P0-P7

Relay Outputs (via PCF8574 @ 0x3C):
  RELAY1 → P3
  RELAY2 → P2

DAC Outputs (via GP8403 @ 0x58):
  DAC_CH0 → 0-10V analog output
  DAC_CH1 → 0-10V analog output

Direct ESP32 Connections:
  GPIO 34 → PULSE1 (input only)
  GPIO 35 → PULSE2 (input only)
  GPIO 16 → Push Button (with internal pullup)
  GPIO 32 → PWM1 output
  GPIO 33 → PWM2 output
  GPIO  2 → Status LED
  GPIO 14 → RS485 RXD
  GPIO 13 → RS485 TXD
  GPIO  4 → I2C SDA
  GPIO  5 → I2C SCL
```

---

## 📦 Installation

### Method 1: Arduino Library Manager (Recommended)

1. Open the Arduino IDE
2. Go to **Sketch > Include Library > Manage Libraries...**
3. Search for **"NI-EVALUATION_BOARD-V1"**
4. Click **Install**

### Method 2: Manual Installation

1. Download the latest release from [GitHub](https://github.com/nikolaindustry/NI-EVALUATION_BOARD-V1)
2. Extract the ZIP file
3. Move the folder to your Arduino libraries directory:
   - **Windows**: `Documents\Arduino\libraries\`
   - **macOS**: `~/Documents/Arduino/libraries/`
   - **Linux**: `~/Arduino/libraries/`
4. Restart Arduino IDE

### Method 3: Git Clone

```bash
cd ~/Documents/Arduino/libraries/
git clone https://github.com/nikolaindustry/NI-EVALUATION_BOARD-V1.git
```

### Dependencies

This library requires:
- **PCF8574 library** by Renzo Mischianti (install via Library Manager)
- **ESP32 Arduino Core** (install via Boards Manager)

---

## ⚡ Quick Start

### Basic Example

```cpp
#include <NI-EVALUATION_BOARD-V1.h>

NIEvaluationBoardV1 board;

void setup() {
  Serial.begin(115200);
  
  // Initialize the board with default settings
  if (board.begin()) {
    Serial.println("Board initialized successfully!");
  } else {
    Serial.println("Board initialization failed!");
  }
}

void loop() {
  // Read digital input 1
  bool input1 = board.readDigitalInput(1);
  
  // Control relay 1 based on input
  board.setRelay(1, input1);
  
  // Set DAC output to 5.0V
  board.setAnalogVoltage(0, 5.0);
  
  // Read pulse count
  unsigned long pulses = board.getPulseCount(1);
  Serial.println(pulses);
  
  delay(100);
}
```

---

## 📖 API Reference

### Initialization

#### `begin()`

Initializes all board hardware components.

```cpp
bool begin(uint32_t rs485Baud = 115200,
           uint32_t pwmFreq = 5000,
           uint8_t pwmResolution = 8);
```

**Parameters:**
- `rs485Baud` - RS485 baud rate (default: 115200)
- `pwmFreq` - PWM frequency in Hz (default: 5000)
- `pwmResolution` - PWM resolution in bits (default: 8, range: 0-255)

**Returns:** `true` if initialization successful, `false` otherwise

**Example:**
```cpp
// Default initialization
board.begin();

// Custom RS485 baud rate
board.begin(9600);

// Custom PWM settings: 10 kHz, 10-bit resolution (0-1023)
board.begin(115200, 10000, 10);
```

---

### Digital Inputs

#### `readDigitalInput()`

Reads a single digital input channel.

```cpp
bool readDigitalInput(uint8_t channel);
```

**Parameters:**
- `channel` - Input channel number (1-8)

**Returns:** `true` if input is HIGH (active), `false` if LOW

**Example:**
```cpp
if (board.readDigitalInput(1)) {
  Serial.println("Input 1 is active");
}
```

#### `readDigitalInputs()`

Reads all 8 digital inputs as a bitmask.

```cpp
uint8_t readDigitalInputs();
```

**Returns:** 8-bit mask where bit 0 = IN1, bit 7 = IN8

**Example:**
```cpp
uint8_t inputs = board.readDigitalInputs();

// Check individual inputs using bitmask
if (inputs & 0x01) Serial.println("IN1 active");
if (inputs & 0x02) Serial.println("IN2 active");
if (inputs & 0x04) Serial.println("IN3 active");
// ... etc
```

---

### Relay Outputs

#### `setRelay()`

Controls a single relay channel.

```cpp
void setRelay(uint8_t channel, bool on);
```

**Parameters:**
- `channel` - Relay channel (1 or 2)
- `on` - `true` to energize relay, `false` to de-energize

**Example:**
```cpp
board.setRelay(1, true);   // Turn relay 1 ON
board.setRelay(2, false);  // Turn relay 2 OFF
```

#### `setAllRelays()`

Controls both relays simultaneously.

```cpp
void setAllRelays(bool on);
```

**Parameters:**
- `on` - `true` to energize all relays, `false` to de-energize

**Example:**
```cpp
board.setAllRelays(true);   // Turn both relays ON
board.setAllRelays(false);  // Turn both relays OFF
```

---

### DAC 0-10V Outputs

#### `setAnalogVoltage()`

Sets DAC output voltage (0-10V range).

```cpp
void setAnalogVoltage(uint8_t channel, float voltage);
```

**Parameters:**
- `channel` - DAC channel (0 or 1)
- `voltage` - Output voltage (0.0 to 10.0V, automatically clamped)

**Example:**
```cpp
board.setAnalogVoltage(0, 5.0);   // Set channel 0 to 5.0V
board.setAnalogVoltage(1, 3.3);   // Set channel 1 to 3.3V
```

#### `setAnalogRaw()`

Sets DAC output using raw 12-bit value.

```cpp
void setAnalogRaw(uint8_t channel, uint16_t value);
```

**Parameters:**
- `channel` - DAC channel (0 or 1)
- `value` - Raw DAC value (0-4095, where 4095 = 10V)

**Example:**
```cpp
board.setAnalogRaw(0, 2048);  // Set to approximately 5V (mid-scale)
board.setAnalogRaw(1, 4095);  // Set to 10V (full-scale)
```

---

### Pulse Counting

#### `getPulseCount()`

Retrieves the pulse count from a counter channel.

```cpp
unsigned long getPulseCount(uint8_t channel, bool resetAfterRead = false);
```

**Parameters:**
- `channel` - Pulse counter channel (1 or 2)
- `resetAfterRead` - If `true`, resets counter to 0 after reading

**Returns:** Current pulse count

**Example:**
```cpp
// Read without reset
unsigned long count1 = board.getPulseCount(1);
Serial.println(count1);

// Read and reset
unsigned long count2 = board.getPulseCount(2, true);
Serial.println(count2);
// count2 is now 0
```

**Technical Details:**
- Interrupt-driven counting on FALLING edge
- Counts stored in volatile variables for ISR safety
- Atomic read operations with interrupt protection
- Maximum count rate depends on pulse frequency and ESP32 performance

---

### RS485 Communication

#### `rs485Write()`

Sends data over RS485 (no line termination).

```cpp
size_t rs485Write(const String &data);
```

**Parameters:**
- `data` - String to transmit

**Returns:** Number of bytes written

**Example:**
```cpp
board.rs485Write("HELLO");
```

#### `rs485WriteLine()`

Sends a line over RS485 (appends newline `\n`).

```cpp
size_t rs485WriteLine(const String &line);
```

**Parameters:**
- `line` - String to transmit

**Returns:** Number of bytes written

**Example:**
```cpp
board.rs485WriteLine("SENSOR:READ");
```

#### `rs485ReadLine()`

Reads a line from RS485 (blocking until newline or timeout).

```cpp
String rs485ReadLine(uint32_t timeoutMs = 1000);
```

**Parameters:**
- `timeoutMs` - Timeout in milliseconds (default: 1000ms)

**Returns:** Received string (trimmed), or empty string on timeout

**Example:**
```cpp
String response = board.rs485ReadLine(2000);
if (response.length() > 0) {
  Serial.println("Received: " + response);
}
```

---

### Push Button Events

#### `updateButton()`

Updates button state and returns detected event. **Call this in `loop()`**.

```cpp
NIButtonEvent updateButton();
```

**Returns:** Button event type (enum)

**Event Types:**
- `NI_BUTTON_NONE` - No event
- `NI_BUTTON_SINGLE` - Single click detected
- `NI_BUTTON_DOUBLE` - Double click detected
- `NI_BUTTON_TRIPLE` - Triple click detected
- `NI_BUTTON_LONG` - Long press detected (>1500ms)

**Example:**
```cpp
void loop() {
  NIButtonEvent event = board.updateButton();
  
  switch (event) {
    case NI_BUTTON_SINGLE:
      Serial.println("Single click!");
      break;
    case NI_BUTTON_DOUBLE:
      Serial.println("Double click!");
      break;
    case NI_BUTTON_TRIPLE:
      Serial.println("Triple click!");
      break;
    case NI_BUTTON_LONG:
      Serial.println("Long press!");
      break;
    default:
      break;
  }
}
```

**Timing Configuration:**
- Long press threshold: 1500ms
- Multi-click gap: 400ms

---

### PWM Outputs

#### `setPwmDuty()`

Sets PWM duty cycle.

```cpp
void setPwmDuty(uint8_t channel, uint8_t duty);
```

**Parameters:**
- `channel` - PWM channel (1 or 2)
- `duty` - Duty cycle (0-255 for 8-bit, adjust for other resolutions)

**Example:**
```cpp
// 8-bit resolution (0-255)
board.setPwmDuty(1, 128);  // 50% duty cycle
board.setPwmDuty(2, 255);  // 100% duty cycle

// 10-bit resolution (0-1023) - if configured in begin()
board.setPwmDuty(1, 512);  // 50% duty cycle
```

#### `configurePwm()`

Reconfigures PWM frequency and resolution after `begin()`.

```cpp
void configurePwm(uint32_t pwmFreq, uint8_t pwmResolution);
```

**Parameters:**
- `pwmFreq` - PWM frequency in Hz
- `pwmResolution` - Resolution in bits (1-16)

**Example:**
```cpp
// Change to 20 kHz, 12-bit resolution
board.configurePwm(20000, 12);
board.setPwmDuty(1, 2048);  // 50% of 4096 (12-bit)
```

---

## 💡 Examples

The library includes 13 comprehensive examples:

### Basic Examples
- **[DacOutputSimple](examples/DacOutputSimple)** - Simple DAC voltage output
- **[DigitalInputsSimple](examples/DigitalInputsSimple)** - Read digital inputs
- **[PWMOutputSimple](examples/PWMOutputSimple)** - Basic PWM control

### Advanced Examples
- **[DacOutput](examples/DacOutput)** - Voltage ramping and waveform generation
- **[DigitalInputs](examples/DigitalInputs)** - All inputs monitoring
- **[PWMOutput](examples/PWMOutput)** - PWM with frequency control
- **[RelayControl](examples/RelayControl)** - Relay switching patterns
- **[PulseCounting](examples/PulseCounting)** - Interrupt-based pulse counter
- **[PushButtonEvents](examples/PushButtonEvents)** - Button event detection
- **[RS485master](examples/RS485master)** - RS485 master communication
- **[RS485slave](examples/RS485slave)** - RS485 slave communication
- **[InputTest](examples/InputTest)** - Complete input testing
- **[I2CScanner](examples/I2CScanner)** - Scan I2C bus for devices

### Load Examples in Arduino IDE

1. Go to **File > Examples > NI-EVALUATION_BOARD-V1**
2. Select desired example
3. Upload to your board

---

## 🎯 Advanced Usage

### Complete Industrial Control System

```cpp
#include <NI-EVALUATION_BOARD-V1.h>

NIEvaluationBoardV1 board;

void setup() {
  Serial.begin(115200);
  
  // Initialize with custom settings
  board.begin(9600, 10000, 10);  // RS485: 9600 baud, PWM: 10kHz, 10-bit
  
  Serial.println("Industrial Control System Started");
}

void loop() {
  // Read all digital inputs
  uint8_t inputs = board.readDigitalInputs();
  
  // Control relays based on inputs
  board.setRelay(1, inputs & 0x01);  // Relay 1 follows Input 1
  board.setRelay(2, inputs & 0x02);  // Relay 2 follows Input 2
  
  // Analog output proportional to pulse frequency
  static unsigned long lastPulseCount = 0;
  static unsigned long lastTime = millis();
  
  unsigned long currentPulses = board.getPulseCount(1, true);
  unsigned long currentTime = millis();
  
  if (currentTime - lastTime >= 1000) {
    // Calculate pulses per second
    float frequency = currentPulses;
    
    // Output 0-10V proportional to 0-100 Hz
    float voltage = constrain(frequency / 10.0, 0, 10);
    board.setAnalogVoltage(0, voltage);
    
    lastTime = currentTime;
  }
  
  // PWM output based on analog calculation
  int pwmValue = map(analogRead(36), 0, 4095, 0, 1023);
  board.setPwmDuty(1, pwmValue);
  
  // Button control for system modes
  NIButtonEvent btnEvent = board.updateButton();
  if (btnEvent == NI_BUTTON_SINGLE) {
    Serial.println("Mode 1 activated");
  } else if (btnEvent == NI_BUTTON_DOUBLE) {
    Serial.println("Mode 2 activated");
  } else if (btnEvent == NI_BUTTON_LONG) {
    Serial.println("System reset");
  }
  
  // RS485 communication
  if (Serial2.available()) {
    String command = board.rs485ReadLine(100);
    if (command == "STATUS") {
      board.rs485WriteLine("OK");
    }
  }
  
  delay(10);
}
```

### Modbus-like Protocol Implementation

```cpp
void handleModbusCommand(String cmd) {
  // Parse command: "SET:RELAY:1:ON"
  if (cmd.startsWith("SET:RELAY:")) {
    int channel = cmd.substring(10, 11).toInt();
    bool state = cmd.endsWith("ON");
    board.setRelay(channel, state);
    board.rs485WriteLine("ACK");
  }
  else if (cmd.startsWith("GET:INPUT:")) {
    int channel = cmd.substring(10, 11).toInt();
    bool state = board.readDigitalInput(channel);
    board.rs485WriteLine(state ? "HIGH" : "LOW");
  }
  else if (cmd.startsWith("SET:DAC:")) {
    int channel = cmd.substring(8, 9).toInt();
    float voltage = cmd.substring(10).toFloat();
    board.setAnalogVoltage(channel, voltage);
    board.rs485WriteLine("ACK");
  }
}
```

---

## 🔍 Troubleshooting

### I2C Device Not Found

**Problem:** Board initialization fails or devices not responding

**Solutions:**
1. Run the **I2CScanner** example to verify I2C devices
2. Check I2C addresses:
   - Digital inputs: 0x3A
   - Relays: 0x3C
   - DAC: 0x58
3. Verify SDA (GPIO 4) and SCL (GPIO 5) connections
4. Add 4.7kΩ pull-up resistors to SDA/SCL if using long cables

### Relays Not Switching

**Problem:** Relays don't respond to commands

**Solutions:**
1. Verify PCF8574 is detected at 0x3C
2. Check relay logic (LOW = ON, HIGH = OFF)
3. Ensure adequate power supply for relay coils
4. Test with `setAllRelays(true)` for both relays

### DAC Output Incorrect

**Problem:** DAC voltage not matching setpoint

**Solutions:**
1. Verify GP8403 at I2C address 0x58
2. Check voltage range configuration (0-10V mode)
3. Measure with calibrated voltmeter
4. Test with known values: `setAnalogVoltage(0, 5.0)`

### Pulse Counter Not Counting

**Problem:** getPulseCount() always returns 0

**Solutions:**
1. Verify pulse signal connects to GPIO 34 or 35
2. Check pulse signal voltage (3.3V logic level)
3. Ensure falling-edge triggers are present
4. Test with slow manual button presses first

### RS485 Communication Fails

**Problem:** No data received or garbled transmission

**Solutions:**
1. Verify RX/TX pins: GPIO 14 (RX), GPIO 13 (TX)
2. Match baud rates on both master and slave
3. Check RS485 bus termination (120Ω resistors)
4. Ensure common ground between devices
5. Verify A/B polarity on RS485 connections

### Button Events Not Detected

**Problem:** updateButton() doesn't return events

**Solutions:**
1. Call `updateButton()` regularly in `loop()` (not in ISR)
2. Verify button connects to GPIO 16
3. Check button is normally open (NO) type
4. Don't use delay() that blocks button polling
5. Adjust timing constants if needed

### PWM Not Working

**Problem:** No PWM output on GPIO 32/33

**Solutions:**
1. Verify LEDC peripheral is available (ESP32 only)
2. Check PWM frequency and resolution are valid
3. Ensure duty cycle matches resolution (0-255 for 8-bit)
4. Test with `setPwmDuty(1, 128)` for 50% duty cycle

### Compilation Errors

**Problem:** Code won't compile

**Solutions:**
1. Install **PCF8574** library via Library Manager
2. Select **ESP32** board in Tools > Board menu
3. Update ESP32 Arduino Core to latest version
4. Check `#include <NI-EVALUATION_BOARD-V1.h>` spelling

---

## 🤝 Contributing

We welcome contributions! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### Development Guidelines

- Follow existing code style and formatting
- Add examples for new features
- Update documentation and README
- Test on actual hardware before submitting
- Include comments for complex logic

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 📞 Support

- **Website:** [https://NIKOLAINDUSTRY.com](https://NIKOLAINDUSTRY.com)
- **Email:** support@example.com
- **GitHub Issues:** [Report a bug](https://github.com/nikolaindustry/NI-EVALUATION_BOARD-V1/issues)
- **GitHub Discussions:** [Ask questions](https://github.com/nikolaindustry/NI-EVALUATION_BOARD-V1/discussions)

---

## 🏆 Credits

Developed and maintained by **NIKOLAINDUSTRY**

Special thanks to:
- Renzo Mischianti for the PCF8574 library
- Espressif Systems for ESP32 Arduino Core
- Arduino community for continuous support

---

## 📊 Board Schematic Reference

```
┌─────────────────────────────────────────────────────────────┐
│                    NI EVALUATION BOARD V1                   │
│                                                             │
│  ┌──────────┐     ┌──────────┐     ┌──────────┐           │
│  │ PCF8574  │     │ PCF8574  │     │ GP8403   │           │
│  │  0x3A    │     │  0x3C    │     │  0x58    │           │
│  │ (INPUTS) │     │ (RELAYS) │     │  (DAC)   │           │
│  └────┬─────┘     └────┬─────┘     └────┬─────┘           │
│       │                │                │                  │
│       └────────────────┴────────────────┘                  │
│                        │                                    │
│              I2C Bus (SDA:4, SCL:5)                        │
│                        │                                    │
│                   ┌────┴─────┐                             │
│                   │          │                             │
│                   │  ESP32   │                             │
│                   │          │                             │
│  PULSE1 (34) ────►│          │────► PWM1 (32)             │
│  PULSE2 (35) ────►│          │────► PWM2 (33)             │
│  BUTTON (16) ────►│          │────► STATUS_LED (2)        │
│  RS485_RX (14) ───►│          │────► RS485_TX (13)        │
│                   └──────────┘                             │
└─────────────────────────────────────────────────────────────┘
```

---

**Made with ❤️ by NIKOLAINDUSTRY**
