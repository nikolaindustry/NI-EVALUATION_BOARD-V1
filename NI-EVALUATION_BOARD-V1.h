#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <PCF8574.h>

// High-level button events for the onboard push button
enum NIButtonEvent {
    NI_BUTTON_NONE = 0,
    NI_BUTTON_SINGLE,
    NI_BUTTON_DOUBLE,
    NI_BUTTON_TRIPLE,
    NI_BUTTON_LONG
};

class NIEvaluationBoardV1 {
public:
    // --- Board-specific defaults ---
    static const uint8_t I2C_SDA_PIN      = 4;
    static const uint8_t I2C_SCL_PIN      = 5;

    static const uint8_t INPUT_PCF_ADDR   = 0x3A; // PCF8574 for digital inputs
    static const uint8_t RELAY_PCF_ADDR   = 0x3C; // PCF8574 for relays
    static const uint8_t DAC_ADDR         = 0x58; // GP8403 DAC

    static const uint8_t PIN_PULSE1       = 34;
    static const uint8_t PIN_PULSE2       = 35;

    static const uint8_t PIN_BUTTON       = 16;

    static const uint8_t PIN_PWM1         = 32;
    static const uint8_t PIN_PWM2         = 33;

    static const uint8_t PIN_STATUS_LED   = 2;

    static const uint8_t PIN_RS485_RXD    = 14;
    static const uint8_t PIN_RS485_TXD    = 13;

    static const uint32_t PWM_FREQ        = 5000;  // 5 kHz (default)
    static const uint8_t  PWM_RESOLUTION  = 8;     // 8-bit (0–255, default)
    
    // Top-level initialization for the whole board
    // Allows optional custom PWM frequency and resolution.
    bool begin(uint32_t rs485Baud = 115200,
               uint32_t pwmFreq = PWM_FREQ,
               uint8_t pwmResolution = PWM_RESOLUTION);

    // -------------- Digital inputs (PCF8574 @ 0x3A) --------------

    // Read one input channel (1..8). Returns true = HIGH, false = LOW.
    bool readDigitalInput(uint8_t channel);
    
    // TEMPORARY: Renamed version to force recompilation
    inline bool readInput(uint8_t channel) {
        if (channel == 0 || channel > 8) return false;
        uint8_t mask = readDigitalInputs();
        return (mask & (1 << (channel - 1))) != 0;
    }

    // Read all 8 digital inputs as a bit mask. Bit 0 = IN1, bit 7 = IN8.
    uint8_t readDigitalInputs();

    // -------------- Relay outputs (PCF8574 @ 0x3C) --------------

    // Set a relay by channel index:
    // channel 1 -> RELAY1 (PCF8574.P3)
    // channel 2 -> RELAY2 (PCF8574.P2)
    void setRelay(uint8_t channel, bool on);

    // Convenience: set both relays
    void setAllRelays(bool on);

    // -------------- DAC 0–10 V (GP8403 @ 0x58) --------------

    // Set raw 12-bit DAC code (0..4095) for channel 0 or 1
    void setAnalogRaw(uint8_t channel, uint16_t value);

    // Set voltage (0.0 .. 10.0 V). Values are clamped to [0, 10].
    void setAnalogVoltage(uint8_t channel, float voltage);

    // -------------- Pulse counters (GPIO 34 / 35) --------------

    // Get pulse count on channel 1 or 2 (mapped to PIN_PULSE1 / PIN_PULSE2).
    // If resetAfterRead = true, counter is set to 0 after reading.
    unsigned long getPulseCount(uint8_t channel, bool resetAfterRead = false);

    // -------------- RS485 (Serial2) --------------

    // Send data as-is (no newline). Returns bytes written.
    size_t rs485Write(const String &data);

    // Send a line terminated with '\n'. Returns bytes written.
    size_t rs485WriteLine(const String &line);

    // Blocking read of a line ending with '\n', or empty string on timeout.
    String rs485ReadLine(uint32_t timeoutMs = 1000);

    // -------------- Push button (GPIO 16) --------------

    // Call this regularly in loop(). It implements debouncing,
    // double-click and long-press detection and returns an event.
    NIButtonEvent updateButton();

    // -------------- PWM outputs (GPIO 32 / 33) --------------

    // Directly set PWM duty (0–255) for channel 1 or 2.
    void setPwmDuty(uint8_t channel, uint8_t duty);
    
    // Optionally reconfigure PWM after begin()
    void configurePwm(uint32_t pwmFreq, uint8_t pwmResolution);

private:
    // Internal helpers
    bool initI2C();
    bool initExpanders();
    bool initDac();
    void initPulseInputs();
    void initButton();
    void initPwm();
    void initRs485(uint32_t baud);

    NIButtonEvent processButton(uint32_t nowMs);
    
    // Current PWM configuration
    uint32_t pwmFreqHz = PWM_FREQ;
    uint8_t  pwmResolutionBits = PWM_RESOLUTION;
    
    // RS485 port (ESP32 Serial2 by default)
    HardwareSerial *rs485Port = nullptr;
};
