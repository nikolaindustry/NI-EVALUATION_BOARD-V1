#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <PCF8574.h>

// Modbus RTU Function Codes
#define MODBUS_FC_READ_COILS              0x01
#define MODBUS_FC_READ_DISCRETE_INPUTS    0x02
#define MODBUS_FC_READ_HOLDING_REGISTERS  0x03
#define MODBUS_FC_READ_INPUT_REGISTERS    0x04
#define MODBUS_FC_WRITE_SINGLE_COIL       0x05
#define MODBUS_FC_WRITE_SINGLE_REGISTER   0x06
#define MODBUS_FC_WRITE_MULTIPLE_COILS    0x0F
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS 0x10

// Modbus Exception Codes
#define MODBUS_EX_ILLEGAL_FUNCTION        0x01
#define MODBUS_EX_ILLEGAL_DATA_ADDRESS    0x02
#define MODBUS_EX_ILLEGAL_DATA_VALUE      0x03
#define MODBUS_EX_SLAVE_DEVICE_FAILURE    0x04

// Modbus RTU constants
#define MODBUS_MAX_BUFFER                 256
#define MODBUS_RESPONSE_TIMEOUT           1000  // ms
#define MODBUS_FRAME_DELAY                2     // ms (3.5 char times at 9600 baud)

// High-level button events for the onboard push button
enum NIButtonEvent {
    NI_BUTTON_NONE = 0,
    NI_BUTTON_SINGLE,
    NI_BUTTON_DOUBLE,
    NI_BUTTON_TRIPLE,
    NI_BUTTON_LONG
};

// Forward declarations
class NIModbusRTUMaster;
class NIModbusRTUSlave;

// Modbus RTU Master Class
class NIModbusRTUMaster {
public:
    NIModbusRTUMaster(HardwareSerial *serial);
    
    // Core Modbus Functions
    bool readCoils(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint8_t *data);
    bool readDiscreteInputs(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint8_t *data);
    bool readHoldingRegisters(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint16_t *data);
    bool readInputRegisters(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint16_t *data);
    bool writeSingleCoil(uint8_t slaveAddr, uint16_t addr, bool value);
    bool writeSingleRegister(uint8_t slaveAddr, uint16_t addr, uint16_t value);
    bool writeMultipleCoils(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, const uint8_t *data);
    bool writeMultipleRegisters(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, const uint16_t *data);
    
    // Get last exception code
    uint8_t getLastException() { return lastException; }
    
private:
    HardwareSerial *port;
    uint8_t lastException;
    uint8_t txBuffer[MODBUS_MAX_BUFFER];
    uint8_t rxBuffer[MODBUS_MAX_BUFFER];
    
    uint16_t calculateCRC(uint8_t *buffer, uint8_t length);
    bool sendRequest(uint8_t *frame, uint8_t length);
    int receiveResponse(uint8_t expectedAddr, uint8_t expectedFunc, uint32_t timeout);
};

// Modbus RTU Slave Class
class NIModbusRTUSlave {
public:
    NIModbusRTUSlave(HardwareSerial *serial, uint8_t slaveAddress);
    
    // Set slave address
    void setSlaveAddress(uint8_t addr) { slaveAddr = addr; }
    uint8_t getSlaveAddress() { return slaveAddr; }
    
    // Process incoming Modbus requests (call this in loop)
    void process();
    
    // Callback function types
    typedef uint8_t (*ReadCoilCallback)(uint16_t address, bool *value);
    typedef uint8_t (*WriteCoilCallback)(uint16_t address, bool value);
    typedef uint8_t (*ReadRegisterCallback)(uint16_t address, uint16_t *value);
    typedef uint8_t (*WriteRegisterCallback)(uint16_t address, uint16_t value);
    
    // Register callbacks
    void onReadCoil(ReadCoilCallback callback) { readCoilCb = callback; }
    void onWriteCoil(WriteCoilCallback callback) { writeCoilCb = callback; }
    void onReadHoldingRegister(ReadRegisterCallback callback) { readHoldingRegCb = callback; }
    void onReadInputRegister(ReadRegisterCallback callback) { readInputRegCb = callback; }
    void onWriteHoldingRegister(WriteRegisterCallback callback) { writeHoldingRegCb = callback; }
    
private:
    HardwareSerial *port;
    uint8_t slaveAddr;
    uint8_t rxBuffer[MODBUS_MAX_BUFFER];
    uint8_t txBuffer[MODBUS_MAX_BUFFER];
    
    // Callbacks
    ReadCoilCallback readCoilCb;
    WriteCoilCallback writeCoilCb;
    ReadRegisterCallback readHoldingRegCb;
    ReadRegisterCallback readInputRegCb;
    WriteRegisterCallback writeHoldingRegCb;
    
    uint16_t calculateCRC(uint8_t *buffer, uint8_t length);
    void sendResponse(uint8_t *frame, uint8_t length);
    void sendException(uint8_t functionCode, uint8_t exceptionCode);
    
    void handleReadCoils(uint8_t *request, uint8_t requestLen);
    void handleReadDiscreteInputs(uint8_t *request, uint8_t requestLen);
    void handleReadHoldingRegisters(uint8_t *request, uint8_t requestLen);
    void handleReadInputRegisters(uint8_t *request, uint8_t requestLen);
    void handleWriteSingleCoil(uint8_t *request, uint8_t requestLen);
    void handleWriteSingleRegister(uint8_t *request, uint8_t requestLen);
    void handleWriteMultipleCoils(uint8_t *request, uint8_t requestLen);
    void handleWriteMultipleRegisters(uint8_t *request, uint8_t requestLen);
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
    
    // -------------- Modbus RTU (RS485) --------------
    
    // Create Modbus RTU Master instance (returns pointer to manage lifecycle)
    NIModbusRTUMaster* createModbusMaster();
    
    // Create Modbus RTU Slave instance (returns pointer to manage lifecycle)
    NIModbusRTUSlave* createModbusSlave(uint8_t slaveAddress);
    
    // Get RS485 serial port for advanced Modbus usage
    HardwareSerial* getRS485Port() { return rs485Port; }

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
