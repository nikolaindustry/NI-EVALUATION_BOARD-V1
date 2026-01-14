#include "NI-EVALUATION_BOARD-V1.h"

// Library version - increment to force recompilation
#define NI_LIB_VERSION 2

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// ---------- Static hardware objects and state ----------

// PCF8574 expanders
static PCF8574 inputExpander(NIEvaluationBoardV1::INPUT_PCF_ADDR);
static PCF8574 relayExpander(NIEvaluationBoardV1::RELAY_PCF_ADDR);

// Pulse counters (ISR-updated)
static volatile unsigned long pulseCount1 = 0;
static volatile unsigned long pulseCount2 = 0;

// ISR handlers for pulse counting
void IRAM_ATTR isrPulse1() {
    pulseCount1++;
}

void IRAM_ATTR isrPulse2() {
    pulseCount2++;
}

// Button internal state
static bool     btnLastState        = HIGH;
static bool     btnCurrentState     = HIGH;
static bool     btnWaitingForClicks = false;
static uint8_t  btnClickCount       = 0;

static unsigned long btnPressStart      = 0;
static unsigned long btnLastClickTime   = 0;

// Tunable button timing constants
static const unsigned long BTN_LONG_PRESS_MS   = 1500;  // long press threshold
static const unsigned long BTN_DOUBLE_CLICK_MS = 400;   // gap for multi-click

// DAC command and range defines
static const uint8_t CMD_RANGE_CONFIG = 0x01;
static const uint8_t CMD_WRITE_CH0    = 0x02;
static const uint8_t CMD_WRITE_CH1    = 0x04;

static const uint8_t RANGE_0_10V      = 0x11;

// -------------------------------------------------------
// Public: begin
// -------------------------------------------------------
bool NIEvaluationBoardV1::begin(uint32_t rs485Baud,
                                uint32_t pwmFreq,
                                uint8_t pwmResolution) {
    bool ok = true;

    // Store PWM configuration before initializing PWM
    pwmFreqHz = pwmFreq;
    pwmResolutionBits = pwmResolution;

    ok &= initI2C();
    ok &= initExpanders();
    ok &= initDac();

    initPulseInputs();
    initButton();
    initPwm();
    initRs485(rs485Baud);

    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);

    return ok;
}

// -------------------------------------------------------
// Internal: I2C + expanders + DAC
// -------------------------------------------------------
bool NIEvaluationBoardV1::initI2C() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    delay(10);
    return true;
}

bool NIEvaluationBoardV1::initExpanders() {
    bool ok = true;

    // Inputs - PCF8574 library begin() may fail but device still works
    if (!inputExpander.begin()) {
        // Verify device exists on I2C bus
        Wire.beginTransmission(INPUT_PCF_ADDR);
        uint8_t error = Wire.endTransmission();
        if (error != 0) {
            ok = false;
        }
        // Continue anyway - the device may still work
    }
    
    // Configure input pins
    for (uint8_t i = 0; i < 8; i++) {
        inputExpander.pinMode(i, INPUT);
    }

    // Relays - PCF8574 library begin() may fail but device still works
    if (!relayExpander.begin()) {
        // Verify device exists on I2C bus
        Wire.beginTransmission(RELAY_PCF_ADDR);
        uint8_t error = Wire.endTransmission();
        if (error != 0) {
            ok = false;
        }
        // Continue anyway - the device may still work
    }
    
    // Configure relay pins and set to OFF
    relayExpander.pinMode(P3, OUTPUT);
    relayExpander.pinMode(P2, OUTPUT);
    relayExpander.digitalWrite(P3, HIGH);  // HIGH = relay off
    relayExpander.digitalWrite(P2, HIGH);

    return ok;
}

bool NIEvaluationBoardV1::initDac() {
    // Configure 0–10V range
    Wire.beginTransmission(DAC_ADDR);
    Wire.write(CMD_RANGE_CONFIG);
    Wire.write(RANGE_0_10V);
    Wire.endTransmission();
    delay(10);

    // Set both channels to 0 V initially
    setAnalogRaw(0, 0);
    setAnalogRaw(1, 0);

    return true;
}

// -------------------------------------------------------
// Internal: pulse inputs
// -------------------------------------------------------
void NIEvaluationBoardV1::initPulseInputs() {
    pinMode(PIN_PULSE1, INPUT);
    pinMode(PIN_PULSE2, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_PULSE1), isrPulse1, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_PULSE2), isrPulse2, FALLING);
}

// -------------------------------------------------------
// Internal: button
// -------------------------------------------------------
void NIEvaluationBoardV1::initButton() {
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    btnLastState    = digitalRead(PIN_BUTTON);
    btnCurrentState = btnLastState;
    btnWaitingForClicks = false;
    btnClickCount = 0;
}

// -------------------------------------------------------
// Internal: PWM
// -------------------------------------------------------
void NIEvaluationBoardV1::initPwm() {
    if (!ledcAttach(PIN_PWM1, pwmFreqHz, pwmResolutionBits)) {
        // optional debug
    }
    if (!ledcAttach(PIN_PWM2, pwmFreqHz, pwmResolutionBits)) {
        // optional debug
    }

    setPwmDuty(1, 0);
    setPwmDuty(2, 0);
}

void NIEvaluationBoardV1::configurePwm(uint32_t pwmFreq, uint8_t pwmResolution) {
    pwmFreqHz = pwmFreq;
    pwmResolutionBits = pwmResolution;
    initPwm();
}

// -------------------------------------------------------
// Internal: RS485
// -------------------------------------------------------
void NIEvaluationBoardV1::initRs485(uint32_t baud) {
    rs485Port = &Serial2;
    rs485Port->begin(baud, SERIAL_8N1, PIN_RS485_RXD, PIN_RS485_TXD);
}

// -------------------------------------------------------
// Digital inputs
// -------------------------------------------------------
bool NIEvaluationBoardV1::readDigitalInput(uint8_t channel) {
    // Use the inline readInput() function that works correctly
    return readInput(channel);
}

uint8_t NIEvaluationBoardV1::readDigitalInputs() {
    uint8_t mask = 0;
    for (uint8_t i = 0; i < 8; i++) {
        int val = inputExpander.digitalRead(i);
        // Invert logic: set bit when pin is LOW (activated)
        if (!val) {
            mask |= (1u << i);
        }
    }
    return mask;
}

// -------------------------------------------------------
// Relay outputs
// -------------------------------------------------------
void NIEvaluationBoardV1::setRelay(uint8_t channel, bool on) {
    int level = on ? LOW : HIGH;  // LOW = relay ON, HIGH = relay OFF

    switch (channel) {
        case 1:
            relayExpander.digitalWrite(P3, level);
            break;
        case 2:
            relayExpander.digitalWrite(P2, level);
            break;
        default:
            break;
    }
}

void NIEvaluationBoardV1::setAllRelays(bool on) {
    int level = on ? LOW : HIGH;  // LOW = relay ON, HIGH = relay OFF
    relayExpander.digitalWrite(P3, level);
    relayExpander.digitalWrite(P2, level);
}

// -------------------------------------------------------
// DAC: raw + voltage
// -------------------------------------------------------
void NIEvaluationBoardV1::setAnalogRaw(uint8_t channel, uint16_t value) {
    if (value > 4095) value = 4095;

    uint8_t command = (channel == 0) ? CMD_WRITE_CH0 : CMD_WRITE_CH1;

    uint8_t dataLow  = (value << 4) & 0xF0;
    uint8_t dataHigh = (value >> 4) & 0xFF;

    Wire.beginTransmission(DAC_ADDR);
    Wire.write(command);
    Wire.write(dataLow);
    Wire.write(dataHigh);
    Wire.endTransmission();
}

void NIEvaluationBoardV1::setAnalogVoltage(uint8_t channel, float voltage) {
    if (voltage < 0.0f)  voltage = 0.0f;
    if (voltage > 10.0f) voltage = 10.0f;

    uint16_t code = (uint16_t)roundf((voltage / 10.0f) * 4095.0f);
    setAnalogRaw(channel, code);
}

// -------------------------------------------------------
// Pulse counters
// -------------------------------------------------------
unsigned long NIEvaluationBoardV1::getPulseCount(uint8_t channel, bool resetAfterRead) {
    unsigned long value = 0;

    noInterrupts();
    if (channel == 1) {
        value = pulseCount1;
        if (resetAfterRead) pulseCount1 = 0;
    } else if (channel == 2) {
        value = pulseCount2;
        if (resetAfterRead) pulseCount2 = 0;
    }
    interrupts();

    return value;
}

// -------------------------------------------------------
// RS485
// -------------------------------------------------------
size_t NIEvaluationBoardV1::rs485Write(const String &data) {
    if (!rs485Port) return 0;
    return rs485Port->print(data);
}

size_t NIEvaluationBoardV1::rs485WriteLine(const String &line) {
    if (!rs485Port) return 0;
    size_t n = 0;
    n += rs485Port->print(line);
    n += rs485Port->print("\n");
    return n;
}

String NIEvaluationBoardV1::rs485ReadLine(uint32_t timeoutMs) {
    if (!rs485Port) return "";

    unsigned long start = millis();
    String result;

    while (millis() - start < timeoutMs) {
        while (rs485Port->available()) {
            char c = rs485Port->read();
            if (c == '\n') {
                result.trim();
                return result;
            }
            result += c;
        }
    }

    result.trim();
    return result;
}

// -------------------------------------------------------
// Button handling
// -------------------------------------------------------
NIButtonEvent NIEvaluationBoardV1::updateButton() {
    uint32_t now = millis();
    return processButton(now);
}

NIButtonEvent NIEvaluationBoardV1::processButton(uint32_t nowMs) {
    NIButtonEvent evt = NI_BUTTON_NONE;

    btnCurrentState = digitalRead(PIN_BUTTON);

    bool pressed  = (btnCurrentState == LOW);
    bool released = (btnLastState == LOW && btnCurrentState == HIGH);

    if (btnLastState == HIGH && pressed) {
        btnPressStart = nowMs;
    }

    if (released) {
        unsigned long pressDuration = nowMs - btnPressStart;

        if (pressDuration >= BTN_LONG_PRESS_MS) {
            evt = NI_BUTTON_LONG;
            btnWaitingForClicks = false;
            btnClickCount = 0;
        } else {
            btnClickCount++;
            btnLastClickTime = nowMs;
            btnWaitingForClicks = true;
        }
    }

    if (btnWaitingForClicks && (nowMs - btnLastClickTime > BTN_DOUBLE_CLICK_MS)) {
        if (btnClickCount == 1) {
            evt = NI_BUTTON_SINGLE;
        } else if (btnClickCount == 2) {
            evt = NI_BUTTON_DOUBLE;
        } else if (btnClickCount >= 3) {
            evt = NI_BUTTON_TRIPLE;
        }
        btnClickCount = 0;
        btnWaitingForClicks = false;
    }

    btnLastState = btnCurrentState;
    return evt;
}

// -------------------------------------------------------
// PWM
// -------------------------------------------------------
void NIEvaluationBoardV1::setPwmDuty(uint8_t channel, uint8_t duty) {
    if (channel == 1) {
        ledcWrite(PIN_PWM1, duty);
    } else if (channel == 2) {
        ledcWrite(PIN_PWM2, duty);
    }
}

// -------------------------------------------------------
// Modbus RTU Helper Methods
// -------------------------------------------------------
NIModbusRTUMaster* NIEvaluationBoardV1::createModbusMaster() {
    return new NIModbusRTUMaster(rs485Port);
}

NIModbusRTUSlave* NIEvaluationBoardV1::createModbusSlave(uint8_t slaveAddress) {
    return new NIModbusRTUSlave(rs485Port, slaveAddress);
}

// -------------------------------------------------------
// Modbus RTU Master Implementation
// -------------------------------------------------------
NIModbusRTUMaster::NIModbusRTUMaster(HardwareSerial *serial) {
    port = serial;
    lastException = 0;
}

uint16_t NIModbusRTUMaster::calculateCRC(uint8_t *buffer, uint8_t length) {
    uint16_t crc = 0xFFFF;
    
    for (uint8_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)buffer[pos];
        
        for (uint8_t i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

bool NIModbusRTUMaster::sendRequest(uint8_t *frame, uint8_t length) {
    if (!port) return false;
    
    // Calculate and append CRC
    uint16_t crc = calculateCRC(frame, length);
    frame[length] = crc & 0xFF;        // CRC Low
    frame[length + 1] = (crc >> 8) & 0xFF;  // CRC High
    
    // Clear RX buffer
    while (port->available()) {
        port->read();
    }
    
    // Send frame
    port->write(frame, length + 2);
    port->flush();
    
    // Frame delay
    delay(MODBUS_FRAME_DELAY);
    
    return true;
}

int NIModbusRTUMaster::receiveResponse(uint8_t expectedAddr, uint8_t expectedFunc, uint32_t timeout) {
    if (!port) return -1;
    
    unsigned long startTime = millis();
    uint8_t index = 0;
    unsigned long lastByteTime = startTime;
    
    while (millis() - startTime < timeout) {
        if (port->available()) {
            rxBuffer[index++] = port->read();
            lastByteTime = millis();
            
            if (index >= MODBUS_MAX_BUFFER) {
                return -1;  // Buffer overflow
            }
        }
        
        // Check for frame completion (3.5 char silence time)
        if (index > 0 && (millis() - lastByteTime) > MODBUS_FRAME_DELAY) {
            break;
        }
    }
    
    // Validate minimum response length
    if (index < 4) {
        return -1;  // Too short
    }
    
    // Verify CRC
    uint16_t receivedCRC = rxBuffer[index - 2] | (rxBuffer[index - 1] << 8);
    uint16_t calculatedCRC = calculateCRC(rxBuffer, index - 2);
    
    if (receivedCRC != calculatedCRC) {
        return -1;  // CRC error
    }
    
    // Check address
    if (rxBuffer[0] != expectedAddr) {
        return -1;  // Wrong slave
    }
    
    // Check for exception response
    if (rxBuffer[1] == (expectedFunc | 0x80)) {
        lastException = rxBuffer[2];
        return -1;  // Exception
    }
    
    // Check function code
    if (rxBuffer[1] != expectedFunc) {
        return -1;  // Wrong function
    }
    
    return index;  // Success, return frame length
}

bool NIModbusRTUMaster::readCoils(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint8_t *data) {
    if (quantity < 1 || quantity > 2000) return false;
    
    txBuffer[0] = slaveAddr;
    txBuffer[1] = MODBUS_FC_READ_COILS;
    txBuffer[2] = (startAddr >> 8) & 0xFF;
    txBuffer[3] = startAddr & 0xFF;
    txBuffer[4] = (quantity >> 8) & 0xFF;
    txBuffer[5] = quantity & 0xFF;
    
    if (!sendRequest(txBuffer, 6)) return false;
    
    int len = receiveResponse(slaveAddr, MODBUS_FC_READ_COILS, MODBUS_RESPONSE_TIMEOUT);
    if (len < 0) return false;
    
    uint8_t byteCount = rxBuffer[2];
    if (len != (byteCount + 5)) return false;
    
    // Copy coil data
    memcpy(data, &rxBuffer[3], byteCount);
    
    return true;
}

bool NIModbusRTUMaster::readDiscreteInputs(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint8_t *data) {
    if (quantity < 1 || quantity > 2000) return false;
    
    txBuffer[0] = slaveAddr;
    txBuffer[1] = MODBUS_FC_READ_DISCRETE_INPUTS;
    txBuffer[2] = (startAddr >> 8) & 0xFF;
    txBuffer[3] = startAddr & 0xFF;
    txBuffer[4] = (quantity >> 8) & 0xFF;
    txBuffer[5] = quantity & 0xFF;
    
    if (!sendRequest(txBuffer, 6)) return false;
    
    int len = receiveResponse(slaveAddr, MODBUS_FC_READ_DISCRETE_INPUTS, MODBUS_RESPONSE_TIMEOUT);
    if (len < 0) return false;
    
    uint8_t byteCount = rxBuffer[2];
    if (len != (byteCount + 5)) return false;
    
    // Copy input data
    memcpy(data, &rxBuffer[3], byteCount);
    
    return true;
}

bool NIModbusRTUMaster::readHoldingRegisters(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint16_t *data) {
    if (quantity < 1 || quantity > 125) return false;
    
    txBuffer[0] = slaveAddr;
    txBuffer[1] = MODBUS_FC_READ_HOLDING_REGISTERS;
    txBuffer[2] = (startAddr >> 8) & 0xFF;
    txBuffer[3] = startAddr & 0xFF;
    txBuffer[4] = (quantity >> 8) & 0xFF;
    txBuffer[5] = quantity & 0xFF;
    
    if (!sendRequest(txBuffer, 6)) return false;
    
    int len = receiveResponse(slaveAddr, MODBUS_FC_READ_HOLDING_REGISTERS, MODBUS_RESPONSE_TIMEOUT);
    if (len < 0) return false;
    
    uint8_t byteCount = rxBuffer[2];
    if (len != (byteCount + 5)) return false;
    
    // Parse register data (big-endian)
    for (uint16_t i = 0; i < quantity; i++) {
        data[i] = (rxBuffer[3 + i * 2] << 8) | rxBuffer[4 + i * 2];
    }
    
    return true;
}

bool NIModbusRTUMaster::readInputRegisters(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint16_t *data) {
    if (quantity < 1 || quantity > 125) return false;
    
    txBuffer[0] = slaveAddr;
    txBuffer[1] = MODBUS_FC_READ_INPUT_REGISTERS;
    txBuffer[2] = (startAddr >> 8) & 0xFF;
    txBuffer[3] = startAddr & 0xFF;
    txBuffer[4] = (quantity >> 8) & 0xFF;
    txBuffer[5] = quantity & 0xFF;
    
    if (!sendRequest(txBuffer, 6)) return false;
    
    int len = receiveResponse(slaveAddr, MODBUS_FC_READ_INPUT_REGISTERS, MODBUS_RESPONSE_TIMEOUT);
    if (len < 0) return false;
    
    uint8_t byteCount = rxBuffer[2];
    if (len != (byteCount + 5)) return false;
    
    // Parse register data (big-endian)
    for (uint16_t i = 0; i < quantity; i++) {
        data[i] = (rxBuffer[3 + i * 2] << 8) | rxBuffer[4 + i * 2];
    }
    
    return true;
}

bool NIModbusRTUMaster::writeSingleCoil(uint8_t slaveAddr, uint16_t addr, bool value) {
    txBuffer[0] = slaveAddr;
    txBuffer[1] = MODBUS_FC_WRITE_SINGLE_COIL;
    txBuffer[2] = (addr >> 8) & 0xFF;
    txBuffer[3] = addr & 0xFF;
    txBuffer[4] = value ? 0xFF : 0x00;
    txBuffer[5] = 0x00;
    
    if (!sendRequest(txBuffer, 6)) return false;
    
    int len = receiveResponse(slaveAddr, MODBUS_FC_WRITE_SINGLE_COIL, MODBUS_RESPONSE_TIMEOUT);
    if (len < 0) return false;
    
    // Verify echo response
    return (len == 8 && 
            rxBuffer[2] == txBuffer[2] && 
            rxBuffer[3] == txBuffer[3] &&
            rxBuffer[4] == txBuffer[4] &&
            rxBuffer[5] == txBuffer[5]);
}

bool NIModbusRTUMaster::writeSingleRegister(uint8_t slaveAddr, uint16_t addr, uint16_t value) {
    txBuffer[0] = slaveAddr;
    txBuffer[1] = MODBUS_FC_WRITE_SINGLE_REGISTER;
    txBuffer[2] = (addr >> 8) & 0xFF;
    txBuffer[3] = addr & 0xFF;
    txBuffer[4] = (value >> 8) & 0xFF;
    txBuffer[5] = value & 0xFF;
    
    if (!sendRequest(txBuffer, 6)) return false;
    
    int len = receiveResponse(slaveAddr, MODBUS_FC_WRITE_SINGLE_REGISTER, MODBUS_RESPONSE_TIMEOUT);
    if (len < 0) return false;
    
    // Verify echo response
    return (len == 8 && 
            rxBuffer[2] == txBuffer[2] && 
            rxBuffer[3] == txBuffer[3] &&
            rxBuffer[4] == txBuffer[4] &&
            rxBuffer[5] == txBuffer[5]);
}

bool NIModbusRTUMaster::writeMultipleCoils(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, const uint8_t *data) {
    if (quantity < 1 || quantity > 1968) return false;
    
    uint8_t byteCount = (quantity + 7) / 8;
    
    txBuffer[0] = slaveAddr;
    txBuffer[1] = MODBUS_FC_WRITE_MULTIPLE_COILS;
    txBuffer[2] = (startAddr >> 8) & 0xFF;
    txBuffer[3] = startAddr & 0xFF;
    txBuffer[4] = (quantity >> 8) & 0xFF;
    txBuffer[5] = quantity & 0xFF;
    txBuffer[6] = byteCount;
    
    memcpy(&txBuffer[7], data, byteCount);
    
    if (!sendRequest(txBuffer, 7 + byteCount)) return false;
    
    int len = receiveResponse(slaveAddr, MODBUS_FC_WRITE_MULTIPLE_COILS, MODBUS_RESPONSE_TIMEOUT);
    if (len < 0) return false;
    
    return (len == 8);
}

bool NIModbusRTUMaster::writeMultipleRegisters(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, const uint16_t *data) {
    if (quantity < 1 || quantity > 123) return false;
    
    uint8_t byteCount = quantity * 2;
    
    txBuffer[0] = slaveAddr;
    txBuffer[1] = MODBUS_FC_WRITE_MULTIPLE_REGISTERS;
    txBuffer[2] = (startAddr >> 8) & 0xFF;
    txBuffer[3] = startAddr & 0xFF;
    txBuffer[4] = (quantity >> 8) & 0xFF;
    txBuffer[5] = quantity & 0xFF;
    txBuffer[6] = byteCount;
    
    // Pack register data (big-endian)
    for (uint16_t i = 0; i < quantity; i++) {
        txBuffer[7 + i * 2] = (data[i] >> 8) & 0xFF;
        txBuffer[8 + i * 2] = data[i] & 0xFF;
    }
    
    if (!sendRequest(txBuffer, 7 + byteCount)) return false;
    
    int len = receiveResponse(slaveAddr, MODBUS_FC_WRITE_MULTIPLE_REGISTERS, MODBUS_RESPONSE_TIMEOUT);
    if (len < 0) return false;
    
    return (len == 8);
}

// -------------------------------------------------------
// Modbus RTU Slave Implementation
// -------------------------------------------------------
NIModbusRTUSlave::NIModbusRTUSlave(HardwareSerial *serial, uint8_t slaveAddress) {
    port = serial;
    slaveAddr = slaveAddress;
    
    // Initialize callbacks to nullptr
    readCoilCb = nullptr;
    writeCoilCb = nullptr;
    readHoldingRegCb = nullptr;
    readInputRegCb = nullptr;
    writeHoldingRegCb = nullptr;
}

uint16_t NIModbusRTUSlave::calculateCRC(uint8_t *buffer, uint8_t length) {
    uint16_t crc = 0xFFFF;
    
    for (uint8_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)buffer[pos];
        
        for (uint8_t i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

void NIModbusRTUSlave::sendResponse(uint8_t *frame, uint8_t length) {
    if (!port) return;
    
    // Calculate and append CRC
    uint16_t crc = calculateCRC(frame, length);
    frame[length] = crc & 0xFF;
    frame[length + 1] = (crc >> 8) & 0xFF;
    
    // Send response
    port->write(frame, length + 2);
    port->flush();
    
    delay(MODBUS_FRAME_DELAY);
}

void NIModbusRTUSlave::sendException(uint8_t functionCode, uint8_t exceptionCode) {
    txBuffer[0] = slaveAddr;
    txBuffer[1] = functionCode | 0x80;  // Set exception bit
    txBuffer[2] = exceptionCode;
    
    sendResponse(txBuffer, 3);
}

void NIModbusRTUSlave::process() {
    if (!port || !port->available()) return;
    
    // Read incoming frame
    uint8_t index = 0;
    unsigned long lastByteTime = millis();
    
    while (port->available() && index < MODBUS_MAX_BUFFER) {
        rxBuffer[index++] = port->read();
        lastByteTime = millis();
        
        // Wait for more bytes with timeout
        while (!port->available() && (millis() - lastByteTime) < MODBUS_FRAME_DELAY) {
            delayMicroseconds(100);
        }
        
        // Check for end of frame
        if (!port->available() && (millis() - lastByteTime) >= MODBUS_FRAME_DELAY) {
            break;
        }
    }
    
    // Validate minimum frame length
    if (index < 4) return;
    
    // Verify CRC
    uint16_t receivedCRC = rxBuffer[index - 2] | (rxBuffer[index - 1] << 8);
    uint16_t calculatedCRC = calculateCRC(rxBuffer, index - 2);
    
    if (receivedCRC != calculatedCRC) {
        return;  // CRC error, ignore frame
    }
    
    // Check if frame is for this slave
    if (rxBuffer[0] != slaveAddr) {
        return;  // Not for us
    }
    
    // Process function code
    uint8_t functionCode = rxBuffer[1];
    
    switch (functionCode) {
        case MODBUS_FC_READ_COILS:
            handleReadCoils(rxBuffer, index - 2);
            break;
        case MODBUS_FC_READ_DISCRETE_INPUTS:
            handleReadDiscreteInputs(rxBuffer, index - 2);
            break;
        case MODBUS_FC_READ_HOLDING_REGISTERS:
            handleReadHoldingRegisters(rxBuffer, index - 2);
            break;
        case MODBUS_FC_READ_INPUT_REGISTERS:
            handleReadInputRegisters(rxBuffer, index - 2);
            break;
        case MODBUS_FC_WRITE_SINGLE_COIL:
            handleWriteSingleCoil(rxBuffer, index - 2);
            break;
        case MODBUS_FC_WRITE_SINGLE_REGISTER:
            handleWriteSingleRegister(rxBuffer, index - 2);
            break;
        case MODBUS_FC_WRITE_MULTIPLE_COILS:
            handleWriteMultipleCoils(rxBuffer, index - 2);
            break;
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            handleWriteMultipleRegisters(rxBuffer, index - 2);
            break;
        default:
            sendException(functionCode, MODBUS_EX_ILLEGAL_FUNCTION);
            break;
    }
}

void NIModbusRTUSlave::handleReadCoils(uint8_t *request, uint8_t requestLen) {
    if (requestLen != 6) {
        sendException(MODBUS_FC_READ_COILS, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    
    if (!readCoilCb) {
        sendException(MODBUS_FC_READ_COILS, MODBUS_EX_ILLEGAL_FUNCTION);
        return;
    }
    
    uint16_t startAddr = (request[2] << 8) | request[3];
    uint16_t quantity = (request[4] << 8) | request[5];
    
    if (quantity < 1 || quantity > 2000) {
        sendException(MODBUS_FC_READ_COILS, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    
    uint8_t byteCount = (quantity + 7) / 8;
    
    txBuffer[0] = slaveAddr;
    txBuffer[1] = MODBUS_FC_READ_COILS;
    txBuffer[2] = byteCount;
    
    // Read coils using callback
    memset(&txBuffer[3], 0, byteCount);
    
    for (uint16_t i = 0; i < quantity; i++) {
        bool value = false;
        uint8_t result = readCoilCb(startAddr + i, &value);
        
        if (result != 0) {
            sendException(MODBUS_FC_READ_COILS, result);
            return;
        }
        
        if (value) {
            txBuffer[3 + (i / 8)] |= (1 << (i % 8));
        }
    }
    
    sendResponse(txBuffer, 3 + byteCount);
}

void NIModbusRTUSlave::handleReadDiscreteInputs(uint8_t *request, uint8_t requestLen) {
    if (requestLen != 6) {
        sendException(MODBUS_FC_READ_DISCRETE_INPUTS, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    
    if (!readCoilCb) {  // Use same callback as coils for discrete inputs
        sendException(MODBUS_FC_READ_DISCRETE_INPUTS, MODBUS_EX_ILLEGAL_FUNCTION);
        return;
    }
    
    uint16_t startAddr = (request[2] << 8) | request[3];
    uint16_t quantity = (request[4] << 8) | request[5];
    
    if (quantity < 1 || quantity > 2000) {
        sendException(MODBUS_FC_READ_DISCRETE_INPUTS, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    
    uint8_t byteCount = (quantity + 7) / 8;
    
    txBuffer[0] = slaveAddr;
    txBuffer[1] = MODBUS_FC_READ_DISCRETE_INPUTS;
    txBuffer[2] = byteCount;
    
    // Read inputs using callback
    memset(&txBuffer[3], 0, byteCount);
    
    for (uint16_t i = 0; i < quantity; i++) {
        bool value = false;
        uint8_t result = readCoilCb(startAddr + i, &value);
        
        if (result != 0) {
            sendException(MODBUS_FC_READ_DISCRETE_INPUTS, result);
            return;
        }
        
        if (value) {
            txBuffer[3 + (i / 8)] |= (1 << (i % 8));
        }
    }
    
    sendResponse(txBuffer, 3 + byteCount);
}

void NIModbusRTUSlave::handleReadHoldingRegisters(uint8_t *request, uint8_t requestLen) {
    if (requestLen != 6) {
        sendException(MODBUS_FC_READ_HOLDING_REGISTERS, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    
    if (!readHoldingRegCb) {
        sendException(MODBUS_FC_READ_HOLDING_REGISTERS, MODBUS_EX_ILLEGAL_FUNCTION);
        return;
    }
    
    uint16_t startAddr = (request[2] << 8) | request[3];
    uint16_t quantity = (request[4] << 8) | request[5];
    
    if (quantity < 1 || quantity > 125) {
        sendException(MODBUS_FC_READ_HOLDING_REGISTERS, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    
    uint8_t byteCount = quantity * 2;
    
    txBuffer[0] = slaveAddr;
    txBuffer[1] = MODBUS_FC_READ_HOLDING_REGISTERS;
    txBuffer[2] = byteCount;
    
    // Read registers using callback
    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t value = 0;
        uint8_t result = readHoldingRegCb(startAddr + i, &value);
        
        if (result != 0) {
            sendException(MODBUS_FC_READ_HOLDING_REGISTERS, result);
            return;
        }
        
        txBuffer[3 + i * 2] = (value >> 8) & 0xFF;
        txBuffer[4 + i * 2] = value & 0xFF;
    }
    
    sendResponse(txBuffer, 3 + byteCount);
}

void NIModbusRTUSlave::handleReadInputRegisters(uint8_t *request, uint8_t requestLen) {
    if (requestLen != 6) {
        sendException(MODBUS_FC_READ_INPUT_REGISTERS, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    
    if (!readInputRegCb) {
        sendException(MODBUS_FC_READ_INPUT_REGISTERS, MODBUS_EX_ILLEGAL_FUNCTION);
        return;
    }
    
    uint16_t startAddr = (request[2] << 8) | request[3];
    uint16_t quantity = (request[4] << 8) | request[5];
    
    if (quantity < 1 || quantity > 125) {
        sendException(MODBUS_FC_READ_INPUT_REGISTERS, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    
    uint8_t byteCount = quantity * 2;
    
    txBuffer[0] = slaveAddr;
    txBuffer[1] = MODBUS_FC_READ_INPUT_REGISTERS;
    txBuffer[2] = byteCount;
    
    // Read registers using callback
    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t value = 0;
        uint8_t result = readInputRegCb(startAddr + i, &value);
        
        if (result != 0) {
            sendException(MODBUS_FC_READ_INPUT_REGISTERS, result);
            return;
        }
        
        txBuffer[3 + i * 2] = (value >> 8) & 0xFF;
        txBuffer[4 + i * 2] = value & 0xFF;
    }
    
    sendResponse(txBuffer, 3 + byteCount);
}

void NIModbusRTUSlave::handleWriteSingleCoil(uint8_t *request, uint8_t requestLen) {
    if (requestLen != 6) {
        sendException(MODBUS_FC_WRITE_SINGLE_COIL, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    
    if (!writeCoilCb) {
        sendException(MODBUS_FC_WRITE_SINGLE_COIL, MODBUS_EX_ILLEGAL_FUNCTION);
        return;
    }
    
    uint16_t addr = (request[2] << 8) | request[3];
    uint16_t value = (request[4] << 8) | request[5];
    
    if (value != 0x0000 && value != 0xFF00) {
        sendException(MODBUS_FC_WRITE_SINGLE_COIL, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    
    bool coilValue = (value == 0xFF00);
    uint8_t result = writeCoilCb(addr, coilValue);
    
    if (result != 0) {
        sendException(MODBUS_FC_WRITE_SINGLE_COIL, result);
        return;
    }
    
    // Echo request as response
    memcpy(txBuffer, request, 6);
    sendResponse(txBuffer, 6);
}

void NIModbusRTUSlave::handleWriteSingleRegister(uint8_t *request, uint8_t requestLen) {
    if (requestLen != 6) {
        sendException(MODBUS_FC_WRITE_SINGLE_REGISTER, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    
    if (!writeHoldingRegCb) {
        sendException(MODBUS_FC_WRITE_SINGLE_REGISTER, MODBUS_EX_ILLEGAL_FUNCTION);
        return;
    }
    
    uint16_t addr = (request[2] << 8) | request[3];
    uint16_t value = (request[4] << 8) | request[5];
    
    uint8_t result = writeHoldingRegCb(addr, value);
    
    if (result != 0) {
        sendException(MODBUS_FC_WRITE_SINGLE_REGISTER, result);
        return;
    }
    
    // Echo request as response
    memcpy(txBuffer, request, 6);
    sendResponse(txBuffer, 6);
}

void NIModbusRTUSlave::handleWriteMultipleCoils(uint8_t *request, uint8_t requestLen) {
    if (requestLen < 8) {
        sendException(MODBUS_FC_WRITE_MULTIPLE_COILS, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    
    if (!writeCoilCb) {
        sendException(MODBUS_FC_WRITE_MULTIPLE_COILS, MODBUS_EX_ILLEGAL_FUNCTION);
        return;
    }
    
    uint16_t startAddr = (request[2] << 8) | request[3];
    uint16_t quantity = (request[4] << 8) | request[5];
    uint8_t byteCount = request[6];
    
    if (quantity < 1 || quantity > 1968 || byteCount != ((quantity + 7) / 8)) {
        sendException(MODBUS_FC_WRITE_MULTIPLE_COILS, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    
    // Write coils using callback
    for (uint16_t i = 0; i < quantity; i++) {
        bool value = (request[7 + (i / 8)] & (1 << (i % 8))) != 0;
        uint8_t result = writeCoilCb(startAddr + i, value);
        
        if (result != 0) {
            sendException(MODBUS_FC_WRITE_MULTIPLE_COILS, result);
            return;
        }
    }
    
    // Build response
    txBuffer[0] = slaveAddr;
    txBuffer[1] = MODBUS_FC_WRITE_MULTIPLE_COILS;
    txBuffer[2] = request[2];
    txBuffer[3] = request[3];
    txBuffer[4] = request[4];
    txBuffer[5] = request[5];
    
    sendResponse(txBuffer, 6);
}

void NIModbusRTUSlave::handleWriteMultipleRegisters(uint8_t *request, uint8_t requestLen) {
    if (requestLen < 9) {
        sendException(MODBUS_FC_WRITE_MULTIPLE_REGISTERS, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    
    if (!writeHoldingRegCb) {
        sendException(MODBUS_FC_WRITE_MULTIPLE_REGISTERS, MODBUS_EX_ILLEGAL_FUNCTION);
        return;
    }
    
    uint16_t startAddr = (request[2] << 8) | request[3];
    uint16_t quantity = (request[4] << 8) | request[5];
    uint8_t byteCount = request[6];
    
    if (quantity < 1 || quantity > 123 || byteCount != (quantity * 2)) {
        sendException(MODBUS_FC_WRITE_MULTIPLE_REGISTERS, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    
    // Write registers using callback
    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t value = (request[7 + i * 2] << 8) | request[8 + i * 2];
        uint8_t result = writeHoldingRegCb(startAddr + i, value);
        
        if (result != 0) {
            sendException(MODBUS_FC_WRITE_MULTIPLE_REGISTERS, result);
            return;
        }
    }
    
    // Build response
    txBuffer[0] = slaveAddr;
    txBuffer[1] = MODBUS_FC_WRITE_MULTIPLE_REGISTERS;
    txBuffer[2] = request[2];
    txBuffer[3] = request[3];
    txBuffer[4] = request[4];
    txBuffer[5] = request[5];
    
    sendResponse(txBuffer, 6);
}
