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
