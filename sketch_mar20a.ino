#include <SPI.h>
#include <math.h>

constexpr uint8_t HX_DT_PIN = 3;
constexpr uint8_t HX_SCK_PIN = 2;

constexpr uint8_t ADS_CS_PIN = 53;
constexpr uint8_t ADS_DRDY_PIN = 5;
constexpr uint8_t ADS_PDWN_PIN = 4;

enum class OutputDataMode : uint8_t {
  Debug,
  Simple
};

constexpr uint8_t VOLTAGE_PIN = A0;
constexpr bool OUTPUT_TO_SERIAL_PLOTTER = true;
constexpr OutputDataMode OUTPUT_DATA_MODE = OutputDataMode::Simple;

constexpr float HX_SCALE = 88.5f;
constexpr uint8_t HX_TARE_SAMPLES = 30;
constexpr uint8_t HX_READ_SAMPLES = 1;
constexpr unsigned long HX_STARTUP_SETTLE_MS = 1000;

constexpr float DUE_ADC_REF_VOLTS = 3.3f;
constexpr float DUE_ADC_MAX_COUNTS = 4095.0f;
constexpr uint8_t VOLTAGE_AVG_SAMPLES = 1;
constexpr float VOLTAGE_INPUT_SCALE = 10.3114f;  // calibrated from 10.92 V shown to 22.52 V actual

constexpr float ADS_VREF_VOLTS = 2.5f;
constexpr float ADS_PGA = 1.0f;
constexpr uint8_t ADS_CURRENT_CHANNEL = 7;   // AIN7, based on your "A7/GND" wiring note
constexpr uint8_t ADS_DRATE_FAST = 0xB0;     // 2000 SPS for 7.68 MHz clock

constexpr float POWER_PRESENT_THRESHOLD_V = 6.0f;
constexpr float CURRENT_SENSOR_ZERO_VOLTS_DEFAULT = 2.5146f;   // calibrated so ESC idle current reads near 0 A
constexpr float CURRENT_SENSOR_SENSITIVITY_V_PER_A = 0.00962f; // adjusted so 3 A -> ~3 A and 6 A -> ~6 A
constexpr float CURRENT_OFFSET_A = 0.1f;
constexpr float CURRENT_DEAD_ZONE_A = 0.3f;
// CYHCS950-50B5  = 0.0400 V/A
// CYHCS950-100B5 = 0.0200 V/A
// CYHCS950-150B5 = 0.0133 V/A
// CYHCS950-200B5 = 0.0100 V/A

constexpr bool AUTOZERO_CURRENT_SENSOR_AT_STARTUP = false;
constexpr uint8_t CURRENT_ZERO_SAMPLES = 32;
constexpr unsigned long PRINT_PERIOD_MS = 20;

constexpr uint8_t ADS_CMD_RDATA = 0x01;
constexpr uint8_t ADS_CMD_SDATAC = 0x0F;
constexpr uint8_t ADS_CMD_WREG = 0x50;
constexpr uint8_t ADS_CMD_SELFCAL = 0xF0;
constexpr uint8_t ADS_CMD_SYNC = 0xFC;
constexpr uint8_t ADS_CMD_WAKEUP = 0xFF;
constexpr uint8_t ADS_CMD_RESET = 0xFE;

constexpr uint8_t ADS_REG_STATUS = 0x00;
constexpr uint8_t ADS_REG_MUX = 0x01;
constexpr uint8_t ADS_REG_ADCON = 0x02;
constexpr uint8_t ADS_REG_DRATE = 0x03;

SPISettings adsSpiSettings(1900000, MSBFIRST, SPI_MODE1);

bool hxOnline = false;
bool adsOnline = false;
float grams = NAN;
float currentSensorVolts = NAN;
float currentAmps = NAN;
float currentZeroVolts = CURRENT_SENSOR_ZERO_VOLTS_DEFAULT;
float lastA0PinVolts = 0.0f;
float lastA0ScaledVolts = 0.0f;
unsigned long lastPrintMs = 0;
uint8_t currentChannel = ADS_CURRENT_CHANNEL;
bool waitingForChannelDigit = false;
bool powerPresent = false;

struct HX711Reader {
  uint8_t dataPin = 0;
  uint8_t clockPin = 0;
  int32_t offset = 0;
  float scale = 1.0f;

  void begin(uint8_t dtPin, uint8_t sckPin) {
    dataPin = dtPin;
    clockPin = sckPin;

    pinMode(dataPin, INPUT);
    pinMode(clockPin, OUTPUT);
    digitalWrite(clockPin, LOW);
  }

  void set_scale(float newScale) {
    scale = newScale;
  }

  bool is_ready() const {
    return digitalRead(dataPin) == LOW;
  }

  bool wait_ready_timeout(unsigned long timeoutMs) const {
    const unsigned long startedAt = millis();
    while (!is_ready()) {
      if (millis() - startedAt >= timeoutMs) {
        return false;
      }
    }
    return true;
  }

  int32_t read() const {
    uint32_t raw = 0;

    noInterrupts();
    for (uint8_t i = 0; i < 24; ++i) {
      digitalWrite(clockPin, HIGH);
      delayMicroseconds(1);
      raw = (raw << 1) | static_cast<uint32_t>(digitalRead(dataPin));
      digitalWrite(clockPin, LOW);
      delayMicroseconds(1);
    }

    // One extra pulse selects channel A, gain 128 for the next conversion.
    digitalWrite(clockPin, HIGH);
    delayMicroseconds(1);
    digitalWrite(clockPin, LOW);
    interrupts();

    if (raw & 0x800000UL) {
      raw |= 0xFF000000UL;
    }

    return static_cast<int32_t>(raw);
  }

  int32_t read_average(uint8_t samples) const {
    if (samples == 0) {
      samples = 1;
    }

    int64_t sum = 0;
    uint8_t collected = 0;

    while (collected < samples) {
      if (wait_ready_timeout(1000)) {
        sum += read();
        ++collected;
      }
    }

    return static_cast<int32_t>(sum / samples);
  }

  void tare(uint8_t samples) {
    offset = read_average(samples);
  }

  float get_units(uint8_t samples) const {
    if (scale == 0.0f) {
      return NAN;
    }

    return static_cast<float>(read_average(samples) - offset) / scale;
  }
};

HX711Reader scale;

float plotterSafeValue(float value) {
  return isnan(value) ? 0.0f : value;
}

void adsSelect() {
  if (ADS_CS_PIN >= 0) {
    digitalWrite(ADS_CS_PIN, LOW);
  }
}

void adsDeselect() {
  if (ADS_CS_PIN >= 0) {
    digitalWrite(ADS_CS_PIN, HIGH);
  }
}

void adsSendCommand(uint8_t command) {
  SPI.beginTransaction(adsSpiSettings);
  adsSelect();
  SPI.transfer(command);
  adsDeselect();
  SPI.endTransaction();
  delayMicroseconds(5);
}

void adsWriteRegister(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(adsSpiSettings);
  adsSelect();
  SPI.transfer(ADS_CMD_WREG | reg);
  SPI.transfer(0x00);
  SPI.transfer(value);
  adsDeselect();
  SPI.endTransaction();
  delayMicroseconds(5);
}

bool adsWaitForDrdy(unsigned long timeoutMs) {
  const unsigned long startedAt = millis();
  while (digitalRead(ADS_DRDY_PIN) == HIGH) {
    if (millis() - startedAt >= timeoutMs) {
      return false;
    }
  }
  return true;
}

int32_t adsSignExtend24(uint32_t raw24) {
  if (raw24 & 0x800000UL) {
    raw24 |= 0xFF000000UL;
  }
  return static_cast<int32_t>(raw24);
}

float adsRawToVolts(int32_t raw) {
  const float fullScaleVolts = (2.0f * ADS_VREF_VOLTS) / ADS_PGA;
  return (static_cast<float>(raw) * fullScaleVolts) / 8388607.0f;
}

float applyCurrentAdjustments(float currentA) {
  currentA += CURRENT_OFFSET_A;

  if (fabs(currentA) <= CURRENT_DEAD_ZONE_A) {
    return 0.0f;
  }

  return currentA;
}

bool isPowerPresent(float scaledVolts) {
  return scaledVolts >= POWER_PRESENT_THRESHOLD_V;
}

bool adsReadData(int32_t &raw) {
  if (!adsWaitForDrdy(100)) {
    return false;
  }

  SPI.beginTransaction(adsSpiSettings);
  adsSelect();
  SPI.transfer(ADS_CMD_RDATA);
  delayMicroseconds(10);

  const uint32_t b2 = SPI.transfer(0xFF);
  const uint32_t b1 = SPI.transfer(0xFF);
  const uint32_t b0 = SPI.transfer(0xFF);

  adsDeselect();
  SPI.endTransaction();

  raw = adsSignExtend24((b2 << 16) | (b1 << 8) | b0);
  return true;
}

bool adsSelectChannel(uint8_t channel) {
  if (channel > 7) {
    return false;
  }

  adsWriteRegister(ADS_REG_MUX, static_cast<uint8_t>((channel << 4) | 0x08));
  adsSendCommand(ADS_CMD_SYNC);
  adsSendCommand(ADS_CMD_WAKEUP);
  return adsWaitForDrdy(100);
}

bool adsInit() {
  pinMode(ADS_DRDY_PIN, INPUT_PULLUP);
  pinMode(ADS_PDWN_PIN, OUTPUT);

  if (ADS_CS_PIN >= 0) {
    pinMode(ADS_CS_PIN, OUTPUT);
    digitalWrite(ADS_CS_PIN, HIGH);
  }

  digitalWrite(ADS_PDWN_PIN, LOW);
  delay(5);
  digitalWrite(ADS_PDWN_PIN, HIGH);
  delay(10);

  SPI.begin();

  adsSendCommand(ADS_CMD_RESET);
  delay(5);

  if (!adsWaitForDrdy(1000)) {
    return false;
  }

  adsSendCommand(ADS_CMD_SDATAC);

  adsWriteRegister(ADS_REG_STATUS, 0x00);
  adsWriteRegister(ADS_REG_MUX, static_cast<uint8_t>((currentChannel << 4) | 0x08));
  adsWriteRegister(ADS_REG_ADCON, 0x00);
  adsWriteRegister(ADS_REG_DRATE, ADS_DRATE_FAST);

  adsSendCommand(ADS_CMD_SYNC);
  adsSendCommand(ADS_CMD_WAKEUP);
  adsSendCommand(ADS_CMD_SELFCAL);

  return adsWaitForDrdy(1000);
}

bool calibrateCurrentZero() {
  if (!adsOnline) {
    return false;
  }

  const float supplyVolts = readA0PinVolts() * VOLTAGE_INPUT_SCALE;
  if (!isPowerPresent(supplyVolts)) {
    return false;
  }

  float sum = 0.0f;
  uint8_t collected = 0;

  while (collected < CURRENT_ZERO_SAMPLES) {
    int32_t raw = 0;
    if (adsReadData(raw)) {
      sum += adsRawToVolts(raw);
      ++collected;
    }
  }

  currentZeroVolts = sum / static_cast<float>(CURRENT_ZERO_SAMPLES);
  return true;
}

void printCalibrationHelp() {
  Serial.println(F("# Commands:"));
  Serial.println(F("#   w  - tare weight"));
  Serial.println(F("#   i  - zero current sensor at present current"));
  Serial.println(F("#   z  - tare weight and zero current"));
  Serial.println(F("#   c0..c7 - select ADS1256 input channel"));
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());

    if (waitingForChannelDigit) {
      waitingForChannelDigit = false;
      if (command >= '0' && command <= '7') {
        currentChannel = static_cast<uint8_t>(command - '0');
        if (adsOnline && adsSelectChannel(currentChannel)) {
          Serial.print(F("# Current channel set to AIN"));
          Serial.println(currentChannel);
        }
      }
    } else if (command == 'w') {
      if (hxOnline) {
        scale.tare(HX_TARE_SAMPLES);
        Serial.println(F("# Weight tared"));
      }
    } else if (command == 'i') {
      if (adsOnline && calibrateCurrentZero()) {
        Serial.print(F("# Current zero set to "));
        Serial.println(currentZeroVolts, 4);
      }
    } else if (command == 'z') {
      if (hxOnline) {
        scale.tare(HX_TARE_SAMPLES);
      }
      if (adsOnline && calibrateCurrentZero()) {
        Serial.print(F("# Current zero set to "));
        Serial.println(currentZeroVolts, 4);
      }
      Serial.println(F("# Weight tared"));
    } else if (command == 'c') {
      waitingForChannelDigit = true;
    }
  }
}

float readA0PinVolts() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < VOLTAGE_AVG_SAMPLES; ++i) {
    sum += analogRead(VOLTAGE_PIN);
  }

  const float averageCounts = static_cast<float>(sum) / static_cast<float>(VOLTAGE_AVG_SAMPLES);
  return averageCounts * DUE_ADC_REF_VOLTS / DUE_ADC_MAX_COUNTS;
}

void printFloatOrNan(float value, uint8_t digits) {
  if (isnan(value)) {
    Serial.print(F("nan"));
  } else {
    Serial.print(value, digits);
  }
}

void printMonitorLine(float timeSec, float a0PinVolts, float a0ScaledVolts) {
  if (OUTPUT_DATA_MODE == OutputDataMode::Debug) {
    Serial.print(F("time_s:"));
    Serial.print(timeSec, 2);
    Serial.print(F(", weight_g:"));
    printFloatOrNan(grams, 1);
    Serial.print(F(", current_A:"));
    if (powerPresent) {
      printFloatOrNan(currentAmps, 3);
    } else {
      Serial.print(F("нет питания"));
    }
    Serial.print(F(", current_sensor_V:"));
    if (powerPresent) {
      printFloatOrNan(currentSensorVolts, 4);
    } else {
      Serial.print(F("нет питания"));
    }
    Serial.print(F(", a0_pin_V:"));
    Serial.print(a0PinVolts, 4);
    Serial.print(F(", voltage_V:"));
    if (powerPresent) {
      Serial.println(a0ScaledVolts, 4);
    } else {
      Serial.println(F("нет питания"));
    }
  } else {
    Serial.print(F("weight_g:"));
    printFloatOrNan(grams, 1);
    Serial.print(F(", current_A:"));
    if (powerPresent) {
      printFloatOrNan(currentAmps, 3);
    } else {
      Serial.print(F("нет питания"));
    }
    Serial.print(F(", voltage_V:"));
    if (powerPresent) {
      Serial.println(a0ScaledVolts, 4);
    } else {
      Serial.println(F("нет питания"));
    }
  }
}

void printPlotterLine(float a0PinVolts, float a0ScaledVolts) {
  Serial.print(F("weight_g:"));
  Serial.print(plotterSafeValue(grams), 1);
  Serial.print(F(",current_A:"));
  Serial.print(powerPresent ? plotterSafeValue(currentAmps) : 0.0f, 3);

  if (OUTPUT_DATA_MODE == OutputDataMode::Debug) {
    Serial.print(F(",current_sensor_V:"));
    Serial.print(powerPresent ? plotterSafeValue(currentSensorVolts) : 0.0f, 4);
    Serial.print(F(",a0_pin_V:"));
    Serial.print(plotterSafeValue(a0PinVolts), 4);
  }

  Serial.print(F(",voltage_V:"));
  Serial.println(powerPresent ? plotterSafeValue(a0ScaledVolts) : 0.0f, 3);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }

  analogReadResolution(12);
  pinMode(VOLTAGE_PIN, INPUT);

  scale.begin(HX_DT_PIN, HX_SCK_PIN);
  scale.set_scale(HX_SCALE);

  delay(HX_STARTUP_SETTLE_MS);

  if (scale.wait_ready_timeout(2000)) {
    scale.tare(HX_TARE_SAMPLES);
    hxOnline = true;
  } else if (!OUTPUT_TO_SERIAL_PLOTTER) {
    Serial.println(F("# HX711 not ready at startup"));
  }

  adsOnline = adsInit();
  if (!adsOnline) {
    Serial.println(F("# ADS1256 not ready"));
  } else if (AUTOZERO_CURRENT_SENSOR_AT_STARTUP) {
    calibrateCurrentZero();
  }

  if (!OUTPUT_TO_SERIAL_PLOTTER) {
    printCalibrationHelp();
    Serial.println(F("# time_s,weight_g,current_A,current_sensor_V,a0_pin_V,a0_scaled_V"));
  }
}

void loop() {
  handleSerialCommands();

  lastA0PinVolts = readA0PinVolts();
  lastA0ScaledVolts = lastA0PinVolts * VOLTAGE_INPUT_SCALE;
  powerPresent = isPowerPresent(lastA0ScaledVolts);

  if (hxOnline && scale.is_ready()) {
    grams = scale.get_units(HX_READ_SAMPLES);
  }

  if (adsOnline) {
    if (powerPresent) {
      int32_t raw = 0;
      if (adsReadData(raw)) {
        currentSensorVolts = adsRawToVolts(raw);
        currentAmps = applyCurrentAdjustments(
          (currentSensorVolts - currentZeroVolts) / CURRENT_SENSOR_SENSITIVITY_V_PER_A
        );
      }
    } else {
      currentSensorVolts = NAN;
      currentAmps = NAN;
    }
  }

  if (millis() - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = millis();

    const float timeSec = millis() / 1000.0f;
    const float a0PinVolts = lastA0PinVolts;
    const float a0ScaledVolts = lastA0ScaledVolts;

    if (OUTPUT_TO_SERIAL_PLOTTER) {
      printPlotterLine(a0PinVolts, a0ScaledVolts);
    } else {
      printMonitorLine(timeSec, a0PinVolts, a0ScaledVolts);
    }
  }
}
