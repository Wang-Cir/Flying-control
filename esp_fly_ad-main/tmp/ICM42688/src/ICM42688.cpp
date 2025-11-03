#include "ICM42688.h"

ICM42688P::ICM42688P(int csPin, int sckPin, int mosiPin, int misoPin) : _csPin(csPin) {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  _spi = SPI(sckPin, misoPin, mosiPin, csPin);  // 指定SPI引脚
  _spi->begin();
}

void ICM42688P::_writeRegister(uint8_t reg, uint8_t value) {
  digitalWrite(_csPin, LOW);
  _spi->transfer(reg & 0x7F);
  _spi->transfer(value);
  digitalWrite(_csPin, HIGH);
}

uint8_t ICM42688P::_readRegister(uint8_t reg) {
  digitalWrite(_csPin, LOW);
  _spi->transfer(reg | 0x80);
  uint8_t result = _spi->transfer(0x00);
  digitalWrite(_csPin, HIGH);
  return result;
}

void ICM42688P::_readMultipleRegisters(uint8_t reg, uint8_t *buffer, int length) {
  digitalWrite(_csPin, LOW);
  _spi->transfer(reg | 0x80);
  for (int i = 0; i < length; i++) {
    buffer[i] = _spi->transfer(0x00);
  }
  digitalWrite(_csPin, HIGH);
}

void ICM42688P::initialize() {
  _writeRegister(ICM42688_REG_BANK_SEL, 0x00);
  _writeRegister(ICM42688_DEVICE_CONFIG, 0x01);
  delay(100);

  uint8_t whoAmI = _readRegister(ICM42688_WHO_AM_I);
  if (whoAmI != 0x47) {
    Serial.print("Unexpected WHO_AM_I value: 0x");
    Serial.println(whoAmI, HEX);
    while (1);
  }

  _writeRegister(ICM42688_PWR_MGMT0, ICM42688_PWR_TEMP_ON | ICM42688_PWR_GYRO_MODE_LN | ICM42688_PWR_ACCEL_MODE_LN);
  delay(50);

  configureAccelerometer();
  configureGyroscope();
}

void ICM42688P::configureAccelerometer() {
  _writeRegister(0x50, ICM42688_AFS_16G | ICM42688_AODR_1kHz);
}

void ICM42688P::configureGyroscope() {
  _writeRegister(0x4F, ICM42688_GFS_2000DPS | ICM42688_GODR_1kHz);
}

void ICM42688P::readAccelerometer(float &accelX, float &accelY, float &accelZ) {
  uint8_t data[6];
  _readMultipleRegisters(ICM42688_ACCEL_DATA_X1, data, 6);
  int16_t rawX = (data[0] << 8) | data[1];
  int16_t rawY = (data[2] << 8) | data[3];
  int16_t rawZ = (data[4] << 8) | data[5];

  accelX = (rawX / 32768.0) * _accelScale;
  accelY = (rawY / 32768.0) * _accelScale;
  accelZ = (rawZ / 32768.0) * _accelScale;
}

void ICM42688P::readGyroscope(float &gyroX, float &gyroY, float &gyroZ) {
  uint8_t data[6];
  _readMultipleRegisters(ICM42688_GYRO_DATA_X1, data, 6);
  int16_t rawX = (data[0] << 8) | data[1];
  int16_t rawY = (data[2] << 8) | data[3];
  int16_t rawZ = (data[4] << 8) | data[5];

  gyroX = (rawX / 32768.0) * _gyroScale;
  gyroY = (rawY / 32768.0) * _gyroScale;
  gyroZ = (rawZ / 32768.0) * _gyroScale;
}

float ICM42688P::readTemperature() {
  uint8_t data[2];
  _readMultipleRegisters(ICM42688_TEMP_DATA1, data, 2);
  int16_t tempRaw = (data[0] << 8) | data[1];
  return (tempRaw / 132.48) + 25.0;
}