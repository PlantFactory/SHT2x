/*
 * SHT2x.cpp
 *
 * Author: Makoto Uju
 * Created: 2016/11/07
 */

#include "SHT2x.h"

#include <math.h>
#include <Arduino.h>
#include <Wire.h>

uint8_t SHT2x::readUserRegistor(void) {
  Wire.beginTransmission(this->ADDR);
  Wire.write(static_cast<uint8_t>(Command::ReadUserRegistor));
  Wire.endTransmission();

  Wire.requestFrom((int)this->ADDR, 1);
  while(!Wire.available()) {}
  return Wire.read();
}

uint8_t SHT2x::writeUserRegistor(uint8_t value) {
  Wire.beginTransmission(this->ADDR);
  Wire.write(static_cast<uint8_t>(Command::WriteUserRegistor));
  Wire.write(value);
  Wire.endTransmission();

  Wire.requestFrom((int)this->ADDR, 1);
  while(!Wire.available()) {}

  return value == Wire.read();
}

uint16_t SHT2x::readValue(uint8_t trigger) {
  uint8_t msb, lsb, crc;

  Wire.beginTransmission(this->ADDR);
  Wire.write(trigger);
  Wire.endTransmission();

  Wire.requestFrom((int)this->ADDR, 3);
  while(!Wire.available()) {}
  msb = Wire.read();
  lsb = Wire.read();
  crc = Wire.read();

  return msb << 8 | lsb;
}

void SHT2x::begin(void){
  reset();
}

void SHT2x::reset(void){
  Wire.beginTransmission(this->ADDR);
  Wire.write(static_cast<uint8_t>(Command::SoftReset));
  Wire.endTransmission();
  delay(15);
}

float SHT2x::readTemperature(uint16_t timeout_ms=SHT2x_DEFAULT_TIMEOUT_MS) {
  uint16_t st = readValue(static_cast<uint8_t>(Command::TemperatureHold));
  st &= ~(0b11U);

  return convertStToTemperature(st);
}

float SHT2x::readHumidity(uint16_t timeout_ms=SHT2x_DEFAULT_TIMEOUT_MS) {
  uint16_t srh = readValue(static_cast<uint8_t>(Command::HumidityHold));
  srh &= ~(0b11U);

  return convertSrhToHumidity(srh);
}

bool SHT2x::heaterOn(void) {
  uint8_t value;

  value = readUserRegistor();
  bitSet(value, 2);
  return value == writeUserRegistor(value);
}

bool SHT2x::heaterOff(void) {
  uint8_t value;

  value = readUserRegistor();
  bitClear(value, 2);
  return value == writeUserRegistor(value);
}

bool SHT2x::otpReloadOff(void) {
  uint8_t value;

  value = readUserRegistor();
  bitSet(value, 1);
  return value == writeUserRegistor(value);
}

bool SHT2x::otpReloadOn(void) {
  uint8_t value;

  value = readUserRegistor();
  bitClear(value, 1);
  return value == writeUserRegistor(value);
}

bool SHT2x::isEOD(void) {
  uint8_t value;

  value = readUserRegistor();
  return bitRead(value, 6);
}

bool SHT2x::setResolution(Resolution resolution) {
  uint8_t value;

  value = readUserRegistor();
  value &= ~0b10000001;
  value |= static_cast<uint8_t>(resolution);
  return value == writeUserRegistor(value);
}
