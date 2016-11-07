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

void SHT2x::begin(void){
  reset();
}

void SHT2x::reset(void){
  Wire.beginTransmission(this->ADDR);
  Wire.write(0xFE);
  Wire.endTransmission();
  delay(15);
}

float SHT2x::readTemperature(uint16_t timeout_ms=SHT2x_DEFAULT_TIMEOUT_MS) {
  uint8_t recv_packet[3];
  uint16_t st;

  Wire.beginTransmission(this->ADDR);
  Wire.write(0b11100011);
  Wire.endTransmission();

  Wire.requestFrom((int)this->ADDR, 3);
  while(!Wire.available()) {}
  recv_packet[0] = Wire.read();
  recv_packet[1] = Wire.read();
  recv_packet[2] = Wire.read();

  st = recv_packet[0] << 8 | recv_packet[1];

  return convertStToTemperature(st);
}

float SHT2x::readHumidity(uint16_t timeout_ms=SHT2x_DEFAULT_TIMEOUT_MS) {
  uint8_t recv_packet[3];
  uint16_t srh;

  Wire.beginTransmission(this->ADDR);
  Wire.write(0b11100101);
  Wire.endTransmission();

  Wire.requestFrom((int)this->ADDR, 3);
  while(!Wire.available()) {}
  recv_packet[0] = Wire.read();
  recv_packet[1] = Wire.read();
  recv_packet[2] = Wire.read();

  srh = recv_packet[0] << 8 | recv_packet[1];

  return convertSrhToHumidity(srh);
}

bool SHT2x::heaterOn(void) {
  uint8_t reg;
  uint8_t res;

  Wire.beginTransmission(this->ADDR);
  Wire.write(0b11100111);
  Wire.endTransmission();

  Wire.requestFrom((int)this->ADDR, 1);
  while(!Wire.available()) {}
  reg = Wire.read();
  bitSet(reg, 2);

  Wire.beginTransmission(this->ADDR);
  Wire.write(0b11100110);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom((int)this->ADDR, 1);
  while(!Wire.available()) {}
  res = Wire.read();

  return res == reg;
}

bool SHT2x::heaterOff(void) {
  uint8_t reg;
  uint8_t res;

  Wire.beginTransmission(this->ADDR);
  Wire.write(0b11100111);
  Wire.endTransmission();

  Wire.requestFrom((int)this->ADDR, 1);
  while(!Wire.available()) {}
  reg = Wire.read();
  bitClear(reg, 2);

  Wire.beginTransmission(this->ADDR);
  Wire.write(0b11100110);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom((int)this->ADDR, 1);
  while(!Wire.available()) {}
  res = Wire.read();

  return res == reg;
}
