#ifndef SHT2X_H_INCLUDED
#define SHT2X_H_INCLUDED
/*
 * SHT2x.h
 *
 * Author: Makoto Uju
 * Created: 2016/11/07
 */

#include <Arduino.h>
#include <math.h>

#define SHT2x_DEFAULT_TIMEOUT_MS 10

class SHT2x {
  private:
    inline float convertStToTemperature(uint16_t st) {
      return -46.85 + 175.72 * (float)st / pow(2, 16);
    }

    inline float convertSrhToHumidity(uint16_t srh) {
      return -6.0 + 125.0 * (float)srh / pow(2, 16);
    }

  public:
    const uint8_t ADDR = 0b1000000;
    void begin(void);
    void reset(void);

    float readTemperature(uint16_t timeout_ms=SHT2x_DEFAULT_TIMEOUT_MS);
    float readHumidity(uint16_t timeout_ms=SHT2x_DEFAULT_TIMEOUT_MS);

    bool heaterOn(void);
    bool heaterOff(void);

    // float readTemperatureRequest(void) = 0;
    // float readHumidityRequest(void) = 0;
};

#endif
